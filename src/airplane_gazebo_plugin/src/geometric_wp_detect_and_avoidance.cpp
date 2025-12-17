// Add ROS2 library
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
// C++ libraries: 
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <cmath>
#include <algorithm> 
#include <Eigen/Dense> 
// ROS2 messages:
#include "airplane_gazebo_plugin/msg/airplane_kinetic_model_info.hpp"
#include "airplane_gazebo_plugin/msg/avoidance_states.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
// Trigger service in ROS2:
#include <std_srvs/srv/trigger.hpp>


// Start the class fo the Geometric DAA executbale:
class GeometricWpDetectAndAvoidance : public rclcpp::Node
{
public:
    // Generate the starter:
    GeometricWpDetectAndAvoidance(const std::string &avoider_name, const std::string &waypoints_str): Node("geometric_wp_detect_and_avoidance"), avoider_name_(avoider_name)
    {
        // Qualioty of service of the messages:
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);


        // Parse teh avoidance waypoints:
        parse_waypoints(waypoints_str);


        // Define the states node of the avoider:
        std::string avoider_topic = avoider_name_ + "/states";
        std::string cmd_vel_topic = avoider_name_ + "/cmd_vel";
        std::string complete_topic = avoider_name_ + "/traj_complete";
        std::string trigger_service = avoider_name_ + "/trigger_service";


        // Subscribe to the states of the avoider;
        avoider_states_sub_ = this->create_subscription<airplane_gazebo_plugin::msg::AirplaneKineticModelInfo>(
            avoider_topic, qos, std::bind(&GeometricWpDetectAndAvoidance::avoider_states_callback, this, std::placeholders::_1));
        // Print the name of the states_topic:
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", avoider_topic.c_str());

        
        // Subscribe tot he obstacle states list:
        obstacles_states_sub_ = this->create_subscription<airplane_gazebo_plugin::msg::AvoidanceStates>(
            "obstacles_states", qos, std::bind(&GeometricWpDetectAndAvoidance::obstacle_states_callback, this, std::placeholders::_1));

        
        // Open a publisher to the commanded velcoity:
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos);
        // Print the comanded velocity topic:
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());


        // Publisher to say that this airpalnes complete the trajectory:
        complete_pub_ = this->create_publisher<std_msgs::msg::Bool>(complete_topic, qos);


        // Service to start the waypoint follower:
        start_follow_srv_ = this->create_service<std_srvs::srv::Trigger>(
            trigger_service,
            std::bind(&GeometricWpDetectAndAvoidance::trigger_callback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );


        // Subscriber to a complete encouenter if any other airplanes complete the trajectory:
        complete_encounter_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "complete_encounter", qos, std::bind(&GeometricWpDetectAndAvoidance::completeEncounterCallback, this, std::placeholders::_1)
        );

    }

    // Function to say when goal is reached:
    bool is_goal_reached() const { return goal_reached_; }

private:

    // Start defining all the private variables used along the code
    // Generate the model name variable:
    std::string avoider_name_;
    // Save the current avoider state at every callback:
    airplane_gazebo_plugin::msg::AirplaneKineticModelInfo avoider_current_state_;

    // Generate the vectors where the waypoints are going to be saved:
    std::vector<double> cmd_vel_;
    // waypoints are (North, East, Down, Command_vel): 
    std::vector<Eigen::Vector3d> waypoints_;
    Eigen::Vector3d last_waypoint_;

    // Define the vraibles of the Waypoint follower to see the next position:
    double transition_radius = 420;
    double look_ahead_distance = 250;

    // Generate a boolean to start the following process with the service:
    bool start_following = false;
    // Variable to see when the UAV reaches the last waypoint:
    bool goal_reached_ = false;
    bool last_published_goal_reached_ = false;

    // Re-initialize the waypoint index changing the MIT encounter set:
    size_t current_idx = 0;

    // Iteration counter:
    int count = 0;
    
    // Boolean to restart the follower and make the past reminder velcoity zero.
    bool have_the_first_valid_msg = false;
    double last_velocity_ = 0.0;

    // Boolean to tell the system start the avoidance:
    bool start_the_avoidance = false; 
    // Define the avoidance last point: 
    Eigen::Vector3d avoidance_last_point_enu;
    // ENd of avoidance arc made for avoidance:
    size_t end_of_arc_ = 0; 



    // GEOMETRIC DETECT AND AVOID VARIABLES:
    // Define the number of conflixts the DAA encounters:
    int conflicts = 0;

    // Critical time to start the avoidance:
    double crit_time_ = 0.1;
    // Minimum transition radius for the avoidance aircraft
    double min_radius = 175;



    // Define the susbcribers or subsctiptions to the topics:
    rclcpp::Subscription<airplane_gazebo_plugin::msg::AirplaneKineticModelInfo>::SharedPtr avoider_states_sub_;
    rclcpp::Subscription<airplane_gazebo_plugin::msg::AvoidanceStates>::SharedPtr obstacles_states_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_follow_srv_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_encounter_sub_;


    // Define all the functions used in the executable.
    // Waypoint follower callback:
    void avoider_states_callback(const airplane_gazebo_plugin::msg::AirplaneKineticModelInfo::SharedPtr msg)
    {
        // Save the actual avoider state:
        avoider_current_state_ = *msg;

        // Only start if the Path follower its started:
        if (!start_following){
            return;
        }

        // Apply the inital velocity
        if (msg->velocity_a < 5.0) {
            geometry_msgs::msg::Twist v0;
            const double v_init = std::max(cmd_vel_.empty() ? 5.0 : cmd_vel_.front(), 5.0);
            v0.linear.x = v_init;
            v0.angular.x = 0.0; v0.angular.y = 0.0; v0.angular.z = 0.0;
            cmd_vel_pub_->publish(v0);
            return;
        }
        
        // Change the order of the states information:
        // Todefine the actual position
        Eigen::Vector3d actual_pose;
        actual_pose << msg->north, msg->east, msg->up;
        // Define a vector as the actual state:
        Eigen::VectorXd actual_state(8);
        actual_state << msg->north, msg->east, msg->up, msg->course, msg-> fpa, msg->velocity_a, msg-> roll, msg->roll_speed;

        // Use the Waypoint follower to obtian the next goal information:
        NextGoalInformation next_goal = WaypointFollower(actual_pose, waypoints_, look_ahead_distance, current_idx, transition_radius);
        // Update the index:
        current_idx = next_goal.act_idx;

        // Define the important information of the goal position:
        double vel_cmd = cmd_vel_[current_idx];
        double course_cmd = next_goal.course_cmd;
        double altitude_cmd = -next_goal.look_ahead_pos(2);
        
        // Use the Small UAV kinematic logic:
        Eigen::VectorXd actual_dstates = FixedWingLogic(actual_state, vel_cmd, course_cmd, altitude_cmd);

         // Add more vlue tot eh counter
        count = count +1;

        // Publish the velocity:
        geometry_msgs::msg::Twist cmd_velocity;

        // Check if it gets near the goal:
        Eigen::Vector3d act_mod_pose = actual_pose;
        act_mod_pose(2) = -act_mod_pose(2); 

        // Identify when the systems gets to the avoidance waypoint:
        if (start_the_avoidance) {
            const Eigen::Vector3d avoidance_last_point_ned = ENU_to_NED(avoidance_last_point_enu); // (N,E,-U)

            if (((act_mod_pose - avoidance_last_point_ned).norm() <= 100.0) || current_idx >= end_of_arc_) {
                start_the_avoidance = false;

                // restore thresholds/params
                crit_time_ = 0.8;
                min_radius = 150;         
                transition_radius = 420;   
                look_ahead_distance = 250;

                RCLCPP_INFO(this->get_logger(), "Avoidance complete; resuming nominal path.");
            }
        }

        // Do a threshold to stop the follower:
        if ((act_mod_pose - last_waypoint_).norm() <= 50){
            goal_reached_ = true;
            std_msgs::msg::Bool finish_follow;
            // send a zero speed before starting a new trajectory
            RCLCPP_INFO(this->get_logger(), "GOAL REACHED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            cmd_velocity.linear.x = 0;
            cmd_velocity.angular.x = 0;
            cmd_velocity.angular.y = 0;
            cmd_velocity.angular.z = 0;
            // publish it:
            for (int i = 0; i < 3; ++i) {
                cmd_vel_pub_->publish(cmd_velocity);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            goal_reached_ = false;
            // Send the required velcoity command:
            double com_linear_vel = std::sqrt(std::max(0.0,
                actual_dstates(0)*actual_dstates(0)+
                actual_dstates(1)*actual_dstates(1)+
                actual_dstates(2)*actual_dstates(2)));
            com_linear_vel = std::clamp(com_linear_vel, 5.0, 420.0);
            cmd_velocity.linear.x = com_linear_vel;
            cmd_velocity.angular.x = actual_dstates(7);
            cmd_velocity.angular.y = -actual_dstates(4);
            cmd_velocity.angular.z = -actual_dstates(3);
            // publish it:
            cmd_vel_pub_->publish(cmd_velocity);
        }
        

        // Publish to the other UAVs that the goal is completed:
        if (goal_reached_ != last_published_goal_reached_) {
            std_msgs::msg::Bool finish_follow;
            finish_follow.data = goal_reached_;
            complete_pub_->publish(finish_follow);
            last_published_goal_reached_ = goal_reached_;
        }
    }


    // Detect and Waypoint follower generation every time obstacles come:
    void obstacle_states_callback(const airplane_gazebo_plugin::msg::AvoidanceStates::SharedPtr msg)
    {
        // Only start if the Path follower its started:
        if (!start_following){
            return;
        }

        // Ownship state (ENU + velocities)
        const double own_e  = avoider_current_state_.east;
        const double own_n  = avoider_current_state_.north;
        const double own_u  = avoider_current_state_.up;
        const double own_ve = avoider_current_state_.v_east;
        const double own_vn = avoider_current_state_.v_north;
        const double own_vu = avoider_current_state_.v_up;
        // Save its actual position and velocit of the ownship:
        Eigen::Vector3d actual_pose;
        actual_pose << own_e, own_n, own_u;
        Eigen::Vector3d actual_vel;
        actual_vel << own_ve, own_vn, own_vu;
        // Course and fligth path angles:
        const double own_course = avoider_current_state_.course;
        const double own_fpa = avoider_current_state_.fpa;

        // Identify the number of conflicts and which are active osbtacles:
        FCABundle active_conflicts = FCA_Detect(*msg, own_e, own_n, own_u, own_ve, own_vn, own_vu);

        // Return in case there is not active conflicts:
        if (active_conflicts.conflicts == 0){
            return;
        }

        // Obtain the future obstacles position:
        std::vector<Eigen::Vector3d> Target_Next_pos = Estimate_target_next_position(active_conflicts,actual_pose,actual_vel);

        // Use a variable the save the overall active obstacles:
        FCABundle overall_obstacles;

        // Overlap obstacle that near each other or inside the minimum radius threshold
        if (active_conflicts.conflicts > 1){
            overall_obstacles = Check_for_overalping(active_conflicts, own_e, own_n, own_u, own_ve, own_vn, own_vu);
            Target_Next_pos = Estimate_target_next_position(overall_obstacles,actual_pose,actual_vel);
        } else {
            overall_obstacles = active_conflicts;
        }

        // for (size_t k = 0; k < Target_Next_pos.size(); ++k) {
        //     const auto &p = Target_Next_pos[k];
        //     RCLCPP_INFO(
        //         this->get_logger(),
        //         "Target %zu next position: East=%f, North=%f, Up=%f",
        //         k, p.x(), p.y(), p.z()
        //     );
        // }

        // Calculate their critical avoidance time to generate the avoiance waypoints:
        avoidance_waypoints(overall_obstacles, own_e, own_n, own_u, own_ve, own_vn, own_vu, own_course, own_fpa, Target_Next_pos);


    }


    // Callback to stop the simulation in case other airplane comeplete the encounter;
    void completeEncounterCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // Check your own airplane already reaches the goal:
        if (!msg->data || goal_reached_) return;
        RCLCPP_INFO(this->get_logger(), "[%s] Another airplane completed the encounter!", avoider_name_.c_str());
        geometry_msgs::msg::Twist cmd_velocity;
        RCLCPP_INFO(this->get_logger(), "GOAL REACHED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        for (int i = 0; i < 3; ++i) {
            cmd_vel_pub_->publish(cmd_velocity);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
         // Optional: stop this follower
        goal_reached_ = true;
        std_msgs::msg::Bool finish_follow;
        finish_follow.data = goal_reached_;
        complete_pub_->publish(finish_follow);
    }


    // Trigger callback to make the waypoint follower to start:
    void trigger_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        // Check if the waypoints are empty to stop the Path follower
        if (waypoints_.empty()){
            response->success = false;
            response->message = "NO Waypoints loaded";
            return;
        }

        start_following = true;
        current_idx = 0;
        response->success = true;
        response->message = "Path following started.";
        RCLCPP_INFO(this->get_logger(), "Path following started.");
    }


    // Fast Detect Avoidance (FCA) Detection outputs:
    struct FCADetect{
        // Index of the intruder:
        std::string aiplane_id;
        // relaive position and oeintation 
        Eigen::Vector3d rel_pos_enu;
        Eigen::Vector3d rel_vel_enu;
        // Avoidance characteristics:
        // relative angle between speed and location:
        double theta_to;
        // MInimum radius in future:
        double r_pca;
        // Minum avoidance radius:
        double dm;
    };

    // Group different FCA Detection Outputs;
    struct FCABundle {
        // Add a number of FCA detection resume:
        std::vector<FCADetect> act_intruders;

        // Number of conflicts start with zero:
        double conflicts = 0;
    };


    // Function to find the intruders that possibly could crash against the ownship:
    FCABundle FCA_Detect(const airplane_gazebo_plugin::msg::AvoidanceStates &intruders,double own_e, double own_n, double own_u,double own_ve, double own_vn, double own_vu)
    {
        // Add a structure to save all teh conflicting intruders:
        FCABundle fca_bundle;
        // Obtain the number of intruders on the message:
        const size_t n = intruders.intruder_states.size();


        // Loop in each of the intruders:
        for (size_t i = 0; i<n; i++){
            // Own velocity norm:
            const double own_vel_a = std::sqrt(own_vn*own_vn+own_ve*own_ve+own_vu*own_vu);
            // Open the data of each intruder:
            const auto &intr = intruders.intruder_states[i];
            const double dm = 1.5*(intr.velocity_a*1.0+125+1.5*6+1.0*own_vel_a);
            const std::string &id = intruders.obstacles_id[i];

            // relative position and velocity:
            Eigen::Vector3d rel_pos(intr.east - own_e,
                                    intr.north - own_n,
                                    intr.up - own_u);

            Eigen::Vector3d rel_vel(-intr.v_east + own_ve,
                                    -intr.v_north + own_vn,
                                    -intr.v_up + own_vu);

            // get the relative position adn velocity norms:
            const double r_rel = rel_pos.norm();
            double v_rel = rel_vel.norm(); 

            // Get te relative angle between the relative position and velcity;
            if (v_rel < 1e-6) v_rel = 1.0;
            const double theta_to = std::acos(std::clamp(rel_pos.dot(rel_vel) / (r_rel * v_rel), -1.0, 1.0));

            // Obtian the closest future relative radius:
            const double r_pca = r_rel * std::sin(theta_to);
            // Analize if hte intruder is an active obstacle:
            if (r_pca < dm){
                ++fca_bundle.conflicts;

                FCADetect det;
                det.aiplane_id = id;
                det.rel_pos_enu = rel_pos;
                det.rel_vel_enu = rel_vel;
                det.theta_to = theta_to;
                det.r_pca = r_pca;
                det.dm = dm;

                fca_bundle.act_intruders.push_back(det);
            }

        }

        // return the dtructure witht he inforamtion of all the active obstacles:
        return fca_bundle;
    }


    // Estimate the targets next position:
    std::vector<Eigen::Vector3d> Estimate_target_next_position(const FCABundle &intruders, const Eigen::Vector3d &act_position, const Eigen::Vector3d &act_vel)
    {
        // Varible where the future target's positions are saved:
        std::vector<Eigen::Vector3d> future_target_pos;
        future_target_pos.reserve(intruders.act_intruders.size());

        for (const auto &o : intruders.act_intruders) {
            // Use the critical minimum time to see the estiamte the future position fo the obstacle
            future_target_pos.emplace_back(o.rel_pos_enu - crit_time_ * (o.rel_vel_enu-act_vel) + act_position);
        }
        return future_target_pos;

    }


    // Funciton to print the ownship psition and the intruders for debbugins
    void print_intruders_and_ownship(const Eigen::Vector3d &own_pos, const Eigen::Vector3d &own_vel, const FCABundle &obstacles)
    {
        // Print ownship state
        RCLCPP_INFO(
            this->get_logger(),
            "OWN Position (ENU):   E=%f  N=%f  U=%f",
            own_pos.x(), own_pos.y(), own_pos.z()
        );

        RCLCPP_INFO(
            this->get_logger(),
            "OWN Velocity (ENU):   VE=%f  VN=%f  VU=%f",
            own_vel.x(), own_vel.y(), own_vel.z()
        );

        // Print each intruder
        for (size_t i = 0; i < obstacles.act_intruders.size(); ++i) {
            const auto &intr = obstacles.act_intruders[i];

            RCLCPP_INFO(
                this->get_logger(),
                "INTRUDER %zu Position (ENU):   E=%f  N=%f  U=%f",
                i, intr.rel_pos_enu.x()+own_pos.x(), intr.rel_pos_enu.y()+own_pos.y(), intr.rel_pos_enu.z()+own_pos.z()
            );

            RCLCPP_INFO(
                this->get_logger(),
                "INTRUDER %zu Velocity (ENU):   VE=%f  VN=%f  VU=%f",
                i, -intr.rel_vel_enu.x()+own_vel.x(), -intr.rel_vel_enu.y()+own_vel.y(), -intr.rel_vel_enu.z()+own_vel.z()
            );
        }
    }


    //  Check for overlaping between the different active osbtacles to fusion them:
    FCABundle Check_for_overalping(const FCABundle &active_obstacles, double own_e, double own_n, double own_u,double own_ve, double own_vn, double own_vu)
    {
        // Start the structure to save the overalping obstacles;
        FCABundle overlaped_obs;

        // Ownship velocity and position information:
        const Eigen::Vector3d act_pos(own_e,own_n,own_u);
        const Eigen::Vector3d act_vel(own_ve,own_vn,own_vu);
        
        // Open the active encounters:
        const auto &obstacles = active_obstacles.act_intruders;
        // Find the number of active obstacles:
        const size_t n = obstacles.size();
        if (n == 0) { overlaped_obs.conflicts = 0; return overlaped_obs; }

        // Loop between the active obstacles:
        for (size_t i = 0; i < n ; ++i){
            // Save the information of the initial position, velocity, and avoidance raius in a variable in case it changes with the overlaping:
            Eigen::Vector3d act_rel_pos = obstacles[i].rel_pos_enu;
            Eigen::Vector3d act_rel_vel = obstacles[i].rel_vel_enu;
            double act_dme = obstacles[i].dm;

            // Loop inside each of the other obstacles:
            for (size_t j=0; j < n; ++j){
                // Jump in case is hte same obstacle as the principal:
                if (j == i) continue;

                // Projected distance between the obstacle j and the actual relative position of the obstacle i
                double proj_j =  obstacles[j].rel_pos_enu.dot(act_rel_pos.normalized());
                // Vector between the the actual osbtacle i to the analized osbtacle j:
                Eigen::Vector3d rel_pos_ij =  act_rel_pos - obstacles[j].rel_pos_enu;

                // Identify if the obstacle j is in the avoidance cone of i:
                double delta_dm = obstacles[j].rel_pos_enu.norm() * std::sin(std::acos(proj_j / obstacles[j].rel_pos_enu.norm())) - proj_j * (act_dme / act_rel_pos.norm());
                bool overlap = false;
                if (proj_j <= act_rel_pos.norm()){
                    overlap = (delta_dm < obstacles[j].dm);
                }else {
                    overlap = (act_dme + obstacles[j].dm > rel_pos_ij.norm());
                }

                // In case they overlap combine them and update the actual aoidance zone:
                if (overlap){
                    act_dme = (act_dme + rel_pos_ij.norm() + obstacles[j].dm) * 0.5;
                    act_rel_pos = obstacles[j].rel_pos_enu + (rel_pos_ij.normalized()) * (act_dme - obstacles[j].dm);

                    // Add the effect of the aded intruder to the obstacle in the velocity:
                    double rel_ij_norm = rel_pos_ij.norm();
                    if (rel_ij_norm > 1e-6) {  // or whatever epsilon you like
                        act_rel_vel += 0.2 * obstacles[j].rel_vel_enu;
                    }
                }
            }

            // Change the characteristics if the avoidancw with respect to the ownship (v_rel, theta_to, r_pca, and nearest crash):
            double theta_to = std::acos(act_rel_pos.dot(act_rel_vel)/(act_rel_pos.norm()*act_rel_vel.norm()));
            double r_pca = act_rel_pos.norm() * std::sin(theta_to);

            // Save the information in the output structure after is overlaped:
            ++overlaped_obs.conflicts;
            FCADetect det;
            det.aiplane_id = obstacles[i].aiplane_id;
            det.rel_pos_enu = act_rel_pos;
            det.rel_vel_enu = act_rel_vel;
            det.theta_to = theta_to;
            det.r_pca = r_pca;
            det.dm = act_dme;

            overlaped_obs.act_intruders.push_back(det);

        }

        return overlaped_obs;
        
    }


    // ENU -> NED conversion
    inline static Eigen::Vector3d ENU_to_NED(const Eigen::Vector3d &enu) {
        return Eigen::Vector3d(enu.y(), enu.x(), -enu.z()); // (N,E,-U)
    }

    
    // Generate rotattion matrix in the x axis in degrees:
    inline static Eigen::Matrix3d Rx_deg(double deg) {
        const double r = deg * M_PI / 180.0;
        const double c = std::cos(r), s = std::sin(r);
        Eigen::Matrix3d R;
        R << 1, 0, 0,
            0, c,-s,
            0, s, c;
        return R;
    }


    // Identify the risker obstacle using the critical time of avoidance and generate teh avoidance waypoints:
    void avoidance_waypoints(const FCABundle &overall_obstacles, double own_e, double own_n, double own_u,double own_ve, double own_vn, double own_vu, double own_course, double own_fpa, const std::vector<Eigen::Vector3d> &future_target_pos)
    {
        // Ownship velocity and position information:
        const Eigen::Vector3d act_pos(own_e,own_n,own_u);
        const Eigen::Vector3d act_vel(own_ve,own_vn,own_vu);
        
        // Open the active encounters:
        const auto &act_obstacles = overall_obstacles.act_intruders;
        // Find the number of active obstacles:
        const size_t n = act_obstacles.size();
        if (n == 0) { return; }

        // Variable to save the minimum critical time:
        double min_crit_time = 0;
        int count_time = 0;
        // Iidentify the critical target
        int crit_target_idx = 0;

        for (size_t i = 0; i < n ; ++i){
            // Caclulate the critical time of each target:
            double act_crit_time = (act_obstacles[i].rel_pos_enu.norm()*(std::cos(act_obstacles[i].theta_to)+std::sin(act_obstacles[i].theta_to))-min_radius-act_obstacles[i].dm)/(act_obstacles[i].rel_vel_enu.norm());
            // check if one of the airplanes that was already avoided is giving problems by staying in a negative avoidance zone:
            if (act_crit_time < -1.5){continue;}
            if (count_time == 0 || act_crit_time < min_crit_time){
                count_time = 1;
                min_crit_time = act_crit_time; 
                crit_target_idx = i;
            }
        }

        // RCLCPP_INFO(this->get_logger(), "critical time = %f", min_crit_time);
        

        // Add the avoidance waypoints in case the critical time is small:
        // return in case the minimum critical time is not enough
        if (start_the_avoidance) {
            RCLCPP_INFO(this->get_logger(), "The avoidance process started!!!");
            return;
        }                   
        if (!std::isfinite(min_crit_time)||count_time==0) return;
        if (min_crit_time > crit_time_) return;

        // Start the avoidance in case it is lower than the critical time:
        start_the_avoidance = true;
        transition_radius = 200;
        look_ahead_distance = 150;

        // Modify the waypoints:
        // Identify the actual avoidance obstacle characteristics:
        Eigen::Vector3d avoidance_center = act_obstacles[crit_target_idx].rel_pos_enu + act_pos;
        Eigen::Vector3d crit_next_pos = future_target_pos[crit_target_idx];
        double dm_avoid = act_obstacles[crit_target_idx].dm;
        // Use the ownship inforamtion to generate the avoiance zone:
        const double own_yaw = M_PI / 2 - own_course;
        const double own_pitch = own_fpa;

        // Generate the avoidance zone:
        // Deine the normal avoidance circle normal to the velcoity of the ownship:
        int N = 50;
        // Rotation matrices:
        Eigen::Matrix3d Rz, Ry;
        Rz << std::cos(own_yaw), -std::sin(own_yaw), 0,
            std::sin(own_yaw),  std::cos(own_yaw), 0,
                    0 ,        0 , 1;
        Ry << std::cos(-own_pitch), 0, std::sin(-own_pitch),
            0,                 1, 0,
            -std::sin(-own_pitch), 0, std::cos(-own_pitch);
        Eigen::Matrix3d Rotation_matrix = Rz * Ry;
        // Generate the circle around the intruder:
        std::vector<Eigen::Vector3d> circle;
        circle.reserve(N);
        for (int i = 0; i < N; i++)
        {
            double theta = 2.0 * M_PI * i / N;
            double y = std::cos(theta);
            double z = std::sin(theta);
            Eigen::Vector3d point(0.0, y, z);
            Eigen::Vector3d rotated_point = dm_avoid * Rotation_matrix * point + avoidance_center;
            circle.push_back(rotated_point);
        }

        // Optional print to see hte avdoaince zone and radius:
        RCLCPP_INFO(this->get_logger(), "dm_avoid = %.3f", dm_avoid);
        RCLCPP_INFO(this->get_logger(), "avoidance_center = [%.3f, %.3f, %.3f]",
                    avoidance_center.x(), avoidance_center.y(), avoidance_center.z());


        // Identify which Waypoint is going to be at the end of the avoidance
        avoidance_last_point_enu = avoidance_center + act_vel.normalized() * dm_avoid;
        // pass the avoidance to point from ENU to NED:
        Eigen::Vector3d avoidance_last_point_ned=  ENU_to_NED(avoidance_last_point_enu);
        // Find the nearest waypoint:
        if (waypoints_.empty()) {return;}
        // Index of the nearest waypoint and actual distance to the las avoidance position:
        size_t near_wp_idx = current_idx;
        double best_d2 = (waypoints_[near_wp_idx] - avoidance_last_point_ned).squaredNorm();

        if (near_wp_idx + 1 < waypoints_.size()) {
            for (size_t i = near_wp_idx + 1; i < waypoints_.size(); ++i) {
                const double d2 = (waypoints_[i] - avoidance_last_point_ned).squaredNorm();
                if (d2 < best_d2) {
                    best_d2 = d2;
                    near_wp_idx = i;
                }
            }
        }
        // Insert avoidance waypoints:
        const size_t insert_at_wp = std::min(current_idx + 1, waypoints_.size());

        // Use a cost fucntion to identify the rigth trajectory in the avoidance zone:
        double min_cost = -1.0;
        size_t best_avoid_idx = 0;
        for (int i = 0; i < N; ++i)
        {
            double d_alt = circle[i].z() - act_pos.z();
            double d_north = circle[i].y() - act_pos.y();
            double d_east = circle[i].x() - act_pos.x();
            double horiz = std::max(std::sqrt(d_north*d_north + d_east*d_east), 1e-6);
            double d_pitch = std::atan2(d_alt, horiz);
            double d_yaw   = std::atan2(d_east, std::max(d_north, 1e-9)); // yaw-from-East

            // Get a ratio that depends on the next posiiton orientation respct to teh pincipal avoidance circle position:
            Eigen::Vector3d r_avo_fut = crit_next_pos - avoidance_center;
            Eigen::Vector3d r_avo_circ = circle[i]  - avoidance_center;
            double value_1 = std::abs(std::acos(r_avo_fut.dot(r_avo_circ)/(r_avo_fut.norm()*r_avo_circ.norm())));
            

            double cost;
            if ((d_pitch - own_pitch) < 0.0) {
                cost = ((1.0/(value_1+0.1)) + 2.4) * std::abs(d_pitch)
                    + (1.0/(value_1+0.1)) * std::abs(d_yaw);
            } else {
                cost = ((1.0/(value_1+0.1)) + 1.2) * std::abs(d_pitch)
                    + (1.0/(value_1+0.1)) * std::abs(d_yaw);
            }

            if (min_cost < 0.0 || cost < min_cost) {
                min_cost = cost;
                best_avoid_idx = i;
            }
        }

        // Generate the avoidance trajectory:
        Eigen::Vector3d principal_avoidance_point_enu = (circle[best_avoid_idx]-avoidance_center).normalized();
        Eigen::Vector3d own_vel_norm = act_vel.normalized();
        Eigen::Vector3d ort_norm = own_vel_norm.cross(principal_avoidance_point_enu);
        if (ort_norm.norm() < 1e-6)
        {
            Eigen::Vector3d tmp(1,0,0);
            if (std::abs(act_vel.dot(tmp)) > 0.9) tmp = Eigen::Vector3d(0,1,0);
            ort_norm = act_vel.cross(tmp);
        }
        ort_norm.normalize();
        // Regenerate principal_avoidance_point_enu in case of almost parallel rotationl axis:
        Eigen::Vector3d principal_avoidance_point_enu_new = ort_norm.cross(own_vel_norm).normalized();
        // Rotational matrix where the principal point rotates:
        Eigen::Matrix3d P;
        P.col(0) = ort_norm;
        P.col(1) = own_vel_norm;
        P.col(2) = principal_avoidance_point_enu_new;
        // Anilize if the rotation needs to go from +/- 90 deg using the closests avoidance point:
        const Eigen::Vector3d cand_p  = avoidance_center + dm_avoid * (P * Rx_deg( +90.0) * P.transpose() * principal_avoidance_point_enu_new);
        const Eigen::Vector3d cand_m  = avoidance_center + dm_avoid * (P * Rx_deg( -90.0) * P.transpose() * principal_avoidance_point_enu_new);
        const double d1 = (avoidance_last_point_enu - cand_p).squaredNorm();
        const double d2 = (avoidance_last_point_enu - cand_m).squaredNorm();
        // Generate the avodiance trajectory using the proper direction:
        std::array<double,7> phi = (d1<d2)
            ? std::array<double,7>{-60.0,-30.0,-15.0,0.0,15.0,30.0,60.0}
            : std::array<double,7>{60.0,30.0,15.0,0.0,-15.0,-30.0,-60.0};
        // Vector to save the avoidance waypoint path:
        std::vector<Eigen::Vector3d> avoidance_waypoints; avoidance_waypoints.reserve(7);
        for (double angle : phi)
        {
            Eigen::Vector3d avoid_point = dm_avoid * (P * Rx_deg(angle) * P.transpose() * principal_avoidance_point_enu_new) + avoidance_center;
            avoidance_waypoints.push_back(ENU_to_NED(avoid_point));
        }

        // Insert the 7 waypoints in the waypoint list:
        waypoints_.insert(waypoints_.begin() + static_cast<long>(insert_at_wp), avoidance_waypoints.begin(), avoidance_waypoints.end());
        // Add velocities as the akst velocity multiplied:
        const double keep_speed = (insert_at_wp < cmd_vel_.size()) ? cmd_vel_[insert_at_wp] : last_velocity_;
        cmd_vel_.insert(cmd_vel_.begin() + static_cast<long>(insert_at_wp), 7, keep_speed);
        const size_t arc_len = avoidance_waypoints.size(); // 7
        end_of_arc_ = insert_at_wp + arc_len - 1;
        // eliminate the middle waypoints in the avoidance path;
        size_t rejoin_idx_new = near_wp_idx;
        if (near_wp_idx >= insert_at_wp) {
            rejoin_idx_new += arc_len;
        }
        if (rejoin_idx_new > end_of_arc_ + 1 && rejoin_idx_new <= waypoints_.size()) {
            auto wp_erase_begin = waypoints_.begin() + static_cast<long>(end_of_arc_ + 1);
            auto wp_erase_end   = waypoints_.begin() + static_cast<long>(rejoin_idx_new);
            waypoints_.erase(wp_erase_begin, wp_erase_end);

            // keep cmd_vel_ in sync
            if (cmd_vel_.size() >= rejoin_idx_new) {
                auto sp_erase_begin = cmd_vel_.begin() + static_cast<long>(end_of_arc_ + 1);
                auto sp_erase_end   = cmd_vel_.begin() + static_cast<long>(rejoin_idx_new);
                cmd_vel_.erase(sp_erase_begin, sp_erase_end);
            }
        }

        // Update the current Idx:
        current_idx = insert_at_wp;

    }


    // Change the Waypoints from a string to a vector
    void parse_waypoints(const std::string &waypoints_str)
    {
        std::stringstream ss(waypoints_str);
        std::string segment;

        // loop to separe teh waypoints:
        while (std::getline(ss, segment, ';')){
            std::stringstream coords(segment);
            std::string val;

            // place to save the waypoints and commanded speed
            Eigen::Vector3d wp;
            double vel = 0.0;
            int i = 0;
            while (std::getline(coords, val, ',')){
                double num = std::stod(val);
                if (i < 3) {
                    wp(i) = num;
                } else if (i == 3) {
                    vel = num;
                }
                i++;
            }
            // Store it in the matrices:
            waypoints_.push_back(wp);
            cmd_vel_.push_back(vel);
        }

        // Save the last waypoint
        if (!waypoints_.empty()){
            last_waypoint_ = waypoints_.back();
        }
    }


    // Use a structure to define the next states goal:
    struct NextGoalInformation
    {
        double course_cmd;
        Eigen::Vector3d look_ahead_pos;
        int act_idx;
    };


    // Waypoint follower algorithm:
    NextGoalInformation WaypointFollower(const Eigen::Vector3d &pose, const std::vector<Eigen::Vector3d> &Waypoints, double look_ahead_distance,
                                    int init_idx, double transition_radius)
    {
        // Pass the position to be in the NED axes:
        Eigen::Vector3d mod_pose = pose;
        mod_pose(2) = -mod_pose(2);

        // Find the Actual idx in which the Airplane is flying using teh Waypoint Hyperplane ondition:
        int max_idx = static_cast<int>(Waypoints.size()) - 2;
        int act_idx = init_idx;
        for (int i = init_idx; i < max_idx; i++)
        {
            if ((mod_pose - Waypoints[i]).dot(Waypoints[i + 1] - Waypoints[i]) < 0){
                act_idx = i;
                if (act_idx >= max_idx)
                    act_idx = max_idx;
                break;
            }
        }
        // Check if the position is at near the next waypoint using the transition radius:
        if (act_idx < max_idx && (mod_pose - Waypoints[act_idx + 1]).norm() <= transition_radius)
        {
            act_idx++;
        }

        // Obtain the unitary vector that defines the direction from the pass waypoin tot hte next waypoint:
        Eigen::Vector3d past_goal = Waypoints[act_idx];
        Eigen::Vector3d goal = Waypoints[act_idx + 1];
        Eigen::Vector3d path_vec = goal - past_goal;
        const double seg_len = path_vec.norm();
        if (seg_len < 1e-6){
            path_vec = Eigen::Vector3d(1.0, 0.0, 0.0);
        } else{
            path_vec /= seg_len;
        }

        // Generate the projected pose on the path_vec trjectory and adding the look ahead distance:
        Eigen::Vector3d past_2_pos = mod_pose - past_goal;
        double proj_dist = past_2_pos.dot(path_vec);
        Eigen::Vector3d proj_pose = past_goal + path_vec * proj_dist;
        Eigen::Vector3d look_ahead_pos = proj_pose + look_ahead_distance * path_vec;

        // Identify the commanded course using the North and East poistion of the look ahead position and the actual position:
        Eigen::Vector3d delta = look_ahead_pos - mod_pose;
        double course_cmd = std::atan2(delta(1),delta(0));

        return {course_cmd, look_ahead_pos, act_idx};
    }


     // Small Fixed-wing UAV Kimenatic model:
    Eigen::VectorXd FixedWingLogic(const Eigen::VectorXd &state, double vel_cmd, double course_cmd, double alt_cmd)
    {
        // Define the constant values
        const double g = 9.81;   // gravity
        const double kp_V = 6.7; // proportional gain of the velocity
        const double kp_roll = 2.5; //proportional gain of the roll
        const double kd_roll = 0.8; //derivatice gain of the roll
        const double kp_Y = 0.25; // proportional gain of the course
        const double kp_h = 0.4; // proportional gain of the velocity
        const double kp_heading = 0.8; // proportional gain of the heading

        // Start teh derivative vector:
        Eigen::VectorXd dstate(8);

        // Define the velocity:
        double V = state(5);
        // Check the velocity and add aminimum vsalue in case is zero:
        if (!std::isfinite(V) || V < 1.0) {
            V = 1.0;
        }

        // Obtain the commanded roll:
        double roll_cmd = atan2(kp_heading * angdiff(course_cmd,state(3)) * V, g);
        // Limit the commanded roll:
        roll_cmd = std::clamp(roll_cmd, -35.0/57.3, 35.0/57.3);

        // Obtian the FPA command:
        double alt_diff = kp_h * (alt_cmd - state(2));
        alt_diff = std::clamp(alt_diff, -V, V);
        double Y_cmd = asin((1.0 / V) * alt_diff);

        // Genate the derivative vector:
        dstate(0) = V * cos(state(3)) * cos(state(4));
        dstate(1) = V * sin(state(3)) * cos(state(4));
        dstate(2) = V * sin(state(4));
        dstate(3) = (g * tan(state(6))) / V;
        dstate(4) = kp_Y * (Y_cmd - state(4));
        dstate(5) = kp_V * (vel_cmd - V);
        dstate(6) = state(7);
        dstate(7) = kp_roll * (roll_cmd - state(6)) - kd_roll * state(7);
        return dstate;

    }


    // generate an absolute angle difference calcualtion:
    inline double angdiff(double a, double b) {
        // define values of and b in teh range 0 to 2pi
        double abs_a = a;
        while (abs_a > 2.0 * M_PI) abs_a -= 2.0 * M_PI;
        while (abs_a < 0.0) abs_a += 2.0 * M_PI;
        double abs_b = b;
        while (abs_b > 2.0 * M_PI) abs_b -= 2.0 * M_PI;
        while (abs_b < 0.0) abs_b += 2.0 * M_PI;
        // obtain the difference
        double diff = abs_a - abs_b;
        if (diff > M_PI)  diff -= 2.0 * M_PI;
        if (diff < -M_PI) diff += 2.0 * M_PI;
        return diff;
    }
};




// Main logic of the executable:
int main(int argc, char **argv)
{
    // Start the ROS2 node:
    rclcpp::init(argc, argv);

    // Safety check that all teh arguments are being inputed:
    if (argc < 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                     "Usage: ros2 run <package_name> <executable_name> <model_name> <waypoints_string>");
        return 1;
    }

    // Obtain the arguments:
    std::string avoider_name = argv[1];
    std::string waypoints_str = argv[2];

    // Put the nod ein the efined waypoint follower class;
    auto node = std::make_shared<GeometricWpDetectAndAvoidance>(avoider_name, waypoints_str);

    // Spin the node:
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && !node->is_goal_reached()){
        exec.spin_once();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // Shutdown ROS 2:
    rclcpp::shutdown();
    return 0;
}



































































