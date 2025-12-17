// ROS2 Library:L
#include <rclcpp/rclcpp.hpp>
// ROS2 messages:
#include "airplane_gazebo_plugin/msg/airplane_kinetic_model_info.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
// Trigger service in ROS2:
#include <std_srvs/srv/trigger.hpp>
// C++ libraries: 
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <cmath>
#include <Eigen/Dense> 
#include <rclcpp/qos.hpp>


// Start the executable witha  class:
class UAV3DWaypointFollower : public rclcpp::Node
{
public:
    // Define a node as a function witht eh arguments as the topic names:
    UAV3DWaypointFollower(const std::string &model_name, const std::string &waypoints_str) : Node(("uav_waypoint_follower_" + model_name).c_str()), model_name_(model_name)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);


        // Define the topic names using the model name:
        std::string states_topic = model_name_ + "/states";
        std::string cmd_vel_topic = model_name_ + "/cmd_vel";
        std::string complete_topic = model_name_ + "/traj_complete";
        std::string trigger_service = model_name_ + "/trigger_service";


        // Parse the waypoints from a string to a eigen3 vector:
        parse_waypoints(waypoints_str);


        // Subscribe to the states of the model:
        states_sub_ = this->create_subscription<airplane_gazebo_plugin::msg::AirplaneKineticModelInfo>(
            states_topic, qos, std::bind(&UAV3DWaypointFollower::states_callback, this, std::placeholders::_1));
        // Print the name of the states_topic:
        RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", states_topic.c_str());


        // Open a publisher to the commanded velcoity:
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos);
        // Print the comanded velocity topic:
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", cmd_vel_topic.c_str());


        // Publisher to say that this airpalnes complete the trajectory:
        complete_pub_ = this->create_publisher<std_msgs::msg::Bool>(complete_topic, qos);


        // Service to start the waypoint follower:
        start_follow_srv_ = this->create_service<std_srvs::srv::Trigger>(
            trigger_service,
            std::bind(&UAV3DWaypointFollower::trigger_callback, this,
                      std::placeholders::_1, std::placeholders::_2)
        );


        // Subscriber to a complete encouenter if any other airplanes complete the trajectory:
        complete_encounter_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "complete_encounter", qos, std::bind(&UAV3DWaypointFollower::completeEncounterCallback, this, std::placeholders::_1)
        );


    }

    // Function to say when goal is reached:
    bool is_goal_reached() const { return goal_reached_; }

private:

    // Start defining all the private variables used along the code
    // Generate the model name variable:
    std::string model_name_;

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

    // Define the subscriptions and publsiher nodes in the private zone:
    rclcpp::Subscription<airplane_gazebo_plugin::msg::AirplaneKineticModelInfo>::SharedPtr states_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_follow_srv_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_encounter_sub_;



    // Define all the functions used in the executable. 
    // Callback every time the states of the model are published:
    void states_callback(const airplane_gazebo_plugin::msg::AirplaneKineticModelInfo::SharedPtr msg)
    {
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
        actual_state<< msg->north, msg->east, msg->up, msg->course, msg-> fpa, msg->velocity_a, msg-> roll, msg->roll_speed;

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
        // Do a threshold to stop the follower:
        if ((act_mod_pose - last_waypoint_).norm() <= 145){
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


    // Callback to stop the simulation in case other airplane comeplete the encounter;
    void completeEncounterCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // Check your own airplane already reaches the goal:
        if (!msg->data || goal_reached_) return;
        RCLCPP_INFO(this->get_logger(), "[%s] Another airplane completed the encounter!", model_name_.c_str());
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
        const double kp_V = 40; // proportional gain of the velocity
        const double kp_roll = 2.5; //proportional gain of the roll
        const double kd_roll = 0.8; //derivatice gain of the roll
        const double kp_Y = 0.25; // proportional gain of the course
        const double kp_h = 0.4; // proportional gain of the velocity
        const double kp_heading = 0.9; // proportional gain of the heading

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
        roll_cmd = std::clamp(roll_cmd, -37.5/57.3, 37.5/57.3);

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
    std::string model_name = argv[1];
    std::string waypoints_str = argv[2];

    // Put the nod ein the efined waypoint follower class;
    auto node = std::make_shared<UAV3DWaypointFollower>(model_name, waypoints_str);

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