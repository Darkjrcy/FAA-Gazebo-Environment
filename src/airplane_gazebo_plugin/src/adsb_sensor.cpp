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
#include <memory>   // for std::make_shared
#include <Eigen/Dense>

// ROS2 messages of the GNSS and the IMU:
#include "gnss_multipath_plugin/msg/gnss_multipath_fix.hpp"
#include "airplane_gazebo_plugin/msg/airplane_kinetic_model_info.hpp"
#include "airplane_gazebo_plugin/msg/adsb_data.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
// Trigger service in ROS2:
#include <std_srvs/srv/trigger.hpp>

// Start the class of the ADSB Sensor Executable:
class ADSBSensor : public rclcpp::Node
{
public:
    // Constructor:
    ADSBSensor(const std::string &model_name, const std::string &imu_name)
    : Node(("adsb_sensor_" + model_name).c_str()), model_name_(model_name), imu_name_(imu_name)
    {
        // QoS for the custom plugins:
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);
        // QoS for the IMU sensor:
        auto imu_qos = rclcpp::QoS(10).reliable().durability_volatile();

        // Define the GNSS topic (note the extra '/'):
        std::string gnss_topic = "/gnss_multipath/" + model_name_ + "/gnss_multipath_fix";
        // Define the IMU topic:
        std::string imu_topic  = "/" + imu_name_ + "/out";
        // Define the adsb topic where the adsb info is going to be published:
        std::string adsb_topic = model_name_ + "/adsb_out";
        // Start the adsb recollection :
        std::string trigger_service_start = model_name_ + "/adsb_start_service";
        std::string trigger_service_stop = model_name_ + "/adsb_stop_service";
        // Topic of the states the UAV:
        std::string states_topic = model_name_ + "/states";

        // Subscribe to the IMU topic:
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos, std::bind(&ADSBSensor::imu_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to imu topic: %s", imu_topic.c_str());

        // Subscribe to the GNSS data (no-op callback while debugging):
        gnss_sub_ = this->create_subscription<gnss_multipath_plugin::msg::GNSSMultipathFix>(
            gnss_topic, qos, std::bind(&ADSBSensor::gnss_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to gnss topic: %s", gnss_topic.c_str());

        // Subscribe to the states of the model:
        states_sub_ = this->create_subscription<airplane_gazebo_plugin::msg::AirplaneKineticModelInfo>(
            states_topic, qos, std::bind(&ADSBSensor::states_callback, this, std::placeholders::_1));

        // Publish the ADSB data so other systems can know about it:
        adsb_pub_ = this->create_publisher<airplane_gazebo_plugin::msg::AdsbData>(adsb_topic, qos);

        // Start the ADS-B publication:
        start_adsb_srv_ = this->create_service<std_srvs::srv::Trigger>(
            trigger_service_start,
            std::bind(&ADSBSensor::trigger_callback_start, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // Stop the ADS-B publication:
        stop_adsb_srv_ = this->create_service<std_srvs::srv::Trigger>(
            trigger_service_stop,
            std::bind(&ADSBSensor::trigger_callback_stop, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        // initialize the EKF matrices:
        Ak.setIdentity();                    
        Ck << 1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0;                   

        Q = (Mat66() <<
            4,0,0,0,0,0,
            0,4,0,0,0,0,
            0,0,4,0,0,0,
            0,0,0,100,0,0,
            0,0,0,0,100,0,
            0,0,0,0,0,100).finished();      

        R = (Eigen::Matrix3d() <<
            5000,0,0,
            0,5000,0,
            0,0,5000).finished();

    }

private:
    // Private variables:
    std::string model_name_;
    std::string imu_name_;

    // Define matrix type 6x6:
    using Mat66 = Eigen::Matrix<double,6,6>;

    // Last roll pitch and yaw obtained:
    double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
    double course;

    // Get the initial position:
    double x0 = 0.0;
    double y0 = 0.0;
    double z0 = 0.0;
    // Get the initial speed:
    double v0 = -1.0;
    double vx0 = 0.0;
    double vy0 = 0.0;
    double vz0 = 0.0;
    // Initialize the initial state covatance matrix:
    Mat66 P0; 

    // Position measurements made by the GNSS:
    double x_mes = 0.0;
    double y_mes = 0.0;
    double z_mes = 0.0;

    // Save the estimated position in the position k:
    // Position:
    double x_next = 0.0;
    double y_next = 0.0;
    double z_next = 0.0;
    // Velocity:
    double vx_next = 0.0;
    double vy_next = 0.0;
    double vz_next = 0.0;

    // Define the variables of the Jacobain matrices of the prpagbated states:
    Mat66 Ak;
    Eigen::Matrix<double,3,6> Ck;

    // Define the kalman gain matrix:
    Eigen::Matrix<double,6,3> Kk;

    // Define the meassurement and state matrices:
    Mat66 Q;
    Eigen::Matrix3d R;

    // Sart the estmiation:
    bool start_est = false;
 
    // Past time that the estimation or meassurement where done:
    double past_time = 0.0;
    double actual_time = 0.0;

    
    // Value thst defines when the adsb detection starts:
    bool start_adsb = false;
    // Boolean to obtian say when the initial conditions are obtianed:
    bool initial_conditions_obt = false;
    // Boolean to represnet when there is a meassurement;
    bool update_gnss = false;
    

    // Subscriptions:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<gnss_multipath_plugin::msg::GNSSMultipathFix>::SharedPtr gnss_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_adsb_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_adsb_srv_;
    rclcpp::Subscription<airplane_gazebo_plugin::msg::AirplaneKineticModelInfo>::SharedPtr states_sub_;
    rclcpp::Publisher<airplane_gazebo_plugin::msg::AdsbData>::SharedPtr adsb_pub_;



    // FUnctions used in the private functions:
    // Quaternion -> RPY (ZYX) helper:
    static inline void quat2rpy(const geometry_msgs::msg::Quaternion &q_msg,
                                double &roll, double &pitch, double &yaw)
    {
        Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        q.normalize();

        // Define the quaternion individually:
        const double w = q.w();
        const double x = q.x();
        const double y = q.y();
        const double z = q.z();


        roll = std::atan2((2*(y*z+w*x)),(1-2*(y*y+x*x)));
        pitch = std::asin(2*(y*w-z*x));
        yaw = std::atan2((2*(y*x+w*z)),(1-2*(z*z+y*y)));
    }


    // IMU callback:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

        // Orientation estimation using quaternions:
        const auto &q = msg->orientation;
        // Transform to roll, pitch and yaw:
        quat2rpy(q, roll_, pitch_, yaw_);
        course = M_PI/2 - yaw_;

        //Get the time at whichc the prpagatation is going to be done:
        rclcpp::Time t1(msg->header.stamp);
        actual_time = t1.seconds();

        // Get the accelerations:
        const auto &a = msg->linear_acceleration;
        const double ax = a.x*std::cos(pitch_)*std::cos(yaw_) + a.y*(std::sin(roll_)*std::sin(pitch_)*std::cos(yaw_)-std::cos(roll_)*std::sin(yaw_)) + a.z*(std::cos(roll_)*std::sin(pitch_)*std::cos(yaw_)+std::sin(roll_)*std::sin(yaw_));
        const double ay = a.x*std::cos(pitch_)*std::sin(yaw_) + a.y*(std::sin(roll_)*std::sin(pitch_)*std::sin(yaw_)+std::cos(roll_)*std::cos(yaw_)) + a.z*(std::cos(roll_)*std::sin(pitch_)*std::sin(yaw_)-std::sin(roll_)*std::cos(yaw_));
        const double az = -a.x*std::sin(pitch_) + a.y*std::sin(roll_)*std::cos(pitch_)  + a.z*std::cos(roll_)*std::cos(pitch_);

        // COntiune only if the initiacl condiitons are obtianed
        if (!initial_conditions_obt || update_gnss){
            return;
        }

        // Porpagate the system using hte npnlinear model:
        const double dt = (actual_time-past_time);
        x_next = vx0*dt+x0;
        y_next = vy0*dt+y0;
        z_next = vz0*dt+z0;
        vx_next = ax*dt+vx0;
        vy_next = ay*dt+vy0;
        vz_next = az*dt+vz0;

        // Prpagate teh covariance state matrix P:
        // Obtain the Jacobian amtrice:
        Ak << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        Ck << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;
        // Propagate P0:
        P0 = Ak*P0*Ak.transpose() + Q*dt;

        // Publish the results, this is going to be done alter:
        publish_adsb(x_next, y_next, z_next, vx_next, vy_next, vz_next, course, rclcpp::Time(msg->header.stamp), model_name_);

        // Reset the initial conditions to make the system forloop:
        x0 = x_next;
        y0 = y_next;
        z0 = z_next;
        vx0 = vx_next;
        vy0 = vy_next;
        vz0 = vz_next;
        past_time = actual_time;
    }


    // GNSS callback (intentionally empty while debugging):
    void gnss_callback(const gnss_multipath_plugin::msg::GNSSMultipathFix::SharedPtr msg)
    {
        // See if the Gnss have a update:
        update_gnss = true;
        // get the time at whcich the meassurement is obtained:
        rclcpp::Time t1(msg->header.stamp);
        actual_time = t1.seconds();

        // Update the meassurements:
        x_mes = msg->enu_gnss_fix[0];
        y_mes = msg->enu_gnss_fix[1];
        z_mes = msg->enu_gnss_fix[2];
        // Make a vector: 
        Eigen::Vector3d act_meas(x_mes, y_mes, z_mes);

        // Obtian the starting position before hte ads-b starts to work:
        if (!start_est){
            x0 = x_mes;
            y0 = y_mes;
            z0 = z_mes;
            return;
        }

        if (!start_adsb){
            return;
        }

        // Define when the ads-b should start to publish:
        if (!initial_conditions_obt){
            if (v0 > 1.0 && start_est)
            {
                // Define the intial velcoities:
                vx0 = v0 * std::cos(pitch_) * std::cos(yaw_);
                vy0 = v0 * std::cos(pitch_) * std::sin(yaw_);
                vz0 = v0 * std::sin(pitch_);

                // Define the past time as the initial time:
                rclcpp::Time t(msg->header.stamp);
                past_time = t.seconds();

                // Define the initial state covariance matrix with ampt that good psoition estimation but a good velocity estimation:
                P0 << 100, 0, 0, 0, 0, 0,
                    0, 100, 0, 0, 0, 0,
                    0, 0, 100, 0, 0, 0,
                    0, 0, 0, 15, 0, 0,
                    0, 0, 0, 0, 15, 0,
                    0, 0, 0, 0, 0, 15;

                // Tellt that the initial condiitons are obtained:
                initial_conditions_obt = true;
            } 
            // Return to do the estimation process:
            return;
        }

        // Obtain the Jacobians:
        const double dt = (actual_time-past_time);
        Ak << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        Ck << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;

        // Pass the estimated positions to a vector:
        Eigen::Matrix<double, 6, 1> x0_vect;
        x0_vect << x0, y0, z0, vx0, vy0, vz0;
        // Make the same for the update;
        Eigen::Matrix<double, 6, 1> xnext_vec;
        // Use the EKF witht he meassurement to update the estimations:
        Eigen::Vector3d z = act_meas;              // 3x1
        Eigen::Vector3d innov = z - Ck * x0_vect;  // 3x1
        Eigen::Matrix3d S = Ck * P0 * Ck.transpose() + R;
        Kk = P0 * Ck.transpose() * S.inverse();    // 6x3
        xnext_vec = x0_vect + Kk * innov; 
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
        P0 = (I-Kk*Ck)*P0;

        // Update the estiamtions:
        x0 = xnext_vec(0);
        y0 = xnext_vec(1);
        z0 = xnext_vec(2);
        vx0 = xnext_vec(3);
        vy0 = xnext_vec(4);
        vz0 = xnext_vec(5);
        past_time = actual_time;

        // Publish to the adsb:
        publish_adsb(x0, y0, z0, vx0, vy0, vz0, course, rclcpp::Time(msg->header.stamp), model_name_);

        // After the GNSS gives the meassurement make it be negative:
        update_gnss = false;
    }

    // Use the states of te airplane only for getting the initial velocity:
    void states_callback(const airplane_gazebo_plugin::msg::AirplaneKineticModelInfo::SharedPtr msg)
    {
        if (initial_conditions_obt){
            return;
        }

        // Get the velocity:
        v0 = 0.5 * msg->velocity_a;

        if (msg->velocity_a > 1.0){
            start_est = true;
        }
    }


    // Trigger callback to make the adsb starts to work:
    void trigger_callback_start(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        start_adsb = true;
        response->success = true;
        response->message = "ADS-B Strated.";
    }

    // Trigger callback to make the adsb stop to work:
    void trigger_callback_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        start_adsb = false;
        initial_conditions_obt = false;
        start_est = false;
        response->success = true;
        response->message = "ADS-B Stoped.";
    }

    // Function to publish the adsb data:
    void publish_adsb(double east, double north, double up, double v_east, double v_north, double v_up, double course_, const rclcpp::Time& stamp, const std::string& model_id)
    {
        airplane_gazebo_plugin::msg::AdsbData msg;

        // Header:
        msg.header.stamp = stamp;
        msg.header.frame_id = model_id; 

        // Position (your states are ENU; the message expects N/E/U):
        msg.north = north;
        msg.east  = east;
        msg.up    = up;

        // Velocity (same ENU→N/E/U mapping):
        msg.v_north = v_north;
        msg.v_east  = v_east;
        msg.v_up    = v_up;

        // Course: ground track from North (use velocity, not yaw)
        msg.course = course_;

        adsb_pub_->publish(msg);
    }
};

// Main logic of the executable:
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                        "Usage: ros2 run <package> <exec> <model_name> <imu_name>");
        return 1;
    }

    // Arguments: airplane name and the imu_name
    const std::string model_name = argv[1];
    const std::string imu_name   = argv[2];

    // Make the ADSBSensor shared:
    auto node = std::make_shared<ADSBSensor>(model_name, imu_name);

    // Spin continuously (so callbacks keep running)
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
