#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <deque>  // For smoothing RTF

namespace gazebo
{
class RealTimeFactorPublisher : public WorldPlugin
{
private:
  gazebo_ros::Node::SharedPtr ros_node;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rtf_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_time_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr real_time_publisher;
  event::ConnectionPtr update_connection;
  physics::WorldPtr world;
  
  // For RTF smoothing
  std::deque<float> rtf_history;
  const size_t rtf_window_size = 30;  // Adjust for smoother/more responsive RTF

public:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
  {
    world = _world;
    ros_node = gazebo_ros::Node::Get(_sdf);
    rtf_publisher = ros_node->create_publisher<std_msgs::msg::Float32>("/real_time_factor", 10);
    sim_time_publisher = ros_node->create_publisher<std_msgs::msg::Float32>("/sim_time", 10);
    real_time_publisher = ros_node->create_publisher<std_msgs::msg::Float32>("/real_time", 10);

    update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RealTimeFactorPublisher::OnUpdate, this));
  }

  void OnUpdate()
  {
    float sim_time = world->SimTime().Float();
    float real_time = world->RealTime().Float();

    // Compute instantaneous RTF (sim_time_delta / real_time_delta)
    static float last_sim_time = 0.0f;
    static float last_real_time = 0.0f;
    float rtf_instant = (sim_time - last_sim_time) / (real_time - last_real_time);
    last_sim_time = sim_time;
    last_real_time = real_time;

    // Smooth RTF using a moving average
    rtf_history.push_back(rtf_instant);
    if (rtf_history.size() > rtf_window_size)
      rtf_history.pop_front();

    float rtf_smoothed = 0.0f;
    for (float val : rtf_history)
      rtf_smoothed += val;
    rtf_smoothed /= rtf_history.size();

    // Publish
    std_msgs::msg::Float32 msg_rtf, msg_sim, msg_real;
    msg_rtf.data = rtf_smoothed;  // Use smoothed RTF (matches Gazebo GUI better)
    msg_sim.data = sim_time;
    msg_real.data = real_time;

    rtf_publisher->publish(msg_rtf);
    sim_time_publisher->publish(msg_sim);
    real_time_publisher->publish(msg_real);
  }
};

GZ_REGISTER_WORLD_PLUGIN(RealTimeFactorPublisher)
}  // namespace gazebo