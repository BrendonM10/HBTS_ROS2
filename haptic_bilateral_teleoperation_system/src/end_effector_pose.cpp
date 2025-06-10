#include "rclcpp/rclcpp.hpp"
#include "franka_msgs/msg/franka_state.hpp"
#include "geometry_msgs/msg/point.hpp"

// Was only used to conduct the shape sorter experiment.

class EndEffectorPublisher : public rclcpp::Node
{
public:
  EndEffectorPublisher()
  : Node("end_effector_publisher")
  {
    // subscriber to robot state
    robot_state_subscriber_ = this->create_subscription<franka_msgs::msg::FrankaState>(
      "/franka_robot_state_broadcaster/robot_state",
      10,
      std::bind(&EndEffectorPublisher::robot_state_callback, this, std::placeholders::_1));
    
    //  publish end effector position
    end_effector_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/franka_endeffector/pose",
      10);
    
    RCLCPP_INFO(this->get_logger(), "End effector position publisher initialized");
  }

private:
  void robot_state_callback(const franka_msgs::msg::FrankaState::SharedPtr msg)
  {
    
    if (msg->o_t_ee.size() >= 16) {
      geometry_msgs::msg::Point position;
      position.x = msg->o_t_ee[12];
      position.y = msg->o_t_ee[13];
      position.z = msg->o_t_ee[14];
      
      // Publish the position
      end_effector_publisher_->publish(position);
      
      
    } 
  }
  
  rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr robot_state_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr end_effector_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EndEffectorPublisher>());
  rclcpp::shutdown();
  return 0;
}
