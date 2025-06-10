#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <omni_msgs/msg/omni_button_event.hpp>
#include <omni_msgs/msg/omni_feedback.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

class HBTS : public rclcpp::Node 
{
public:

    HBTS();
   
private: 
    void GeomagicTouchCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void FEPCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool AllowedMovement(const geometry_msgs::msg::Point& current_position, const geometry_msgs::msg::Point& last_valid_position);
    void WorkspaceLimits(geometry_msgs::msg::PoseStamped& valid_workspace_pose);
    void GeomagicTouchButtonCallback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg);
    void ForceFeedbackCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    void NeglectForceFeedback();
    void AllowedForce(omni_msgs::msg::OmniFeedback& force_feedback);
    
    
    
    // Member variables: 
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr GT_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr FEP_pose_sub_;
    
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr GT_button_sub_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_feedback_sub_;
    
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr processed_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr processed_pose_pub_2;
    rclcpp::Publisher<omni_msgs::msg::OmniFeedback>::SharedPtr processed_force_feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr FEP_gripper_commands_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visualisation_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visualisation_pose_pub_2;
    

    bool press_white_button_ = false;
    bool press_grey_button_ = false;
    bool FEP_open_gripper = false;
    
   
    
    geometry_msgs::msg::PoseStamped::SharedPtr last_FEP_pose_;
    geometry_msgs::msg::PoseStamped last_wished_pose_;

};
    
  
