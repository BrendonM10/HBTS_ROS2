#pragma once


#include "rclcpp/rclcpp.hpp"
#include "franka_msgs/msg/franka_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"


class FrankaSensorFiltering : public rclcpp::Node 
{
public: 

    FrankaSensorFiltering();
    
private:

    void FilteredWrenchCallback(const franka_msgs::msg::FrankaState::SharedPtr msg);
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtred_sensor_pub_;
    rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr sensor_franka_subs_;
    
};
