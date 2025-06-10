#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <array>
#include <algorithm> 


class SensorFiltering : public rclcpp::Node
{
public:

    SensorFiltering();
       
private:

    void FilteredWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    
    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sensor_subs_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtred_sensor_pub_;
};
