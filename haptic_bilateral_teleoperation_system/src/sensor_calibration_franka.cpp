#include "haptic_bilateral_teleoperation_system/sensor_calibration_franka.hpp"


FrankaSensorFiltering::FrankaSensorFiltering() : Node("franka_sensor_filtering_node") 
{

    // Subscriber
    sensor_franka_subs_ = this->create_subscription<franka_msgs::msg::FrankaState>(
        "/franka_robot_state_broadcaster/robot_state", 10,
        std::bind(&FrankaSensorFiltering::FilteredWrenchCallback, this, std::placeholders::_1));
    
    //Publisher
    filtred_sensor_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("franka_calibrated_sensor/wrench", 10);   
}


void FrankaSensorFiltering::FilteredWrenchCallback(const franka_msgs::msg::FrankaState::SharedPtr msg) {

    auto franka_panda_wrench = geometry_msgs::msg::WrenchStamped();
    franka_panda_wrench.header = msg->header;
    franka_panda_wrench.header.frame_id = "panda_hand";  // End-effector frame.

    // Extract O_F_ext_hat_K (6D wrench: force + torque in end-effector frame)
    franka_panda_wrench.wrench.force.x = msg->o_f_ext_hat_k[0];
    franka_panda_wrench.wrench.force.y = msg->o_f_ext_hat_k[1];
    franka_panda_wrench.wrench.force.z = msg->o_f_ext_hat_k[2];
    franka_panda_wrench.wrench.torque.x = msg->o_f_ext_hat_k[3];
    franka_panda_wrench.wrench.torque.y = msg->o_f_ext_hat_k[4];
    franka_panda_wrench.wrench.torque.z = msg->o_f_ext_hat_k[5];

    
    // Apply IIR filter to force and torque data:
    
    // Change this value depending on how you want to control the smoothing strength.
    double alpha = 0.001; 
    
    double apply_filter_force_x = alpha * franka_panda_wrench.wrench.force.x + (1.0 - alpha) * apply_filter_force_x;
    double apply_filter_force_y = alpha * franka_panda_wrench.wrench.force.y + (1.0 - alpha) * apply_filter_force_y;
    double apply_filter_force_z = alpha * franka_panda_wrench.wrench.force.z + (1.0 - alpha) * apply_filter_force_z;

    // The torque data is not used in the project.
    // However, if the torque data is desired in the future, it will already be processed and ready to be used.
    double apply_filter_torque_x = alpha * franka_panda_wrench.wrench.torque.x + (1.0 - alpha) * apply_filter_torque_x;
    double apply_filter_torque_y = alpha * franka_panda_wrench.wrench.torque.x + (1.0 - alpha) * apply_filter_torque_y;
    double apply_filter_torque_z = alpha * franka_panda_wrench.wrench.torque.x + (1.0 - alpha) * apply_filter_torque_z;

    // Publish filtered data.
    geometry_msgs::msg::WrenchStamped processed_sensor_msg;

    processed_sensor_msg.header = msg->header;
    processed_sensor_msg.wrench.force.x = apply_filter_force_x;
    processed_sensor_msg.wrench.force.y = apply_filter_force_y;
    processed_sensor_msg.wrench.force.z = apply_filter_force_z;
    processed_sensor_msg.wrench.torque.x = apply_filter_torque_x;
    processed_sensor_msg.wrench.torque.y = apply_filter_torque_y;
    processed_sensor_msg.wrench.torque.z = apply_filter_torque_z;

    filtred_sensor_pub_->publish(processed_sensor_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrankaSensorFiltering>());
    rclcpp::shutdown();
    return 0;
}
