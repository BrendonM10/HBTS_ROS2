#include "haptic_bilateral_teleoperation_system/sensor_calibration_sim.hpp"


SensorFiltering::SensorFiltering() : Node("sensor_filtering")
{
 
    // Subscriber 
    sensor_subs_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/fts_broadcaster/wrench", 10,
        std::bind(&SensorFiltering::FilteredWrenchCallback, this, std::placeholders::_1));
    
    //Publisher
    filtred_sensor_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/calibrated_sensor/wrench", 10);

}

void SensorFiltering::FilteredWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{

    // Apply IIR filter to force and torque data:

    // Change this value depending on how you want to control the smoothing strength.
    double alpha = 0.05; 

    // Necessary since the end-effector in simulation weights around 7.48 N. 
    double offset_z_axis = -7.448; 

    double apply_filter_force_x = alpha * msg->wrench.force.x + (1.0 - alpha) * apply_filter_force_x;
    double apply_filter_force_y = alpha * msg->wrench.force.y + (1.0 - alpha) * apply_filter_force_y;
    double apply_filter_force_z = alpha * (msg->wrench.force.z + offset_z_axis) + (1.0 - alpha) * apply_filter_force_z;

    // However, if the torque data is desired in the future, it will already be processed and ready to be used.
    double apply_filter_torque_x = alpha * msg->wrench.torque.x + (1.0 - alpha) * apply_filter_torque_x;
    double apply_filter_torque_y = alpha * msg->wrench.torque.y + (1.0 - alpha) * apply_filter_torque_y;
    double apply_filter_torque_z = alpha * msg->wrench.torque.z + (1.0 - alpha) * apply_filter_torque_z;


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


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFiltering>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
