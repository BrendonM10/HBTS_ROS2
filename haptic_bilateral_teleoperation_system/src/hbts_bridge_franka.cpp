#include "haptic_bilateral_teleoperation_system/hbts_bridge_franka.hpp"


HBTS::HBTS() : Node("HBTS")
{
    // Subscriptions: 

    // Subscribe to the current pose of the geomagic touch device.
    GT_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/phantom/pose", 10,
        std::bind(&HBTS::GeomagicTouchCallback, this, std::placeholders::_1));
    
    // Subscribe to the current pose of the FEP robot.
    FEP_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cartesian_compliance_controller/current_pose", 10,
        std::bind(&HBTS::FEPCallback, this, std::placeholders::_1));

    // Subscribe to the Geomagic Touch button state. 
    GT_button_sub_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(
        "/phantom/button", 10,
        std::bind(&HBTS::GeomagicTouchButtonCallback, this, std::placeholders::_1));
    
    // Subscribe to the F/T sensor data.
    force_feedback_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/cartesian_compliance_controller/ft_sensor_wrench", 10,
        std::bind(&HBTS::ForceFeedbackCallback, this, std::placeholders::_1));




    //Publishings:

    // Publish the processed pose to the Cartesian compliance controller.
    processed_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
       "/cartesian_compliance_controller/target_frame", 10);
    
    // Publish the processed force feedback to the Geomagic Touch.
    processed_force_feedback_pub_ = this->create_publisher<omni_msgs::msg::OmniFeedback>(
        "/phantom/force_feedback", 10);
  
    // Publish the processed pose as a visulisation. 
    phantom_visualisation_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/phantom/pose/visualisation", 10);
    



    // Clients: 

    // Send action command to open gripper. 
    FEP_gripper_move_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(this, "/panda_gripper/move");

    // Send action command to close the gripper.
    FEP_gripper_grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(this, "/panda_gripper/grasp");    
}


    
    
// Process pose data from the GT device. 
void HBTS::GeomagicTouchCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // The pose that will be processed.
    geometry_msgs::msg::PoseStamped wished_pose = *msg;
    wished_pose.header = msg->header;

    // Scaling according to the FEP's table workspace. 
    double postion_scale_x = 12.0; 
    double postion_scale_y = 5.0;
    double postion_scale_z = 10.0;

    // Modify this offset if you want to move the FEP robot further in the x-direction.
    double x_offset  = 0.55; 


    // Remapped x and y, and applied scaling. 
    wished_pose.pose.position.x = (msg->pose.position.y * postion_scale_x) + x_offset; 
    wished_pose.pose.position.y = -msg->pose.position.x * postion_scale_y;
    wished_pose.pose.position.z = msg->pose.position.z * postion_scale_z;

    wished_pose.pose.orientation.x = msg->pose.orientation.y; 
    wished_pose.pose.orientation.y = -msg->pose.orientation.x; 
    wished_pose.pose.orientation.z = msg->pose.orientation.z; 

    // Check if the wished movement is allowed.
    if (AllowedMovement(wished_pose.pose.position, last_wished_pose_.pose.position)) {
        // Ignore large movements.
        wished_pose = last_wished_pose_;
    } 
    else {
        // Update the last wished pose.
        last_wished_pose_ = wished_pose;
    }

    // Clamp the wished pose to the workspace limits.
    WorkspaceLimits(wished_pose);

    if (press_white_button_) {
        // Publish the processed pose.
        processed_pose_pub_->publish(wished_pose);
    }
    else {
        // Show the processed pose as visualisation before publishing.
        phantom_visualisation_pose_pub_->publish(wished_pose);
    }

}

// Save the last valid position. 
void HBTS::FEPCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Store the last known robot pose.
    last_FEP_pose_ = msg;
}

// Check if desired movement is allowed.
bool HBTS::AllowedMovement(const geometry_msgs::msg::Point& current_position, const geometry_msgs::msg::Point& last_valid_position)
{
    // Calculate the Euclidean distance between the current and last position.
    double distance_x = current_position.x - last_valid_position.x;
    double distance_y = current_position.y - last_valid_position.y;
    double distance_z = current_position.z - last_valid_position.z;
    double total_distance = std::sqrt(distance_x * distance_x + distance_y * distance_y + distance_z * distance_z);

    // Define a threshold for large movements.
    double valid_distance = 0.55;

    
    return total_distance > valid_distance;
}

// Specifify the allowed workspace the FEP robot can move within.
void HBTS::WorkspaceLimits(geometry_msgs::msg::PoseStamped& valid_workspace_pose)
{
    // Clamp the x, y, and z coordinates to the workspace limits
    valid_workspace_pose.pose.position.x = std::clamp(valid_workspace_pose.pose.position.x, -0.7, 0.7);
    valid_workspace_pose.pose.position.y = std::clamp(valid_workspace_pose.pose.position.y, -0.7, 0.7);
    valid_workspace_pose.pose.position.z = std::clamp(valid_workspace_pose.pose.position.z, 0.0, 1.2);

}
    
// GT device button logic.
void HBTS::GeomagicTouchButtonCallback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg)
{     
    // The gripper's open position (in meters). 
    double FEP_open_griper_width = 0.08;

    // The gripper's closed position (in meters).
    double FEP_close_griper_width = 0.00;   
 
    // Keeps track if the gripper is open or closed.
    if (msg->grey_button && !press_grey_button_) {
        FEP_open_gripper = !FEP_open_gripper; 
    
        // Opens or closes the gripper depending on its current state.  
        if (FEP_open_gripper) {
            SendFEPGripperGrasp(FEP_open_griper_width);
        } 
        else {
            SendFEPGripperMove(FEP_close_griper_width);
        }
    }

    press_grey_button_ = msg->grey_button;

    // Keeps track if the white buttion is pressed or not.
    bool previous_button_state = press_white_button_;
    press_white_button_ = msg->white_button;

    if (previous_button_state && !press_white_button_) {
        // If the white button is released, stop the FEP robot by publishing the last known pose.
        if (last_FEP_pose_) {
            processed_pose_pub_->publish(*last_FEP_pose_);
        }
        // If the white button is released, no force feedback should be published.
        NeglectForceFeedback();
    }
}
    
// Open the gripper.
void HBTS::SendFEPGripperMove(double FEP_gripper_width)
{
    if (!FEP_gripper_move_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
        return;
    }

    auto send_move_goal = franka_msgs::action::Move::Goal();
    send_move_goal.width = FEP_gripper_width;
    send_move_goal.speed = FEP_open_close_gripper_velocity;
    FEP_gripper_move_client_->async_send_goal(send_move_goal);
}
    
// Close the gripper. 
void HBTS::SendFEPGripperGrasp(double FEP_gripper_width)
{
    if (!FEP_gripper_grasp_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
        return;
    }

    auto send_grasp_goal = franka_msgs::action::Grasp::Goal();

    // Parameters to controll how the gripper should grasp the object.
    double FEP_gripper_grasp_force = 60.0; 
    double FEP_gripper_inner_epsilon = 0.01;
    double FEP_gripper_outer_epsilon = 0.01;  

    send_grasp_goal.width = FEP_gripper_width;
    send_grasp_goal.speed = FEP_open_close_gripper_velocity;
    send_grasp_goal.force = FEP_gripper_grasp_force;
    send_grasp_goal.epsilon.inner = FEP_gripper_inner_epsilon;
    send_grasp_goal.epsilon.outer = FEP_gripper_outer_epsilon;
    FEP_gripper_grasp_client_->async_send_goal(send_grasp_goal);
}
    
// Process the force data from the FEP robot's end-effector.
void HBTS::ForceFeedbackCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{	
    // User should only be able to feel force feedback if the white button is pressed.
    if (!press_white_button_) {
        NeglectForceFeedback();
        return;
    }

    // Change the value if you want to feel more or less force feedback.
    double force_scale = 0.5;  

    // Process raw force data.
    geometry_msgs::msg::Vector3 original_force;
    original_force.x = msg->wrench.force.x;
    original_force.y = msg->wrench.force.y;
    original_force.z = msg->wrench.force.z;

    omni_msgs::msg::OmniFeedback scaled_force_feedback;
    scaled_force_feedback.force.x = original_force.y * force_scale;
    scaled_force_feedback.force.y = -original_force.x * force_scale;
    scaled_force_feedback.force.z = -original_force.z * force_scale;

    // Make sure the the force does not exceed 2N.
    AllowedForce(scaled_force_feedback);

    // Publish the processed force feedback. 
    processed_force_feedback_pub_->publish(scaled_force_feedback);
}
    
// Necessary for safety reasons...
void HBTS::NeglectForceFeedback()
{
    omni_msgs::msg::OmniFeedback no_force_feedback;
    no_force_feedback.force.x = 0.0;
    no_force_feedback.force.y = 0.0;
    no_force_feedback.force.z = 0.0;
    processed_force_feedback_pub_->publish(no_force_feedback);
}
    
// Specify allowed force feedback.
void HBTS::AllowedForce(omni_msgs::msg::OmniFeedback& force_feedback)
{
    const double allowed_force_feedback = 2.0;  // The hardware limit is 3.3N, but to not damage the GT 2N is used for testing.

    force_feedback.force.x = std::clamp(force_feedback.force.x, -allowed_force_feedback, allowed_force_feedback);
    force_feedback.force.y = std::clamp(force_feedback.force.y, -allowed_force_feedback, allowed_force_feedback);
    force_feedback.force.z = std::clamp(force_feedback.force.z, -allowed_force_feedback, allowed_force_feedback);
}
    
  



int main(int argc, char** argv)
{
    
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HBTS>();
  
  // Wait for TF frames to arrive
  rclcpp::sleep_for(std::chrono::seconds(1));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
