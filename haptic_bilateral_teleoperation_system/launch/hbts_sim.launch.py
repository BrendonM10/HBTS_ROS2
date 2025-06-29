from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    franka_agx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("panda_agx_bringup"), "launch", "agx_scene_panda_joint_traj_vel.launch.py"])
        ]),
        launch_arguments={"gui": "true"}.items()
    )


    GT_node = Node(
        package="omni_common",
        executable="omni_state",
        name="omni_state_node",
        output="screen",
        parameters=[
            {"omni_name": "phantom"},
            {"publish_rate": 1000},
            {"reference_frame": "panda_link0"}, 
            {"units": "mm"}
        ]
    )
    

    HBTS_node = Node(
        package="haptic_bilateral_teleoperation_system",
        executable="hbts_bridge_sim",
        name="hbts_bridge_sim",
        output="screen",
    )
   
    sensor_filtering = Node(
        package='haptic_bilateral_teleoperation_system', 
        executable='sensor_calibration_sim',
        name='sensor_calibration_sim',
        output="screen",
    )
    
    wrench_relay = Node(
        package='topic_tools',
        executable='relay',
        name='wrench_relay',
        arguments=['/calibrated_sensor/wrench', '/cartesian_compliance_controller/ft_sensor_wrench'],
        output='screen',
    )
    
   
    return LaunchDescription([
        franka_agx_launch,
        wrench_relay,
        sensor_filtering,
        GT_node,
        HBTS_node,
        
    ])
