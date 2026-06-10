import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run MuJoCo simulation headlessly (no OpenGL viewer window)'
    )
    
    xml_path_arg = DeclareLaunchArgument(
        'xml_path',
        default_value='/home/ergys/unitree_rl_gym/resources/robots/g1_description/scene.xml',
        description='Absolute path to the G1 MuJoCo scene XML file'
    )
    
    policy_path_arg = DeclareLaunchArgument(
        'policy_path',
        default_value='/home/ergys/unitree_rl_gym/deploy/pre_train/g1/motion.pt',
        description='Absolute path to the PyTorch JIT policy file'
    )
    
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='/home/ergys/unitree_rl_gym/deploy/deploy_mujoco/configs/g1.yaml',
        description='Absolute path to the deploy configuration YAML file'
    )

    # Simulation Node
    sim_node = Node(
        package='progetto_robotica',
        executable='mujoco_sim',
        name='mujoco_sim_node',
        output='screen',
        parameters=[{
            'headless': LaunchConfiguration('headless'),
            'xml_path': LaunchConfiguration('xml_path'),
            'policy_path': LaunchConfiguration('policy_path'),
            'config_path': LaunchConfiguration('config_path'),
        }]
    )

    # Web teleop / dashboard backend Node
    web_node = Node(
        package='progetto_robotica',
        executable='web_teleop',
        name='web_teleop_node',
        output='screen'
    )

    return LaunchDescription([
        headless_arg,
        xml_path_arg,
        policy_path_arg,
        config_path_arg,
        sim_node,
        web_node
    ])
