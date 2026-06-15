import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RL_GYM = os.path.expanduser(os.environ.get('UNITREE_RL_GYM_PATH', '~/unitree_rl_gym'))


def generate_launch_description():
    args = [
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('xml_path',
            default_value=os.path.join(RL_GYM, 'resources/robots/g1_description/scene.xml')),
        DeclareLaunchArgument('policy_path',
            default_value=os.path.join(RL_GYM, 'deploy/pre_train/g1/motion.pt')),
        DeclareLaunchArgument('config_path',
            default_value=os.path.join(RL_GYM, 'deploy/deploy_mujoco/configs/g1.yaml')),
        DeclareLaunchArgument('bag_dir',
            default_value=os.path.expanduser('~/progetto_robotica_bags')),
        DeclareLaunchArgument('scenario', default_value='flat',
                              description='Scenario MuJoCo: flat oppure obstacle_course'),
        DeclareLaunchArgument('telemetry_hz', default_value='50.0',
                              description='Frequenza topic ROS pubblicati dal simulatore'),
        DeclareLaunchArgument('csv_hz', default_value='50.0',
                              description='Frequenza logging CSV durante la registrazione'),
        DeclareLaunchArgument('viewer_fps', default_value='15.0',
                              description='Frequenza massima sync viewer MuJoCo'),
        DeclareLaunchArgument('record_profile', default_value='metrics',
                              description='Profilo rosbag: minimal, metrics oppure full'),
    ]

    sim_node = Node(
        package='progetto_robotica', executable='mujoco_sim',
        name='mujoco_sim_node', output='screen',
        parameters=[{
            'headless': LaunchConfiguration('headless'),
            'xml_path': LaunchConfiguration('xml_path'),
            'policy_path': LaunchConfiguration('policy_path'),
            'config_path': LaunchConfiguration('config_path'),
            'bag_dir': LaunchConfiguration('bag_dir'),
            'scenario': LaunchConfiguration('scenario'),
            'telemetry_hz': LaunchConfiguration('telemetry_hz'),
            'csv_hz': LaunchConfiguration('csv_hz'),
            'viewer_fps': LaunchConfiguration('viewer_fps'),
        }],
    )

    web_node = Node(
        package='progetto_robotica', executable='web_teleop',
        name='web_teleop_node', output='screen',
        parameters=[{
            'bag_dir': LaunchConfiguration('bag_dir'),
            'scenario': LaunchConfiguration('scenario'),
            'record_profile': LaunchConfiguration('record_profile'),
        }],
    )

    return LaunchDescription(args + [sim_node, web_node])
