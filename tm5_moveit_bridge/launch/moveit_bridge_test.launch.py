from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    moveit_config = get_package_share_directory(
        'tm5-700_moveit_config'
    )

    urdf_file = os.path.join(
        get_package_share_directory('tm_description'),
        'urdf',
        'tm5-700-nominal.urdf'
    )

    srdf_file = os.path.join(
        moveit_config,
        'config',
        'tm5-700.srdf'
    )
    
    kinematics_file = os.path.join(
    	moveit_config,
    	'config',
    	'kinematics.yaml'
    ) 

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()
    
    with open(kinematics_file, 'r') as f:
    	kinematics = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package='tm5_moveit_bridge',
            executable='moveit_bridge',
            name='tm5_moveit_bridge',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics,
                }
            ],
        )
    ])
