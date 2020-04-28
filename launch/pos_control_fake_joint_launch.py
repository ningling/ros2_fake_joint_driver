import os
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def load_local_yaml(file_path):
    current_path = os.path.abspath('.')
    absolute_file_path = os.path.join(current_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_local_file(file_path):
    file_abspath = os.path.abspath('.')
    absolute_file_path = os.path.join(file_abspath, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    robot_description_config = load_file('fake_joint_ros2','config/xARM.urdf')
    robot_description = {'robot_description' : robot_description_config}
    #print (robot_description)

    rviz_config_file = 'config/urdf-ros1.rviz'
    rviz_node = Node(package='rviz2',
                     node_executable='rviz2',
                     node_name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description])
    
    
    robot_state_node = Node(package='robot_state_publisher',
                     node_executable='robot_state_publisher',
                     node_name='robot_state', 
                     output='both',
                     arguments=['config/xARM.urdf'])
                     #parameters=[robot_description])


                     
    fake_joint_ros2_driver_node=Node(package='fake_joint_ros2',
                                                        node_executable='fake_joint_driver_node',
                                                        output='both',
                                                        emulate_tty=True,
                                                        parameters=['config/xARM_controllers.yaml', 
                                                        'config/start_positions_all_zero.yaml',
                                                        robot_description]
                                                        )

    """
    pos_control_node=Node(package='joints_pos_control',
                                                        node_executable='joints_pos_control',
                                                        emulate_tty=True
                                                        )
    return LaunchDescription([rviz_node, robot_state_node, fake_joint_ros2_driver_node, pos_control_node])
    """
    return LaunchDescription([rviz_node, robot_state_node, fake_joint_ros2_driver_node])
    