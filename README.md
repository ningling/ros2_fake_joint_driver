This project is fake_joint_driver for ROS2. MoveIt2 now is in Beta. The demo code need fake_joint_driver. But the code I found is for ROS1. That is why I developed this driver to work with MoveIt2 Beta demo code. 
The project is depending on ros2_control. Please get ros2_control first. Please check out https://github.com/ros-controls/ros2_control.
To avoid the same project name with ROS1 fake_joint_driver, this project changed the package name to fake_joint_ros2. If you want to use it on moveit2 demo, please change the 'Fake joint driver' Node name in the launch file 'run_moveit_cpp.launch.py' from 'fake_joint_driver_node' to 'fake_joint_ros2' like the following:
Origin
```
    # Fake joint driver
    fake_joint_driver_node = Node(package='fake_joint_driver',
```
Change to:
```
    # Fake joint driver
    fake_joint_driver_node = Node(package='fake_joint_ros2',
```

If your Moveit2 has been already built, the launch file is locate on: $(Moveit2 root)/install/run_moveit_cpp/share/run_moveit_cpp/launch/
Please note if you build Moveit2 again, you will need to change the file again.


To use this project, please note the following parameters:
1. If you want to use a yaml file to initial robot's start position, please note that the yaml file  should be something like the following:
```yaml
fake_joint_driver_node: //Node name of this driver. It should be the same as the drv_node.
    ros__parameters:  //DO NOT change 
        start_position:    //NOT changable.  It is the name of parameter which the driver node will need to access. 
        joints: //NOT changable.  It is the name of parameter which the driver node will need to access. 
            - panda_joint1
            - panda_joint2
            - panda_joint3
            - panda_joint4
            - panda_joint5
            - panda_joint6
            - panda_joint7
        values: //NOT changable.  It is the name of parameter which the driver node will need to access. 
            - 0.0
            - -0.785
            - 0.0
            - -2.356
            - 0.0
            - 1.571
            - 0.785
```
2. If no startup position yaml file set, or the parameters name are NOT the valid ones, all start position of joints specified in urdf file will be set to 0.0. 
3. The trajectory controller node name is "fake_joint_trajectory_controller" which subscribing message name "/fake_joint_trajectory_controller/joint_trajectory"

    
