This project is fake_joint_driver for ROS2. MoveIt2 now is in Beta. The demo code need fake_joint_driver. But the code I found is for ROS1. That is why I developed this driver to work with MoveIt2 Beta demo code. 
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

    
