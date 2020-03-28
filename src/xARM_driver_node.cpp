#include <rclcpp/rclcpp.hpp>
#include  "controller_manager/controller_manager.hpp"
#include <iostream>
#include "fake_joint_ros2/xARM_driver.hpp"
void spin (std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
    exe->spin();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const rclcpp::Logger logger=rclcpp::get_logger("fake_driver_main");
    auto my_xARM = std::make_shared<xARM>();
     

    if (my_xARM->init()!=hardware_interface::HW_RET_OK)
    {
        RCLCPP_ERROR(logger, "failed to initialize hardware!\n");
        return -1;
    }


    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

   controller_manager::ControllerManager ctlmgr(my_xARM, executor,  "xARM_controller_mgr");

    ctlmgr.load_controller(
        "ros2_state_controller",
        "joint_state_controller/JointStateController"
    );

    
    ctlmgr.load_controller(
        "fake_joint_trajectory_controller",
        "joint_trajectory_controller/JointTrajectoryController"
    );
    

    auto future_handle = std::async(std::launch::async, spin, executor);
   
    if (ctlmgr.configure() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    {
        RCLCPP_ERROR(logger, "at least one controller failed to configure");
        executor->cancel();
        return -1;
    }
    
    if (ctlmgr.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    {
        RCLCPP_ERROR(logger, "at least one controller NOT activated!\n");
        executor->cancel();
        return -1;
    }
        
    hardware_interface::hardware_interface_ret_t ret;
    //double rate_val=1.0/rclcpp::Duration(0.01).seconds();
    double rate_val=100.0; //Updating in 100Hz.
    rclcpp::Rate rate(rate_val);
    RCLCPP_INFO(logger, "Updating in %.3f Hz", rate_val);
    while (rclcpp::ok()) 
    {
        ret=my_xARM->read();
        if (ret != hardware_interface::HW_RET_OK){
            RCLCPP_ERROR(logger, "read operation failed!");
        }

        ctlmgr.update();

        ret=my_xARM->write();
        if (ret!=hardware_interface::HW_RET_OK){
            RCLCPP_ERROR(logger, "write operation failed!");
        }
        rate.sleep();
    }
    
    executor->cancel();
    return 0;
}