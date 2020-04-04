#include <rclcpp/rclcpp.hpp>
#include  "controller_manager/controller_manager.hpp"
#include <iostream>
#include <exception>
#include <rclcpp_lifecycle/state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "fake_joint_ros2/fake_joint_driver.hpp"

void spin (std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
    RCLCPP_INFO(rclcpp::get_logger("Func Spin"), "Start spinning...");
    exe->spin();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const rclcpp::Logger logger=rclcpp::get_logger("fake_driver_main");
    auto my_arm = std::make_shared<fake_arm>();
     

    if (my_arm->init()!=hardware_interface::HW_RET_OK)
    {
        RCLCPP_ERROR(logger, "failed to initialize hardware!\n");
        return -1;
    }


    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    controller_manager::ControllerManager ctlmgr(my_arm, executor,  "fake_joint_controller_mgr");

    controller_interface::ControllerInterfaceSharedPtr state_controller=ctlmgr.load_controller(
        "ros2_state_controller",
        "joint_state_controller/JointStateController"
    );

    
    controller_interface::ControllerInterfaceSharedPtr traj_controller = ctlmgr.load_controller(
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
    RCLCPP_INFO(logger, "Controller Manager config done!");

    if (ctlmgr.activate() != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS)
    {
        RCLCPP_ERROR(logger, "at least one controller NOT activated!\n");
        executor->cancel();
        return -1;
    }
    RCLCPP_INFO(logger, "Controller Manager Activated.");

    hardware_interface::hardware_interface_ret_t ret;

    double rate_val=1000.0; //Updating in 100Hz.
    rclcpp::Rate rate(rate_val);
    RCLCPP_INFO(logger, "Updating in %.3f Hz", rate_val);

    uint8_t current_state=traj_controller->get_lifecycle_node()->get_current_state().id();

    while (rclcpp::ok()) 
    {
        if (current_state!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_WARN(logger, "Trajectory state is NOT ACTIVE, it is:");
            switch (current_state)
            {
                case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
                {
                    RCLCPP_INFO(logger, "ACTIVE");
                    break;
                }

                case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
                {
                    RCLCPP_INFO(logger, "FINALIZED");
                    break;
                }
                case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
                {
                    RCLCPP_INFO(logger, "UNCONFIGURED");
                    break;
                }
                case lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN:
                {
                    RCLCPP_INFO(logger, "UNKNOW");
                    break;
                }

                default:
                {
                    RCLCPP_WARN(logger, "NOT valid status");
                }

            }
        }

        ret=my_arm->read();
        if (ret != hardware_interface::HW_RET_OK){
            RCLCPP_ERROR(logger, "read operation failed!");
        }
        ctlmgr.update();
        ret=my_arm->write();
        if (ret!=hardware_interface::HW_RET_OK){
            RCLCPP_ERROR(logger, "write operation failed!");
        }
        rate.sleep();
    }

    executor->cancel();
    


    //state_controller->get_lifecycle_node()->cleanup();
    //ctlmgr.cleanup();
    
    rclcpp::shutdown();
    return 0;
}