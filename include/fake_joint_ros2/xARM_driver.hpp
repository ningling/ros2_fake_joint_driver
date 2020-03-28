#include "hardware_interface/robot_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

class xARM : public hardware_interface::RobotHardware
{
    private:
        std::vector<hardware_interface::JointCommandHandle> joint_cmd_handle_;
        std::vector<hardware_interface::JointStateHandle> joint_state_handle_;
        std::vector<hardware_interface::OperationModeHandle> joint_op_mode_handle_;

        std::vector<double> cmd_;
        std::vector<double> pos_;
        std::vector<double> vel_;
        std::vector<double> eff_;

        std::vector<hardware_interface::OperationMode> op_mode_;

        const rclcpp::Logger  logger=rclcpp::get_logger("xARM_logger");

        

    public:
        xARM(void){};
        hardware_interface::hardware_interface_ret_t init();
        hardware_interface::hardware_interface_ret_t read();
        hardware_interface::hardware_interface_ret_t write();
        std::vector<std::string> joint_names_;
        ~xARM(void){};
};
