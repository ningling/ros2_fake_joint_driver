#include "hardware_interface/robot_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

class fake_arm : public hardware_interface::RobotHardware
{
    private:
        bool chg_flg=true; //to monitor whether the position have changed. This flag is to save the unneccessary read access to hardware. 
        std::vector<hardware_interface::JointCommandHandle> joint_cmd_handle_;
        std::vector<hardware_interface::JointStateHandle> joint_state_handle_;
        std::vector<hardware_interface::OperationModeHandle> joint_op_mode_handle_;

        std::vector<double> cmd_;
        std::vector<double> pos_;
        std::vector<double> vel_;
        std::vector<double> eff_;

        std::vector<hardware_interface::OperationMode> op_mode_;

        const rclcpp::Logger  logger=rclcpp::get_logger("fake_arm_logger");
        int trajectory_point_counter=0;
        //rclcpp::Logger logger;      

    public:
        fake_arm(void){};
        hardware_interface::hardware_interface_ret_t init();
        hardware_interface::hardware_interface_ret_t read();
        hardware_interface::hardware_interface_ret_t write();
        std::vector<std::string> joint_names_;
        ~fake_arm(void){};
};
