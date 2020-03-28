#include <stdexcept>
#include <urdf/model.h>
#include "fake_joint_ros2/xARM_driver.hpp"

hardware_interface::hardware_interface_ret_t
xARM::init()
{
    size_t i=0;
    size_t joints_count;
    size_t counter=0;

    std::vector<std::string> start_pos_joint_name;
    std::vector<double> start_pos_value;

    urdf::Model urdf_model;
    std::set<std::string> joint_set;
    std::map<std::string, double> start_position_map;
    std::map<std::string, double>::iterator start_pos_it;
    rclcpp::NodeOptions options;
    rclcpp::Node drv_node("fake_joint_driver_node", options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true));
    rclcpp::Parameter param_joints_name=drv_node.get_parameter("start_position.joints");
    rclcpp::Parameter param_joints_value=drv_node.get_parameter("start_position.values");

    urdf_model.initString(drv_node.get_parameter("robot_description").as_string());
    
    //From the input URDF file to get joints info
    for (auto it=urdf_model.joints_.begin(); it!=urdf_model.joints_.end();it++)
    {
        urdf::Joint joint=*it->second;
        if (joint.type==urdf::Joint::FIXED || joint.type==urdf::Joint::UNKNOWN)
        {
            continue;
        }
        
        joint_set.insert(joint.name);
    }

    std::copy(joint_set.begin(), joint_set.end(), std::back_inserter(joint_names_));
    joints_count=joint_names_.size();
    
    RCLCPP_INFO(logger, "Number of effective joints: [%d]\n", joints_count);
    cmd_.resize(joints_count,0.0);
    pos_.resize(joints_count,0.0);
    vel_.resize(joints_count, 0.0);
    eff_.resize(joints_count, 0.0);

    op_mode_.resize(joints_count, hardware_interface::OperationMode::ACTIVE);

    joint_op_mode_handle_.resize(joints_count);
    joint_state_handle_.resize(joints_count);
    joint_cmd_handle_.resize(joints_count);

    /*Initial start position value via the config from yaml file. The file format should be something like the following example:
    fake_joint_driver_node: //Node name of this driver. It should be the same as the drv_node.
    ros__parameters:  //DO NOT change 
        start_position:    
        joints:
            - panda_joint1
            - panda_joint2
            - panda_joint3
            - panda_joint4
            - panda_joint5
            - panda_joint6
            - panda_joint7
        values:
            - 0.0
            - -0.785
            - 0.0
            - -2.356
            - 0.0
            - 1.571
            - 0.785
    */
    if (param_joints_name.get_type()==rclcpp::ParameterType::PARAMETER_NOT_SET || param_joints_value.get_type()==rclcpp::ParameterType::PARAMETER_NOT_SET)
    {
        RCLCPP_WARN(logger, "Can NOT find valid start position yaml config file. The file should have joints and value entries. Setting all start position value of valid joints in urdf to 0.0");
    }
    else
    {
        start_pos_joint_name=param_joints_name.as_string_array();
        start_pos_value=param_joints_value.as_double_array();
        if (start_pos_value.size()!=start_pos_joint_name.size())
        {
            RCLCPP_WARN(logger, "yaml config file error: joint name and values are NOT the same size! Setting all start position value of valid joints in urdf to 0.0 instead!");
        }
        else
        {
            counter=0;
            
            for (auto joint_name:start_pos_joint_name)
            {
                start_position_map[joint_name]=start_pos_value[counter];
                counter++;
            }

            if (start_position_map.size()!=start_pos_value.size())
            {
                RCLCPP_WARN(logger, "Joint names in yaml file should be  unique. Please check your yaml file. The start position value of duplicate joints will adapt the last value in yaml file. ");
            }
        }       

        RCLCPP_INFO(logger, "Setting start position for joints from robot_description");
        for (counter=0;counter<joints_count;counter++)
        {
            start_pos_it=start_position_map.find(joint_names_[counter]);
            if (start_pos_it!=start_position_map.end())
            {
                pos_[counter]=start_position_map[joint_names_[counter]];
                RCLCPP_INFO(logger, "Joint %s start position: %.3f", joint_names_[counter].c_str(), pos_[counter]);
            }
            else 
            {
                RCLCPP_WARN(logger, "Joint %s does NOT have start position data in yaml config file. Setting initial position to 0.0", joint_names_[counter].c_str());
            }
        }
        counter=0;
        cmd_=pos_;
    }

    for (auto & joint_name : joint_names_)
    {
        hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i] );
        joint_state_handle_[i]=state_handle;
                
        if (register_joint_state_handle(&joint_state_handle_[i])!=hardware_interface::HW_RET_OK)
        {
            RCLCPP_INFO(logger, "unable to register state handle [%d]: %s",i, joint_state_handle_[i].get_name().c_str());
            throw std::runtime_error("unable to register " + joint_state_handle_[i].get_name());
        }

        hardware_interface::JointCommandHandle cmd_handle(joint_name, &cmd_[i]);
        joint_cmd_handle_[i]=cmd_handle;
        if (register_joint_command_handle(&joint_cmd_handle_[i])!=hardware_interface::HW_RET_OK)
        {
            RCLCPP_INFO(logger, "unable to register command handle [%d]: %s",i, joint_cmd_handle_[i].get_name().c_str());
            throw std::runtime_error("unable to register " + joint_cmd_handle_[i].get_name());
        }
        
        
        hardware_interface::OperationModeHandle op_handle(joint_name, &op_mode_[i]);
        joint_op_mode_handle_[i]=op_handle;

        if (register_operation_mode_handle(&joint_op_mode_handle_[i])!=hardware_interface::HW_RET_OK)
        {
            RCLCPP_INFO(logger, "unable to register operation handle [%d]: %s",i, joint_op_mode_handle_[i].get_name().c_str());
            throw std::runtime_error("unable to register operation handle "+joint_op_mode_handle_[i].get_name());
        }

        ++i;
    }
    return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t
xARM::read()
{
    //Do nothing in fake_joint project; For real driver, please fill in the code to access the real hardware for each joint's position and update pos_, vel_ (optional) and eff_(optional)
    return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t
xARM::write()
{
    size_t counter=0;
    
    if (pos_!=cmd_)
    {
        for (counter=0;counter<pos_.size();counter++)
        {
            RCLCPP_INFO(logger, "cmd[%d]=%.3f; current pos[%d]=%.3f", counter, cmd_[counter], counter, pos_[counter]);
            if (std::abs(vel_[counter])>1e-9||std::abs(eff_[counter])>1e-19)
            {
                RCLCPP_INFO(logger, "vel[%d]=%.3f; effort[%d]=%.3f", counter, vel_[counter], counter, eff_[counter]);
            }
            
        }
        
        //For real driver, please fill in this function with the write operation to the real hardware. 
        pos_=cmd_;
    }
    return hardware_interface::HW_RET_OK;
}