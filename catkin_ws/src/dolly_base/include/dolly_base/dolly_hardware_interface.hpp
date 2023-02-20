#ifndef DOLLY_HW
#define DOLLY_HW

#include <ros/ros.h>
#include <ros/console.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <string>
#include <vector>

namespace dolly_hw {
	struct Joint {
        const std::string name;
		double pos = 0;
        double prev_pos = 0;
		// double pos_offset = 0;
		double vel = 0;
		double eff = 0;
		double cmd = 0;

        Joint() = delete;
        Joint(const std::string& name) : name(name) {}

        // void print() {
        //     ROS_INFO_STREAM (
        //         "name: " << name << "\n"
        //         << "pos: " << pos << "\n"
        //         << "vel: " << vel << "\n"
        //         << "eff: " << eff << "\n"
        //         << "cmd: " << cmd << "\n"
        //     );
        // }
	};

    class DollyHW : public hardware_interface::RobotHW {
        public:
            DollyHW() = delete;
            DollyHW(ros::NodeHandle&, ros::NodeHandle&);

            void read(const ros::Duration& delta);
            void write();

        private:
            void _registerJoints();

            void _leftWheelCallback(const std_msgs::Float32&);
            void _rightWheelCallback(const std_msgs::Float32&);

            hardware_interface::JointStateInterface _jnt_state_interface;
            hardware_interface::VelocityJointInterface _vel_jnt_interface;

            Joint _left_wheel;
            Joint _right_wheel;

            std_msgs::Float32MultiArray servo_cmd;

            ros::NodeHandle _root_node;
            ros::NodeHandle _robot_hw_node;

            ros::Subscriber _left_wheel_angle_sub;
            ros::Subscriber _right_wheel_angle_sub;
            ros::Publisher _left_wheel_cmd_vel_pub;
            ros::Publisher _right_wheel_cmd_vel_pub;
            ros::Publisher _servo_cmd_pub;
    };

    DollyHW::DollyHW(ros::NodeHandle& root_node, ros::NodeHandle& robot_hw_node) 
        : _root_node(root_node)
        , _robot_hw_node(robot_hw_node)
        , _left_wheel("left_wheel_to_base")
        , _right_wheel("right_wheel_to_base") {

        _registerJoints();

        servo_cmd.layout.dim.push_back(std_msgs::MultiArrayDimension());
        servo_cmd.layout.dim[0].size = 3;
        servo_cmd.layout.dim[0].stride = 1;
        servo_cmd.layout.dim[0].label = "servo_cmd";
        //servo_cmd.data = {50, 50, 50};

        _left_wheel_angle_sub = _root_node.subscribe("/dolly/left_wheel/angle32", 1, &DollyHW::_leftWheelCallback, this);
        _right_wheel_angle_sub = _root_node.subscribe("/dolly/right_wheel/angle32", 1, &DollyHW::_rightWheelCallback, this);

        _left_wheel_cmd_vel_pub = _root_node.advertise<std_msgs::Float64>("/dolly/left_wheel/cmd_vel64", 1);
        _right_wheel_cmd_vel_pub = _root_node.advertise<std_msgs::Float64>("/dolly/right_wheel/cmd_vel64", 1);

        _servo_cmd_pub = _root_node.advertise<std_msgs::Float32MultiArray>("/servo_cmd_topic",1);
        
    }

    void DollyHW::_registerJoints() {
        std::vector<Joint*> joints = {&_left_wheel, &_right_wheel};

        for(auto& joint : joints) {
            hardware_interface::JointStateHandle state_handle (
                joint->name, 
                &(joint->pos),
                &(joint->vel), 
                &(joint->eff)
            );

            _jnt_state_interface.registerHandle(state_handle);

            hardware_interface::JointHandle vel_handle(state_handle, &(joint->cmd));
            _vel_jnt_interface.registerHandle(vel_handle);
        }

        registerInterface(&_jnt_state_interface);
        registerInterface(&_vel_jnt_interface);
    }

    void DollyHW::read(const ros::Duration& delta) {
        std::vector<Joint*> joints = {&_left_wheel, &_right_wheel};

        // for(auto& joint : joints) {
        //     joint->pos += joint->cmd * delta.toSec();
        //     joint->vel = joint->cmd;
        // }

        for(auto& joint : joints) {
            joint->vel = (joint->pos - joint->prev_pos) / delta.toSec();
            joint->prev_pos = joint->pos;
        }
    }

    void DollyHW::write() {
        std_msgs::Float64 left_cmd_vel;
        std_msgs::Float64 right_cmd_vel;

        left_cmd_vel.data = _left_wheel.cmd;
        right_cmd_vel.data = _right_wheel.cmd;

        _left_wheel_cmd_vel_pub.publish(left_cmd_vel);
        _right_wheel_cmd_vel_pub.publish(right_cmd_vel);

        //_servo_cmd_pub.publish(servo_cmd);
    }

    void DollyHW::_leftWheelCallback(const std_msgs::Float32& angle) {
        _left_wheel.pos = angle.data;
    }

    void DollyHW::_rightWheelCallback(const std_msgs::Float32& angle) {
        _right_wheel.pos = angle.data;
    }
}

#endif // DOLLY_HW