#include <ros/ros.h>
#include <ros/console.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <string>
#include <vector>

namespace robot_control {
	struct Joint {
        const std::string name;
		double pos;
		// double pos_offset = 0;
		double vel;
		double eff;
		double cmd;

        Joint() = delete;
        Joint(const std::string& name) : name(name), pos(0), vel(0), eff(0), cmd(0) {}

        void print() {
            ROS_INFO_STREAM (
                "name: " << name << "\n"
                << "pos: " << pos << "\n"
                << "vel: " << vel << "\n"
                << "eff: " << eff << "\n"
                << "cmd: " << cmd << "\n"
            );
        }
	};

    class MyRobot : public hardware_interface::RobotHW {
        public:
            MyRobot() = delete;
            MyRobot(ros::NodeHandle&, ros::NodeHandle&);
            void write(const ros::Duration& delta);

        private:
            hardware_interface::JointStateInterface _jnt_state_interface;
            hardware_interface::VelocityJointInterface _vel_jnt_interface;

            Joint _left_wheel;
            Joint _right_wheel;

            ros::NodeHandle _root_node;
            ros::NodeHandle _robot_hw_node;
    };

    MyRobot::MyRobot(ros::NodeHandle& root_node, ros::NodeHandle& robot_hw_node) 
        : _root_node(root_node)
        , _robot_hw_node(robot_hw_node)
        , _left_wheel("left_wheel_to_base")
        , _right_wheel("right_wheel_to_base") {

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

    void MyRobot::write(const ros::Duration& delta) {
        std::vector<Joint*> joints = {&_left_wheel, &_right_wheel};

        for(auto& joint : joints) {
            joint->pos += joint->vel * delta.toSec();
            joint->vel = joint->cmd;
        }
    }
}

void controlLoop(robot_control::MyRobot& hardware, controller_manager::ControllerManager& cm, ros::Time& prev_time) {
	ros::Time cur_time = ros::Time::now();
	ros::Duration delta_time = cur_time - prev_time;
	prev_time = cur_time;

    hardware.write(delta_time);
	cm.update(cur_time, delta_time);
	hardware.write(delta_time);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dolly_base");
	ros::NodeHandle root_node;
	ros::NodeHandle robot_hw_node("~");

    robot_control::MyRobot hardware(root_node, robot_hw_node);
    controller_manager::ControllerManager cm(&hardware, root_node);

    ros::Time begin = ros::Time::now();

    ros::Timer timer = robot_hw_node.createTimer (
        ros::Duration(0.1), 
        [&](const ros::TimerEvent&){controlLoop(hardware, cm, begin);}
    );


    ros::MultiThreadedSpinner spinner(2); // Don't work without multithread. Probably, controller_manager eats full work time
	spinner.spin();

	return 0;
}