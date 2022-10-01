#include <ros/ros.h>

#include <std_msgs/Float64.h>

struct Wheel {
    double angle = 0;
    double vel = 0;

    ros::Time last_call;

    ros::Subscriber cmd_vel_sub;

    ros::Publisher cur_vel_pub;
    ros::Publisher angle_pub;

    Wheel() {}
    Wheel(ros::Subscriber, ros::Publisher, ros::Publisher);
};

Wheel::Wheel (ros::Subscriber cmd_vel_sub, ros::Publisher cur_vel_pub, ros::Publisher angle_pub)
    : cmd_vel_sub(cmd_vel_sub), cur_vel_pub(cur_vel_pub), angle_pub(angle_pub) {
        last_call = ros::Time::now();
    }

class HW_Emulator {
    public:
        HW_Emulator(const ros::NodeHandle& node);

    private:
        void _wheelVelCallback(Wheel&, const std_msgs::Float64&);
        void _leftWheelCallback(const std_msgs::Float64&);
        void _rightWheelCallback(const std_msgs::Float64&);
        // void _leftWheelVelCallback(const std_msgs::Float64&);
        // void _rightWheelVelCallback(const std_msgs::Float64&);

        ros::NodeHandle _node;

        // ros::Subscriber _left_wheel_cmd_vel_sub;
	    // ros::Subscriber _right_wheel_cmd_vel_sub;

	    // ros::Publisher _left_wheel_cur_vel_pub;
	    // ros::Publisher _right_wheel_cur_vel_pub;
        // ros::Publisher _left_wheel_angle_pub;
	    // ros::Publisher _right_wheel_angle_pub;

        Wheel _left_wheel;
        Wheel _right_wheel;
};

void foo(std_msgs::Float64) {}

HW_Emulator::HW_Emulator(const ros::NodeHandle& node) : _node(node) {
    _left_wheel = Wheel (
        _node.subscribe("/dolly/left_wheel/cmd_vel", 1, &HW_Emulator::_leftWheelCallback, this),
        _node.advertise<std_msgs::Float64>("/dolly/left_wheel/cur_vel", 1),
        _node.advertise<std_msgs::Float64>("/dolly/left_wheel/angle", 1)
    );

    _right_wheel = Wheel (
        _node.subscribe("/dolly/right_wheel/cmd_vel", 1, &HW_Emulator::_rightWheelCallback, this),
        _node.advertise<std_msgs::Float64>("/dolly/right_wheel/cur_vel", 1),
        _node.advertise<std_msgs::Float64>("/dolly/right_wheel/angle", 1)
    );

    // _left_wheel_cmd_vel_sub = _node.subscribe("/dolly/left_wheel/cmd_vel", 1, &HW_Emulator::_leftWheelVelCallback, this);
    // _right_wheel_cmd_vel_sub = _node.subscribe("/dolly/right_wheel/cmd_vel", 1, &HW_Emulator::_rightWheelVelCallback, this);

    // _left_wheel_cur_vel_pub = _node.advertise<std_msgs::Float64>("/dolly/left_wheel/cur_vel", 1);
    // _right_wheel_cur_vel_pub = _node.advertise<std_msgs::Float64>("/dolly/right_wheel/cur_vel", 1);
    // _left_wheel_angle_pub = _node.advertise<std_msgs::Float64>("/dolly/left_wheel/angle", 1);
    // _right_wheel_angle_pub = _node.advertise<std_msgs::Float64>("/dolly/right_wheel/angle", 1);

    // ros::Time _left_wheel_last_call = ros::Time::now();
    // ros::Time _right_wheel_last_call = ros::Time::now();
}

void HW_Emulator::_wheelVelCallback(Wheel& wheel, const std_msgs::Float64& cmd_vel) {
    ros::Time now = ros::Time::now();
    ros::Duration delta = now - wheel.last_call;
    wheel.last_call = now;

    wheel.angle += wheel.vel * delta.toSec();
    wheel.vel = cmd_vel.data;

    std_msgs::Float64 angle;
    angle.data = wheel.angle;

    wheel.cur_vel_pub.publish(cmd_vel);
    wheel.angle_pub.publish(angle);
}

void HW_Emulator::_leftWheelCallback(const std_msgs::Float64& cmd_vel) {
    // Stupid, but somehow subscribe doesn't work with lambda
    _wheelVelCallback(_left_wheel, cmd_vel);
}

void HW_Emulator::_rightWheelCallback(const std_msgs::Float64& cmd_vel) {
    // Stupid, but somehow subscribe doesn't work with lambda
    _wheelVelCallback(_right_wheel, cmd_vel);
}

// void HW_Emulator::_leftWheelVelCallback(const std_msgs::Float64& cmd_vel) {
//     ros::Time now = ros::Time::now();
//     ros::Duration delta = now - _left_wheel.last_call;

//     _left_wheel.angle += _left_wheel.vel * delta.toSec();
//     _left_wheel.vel = cmd_vel.data;
//     _left_wheel.last_call = now;

//     std_msgs::Float64 angle;
//     angle.data = _left_wheel.angle;

//     _left_wheel_cur_vel_pub.publish(cmd_vel);
//     _left_wheel_angle_pub.publish(angle);
// }

// void HW_Emulator::_rightWheelVelCallback(const std_msgs::Float64& cmd_vel) {
//     ros::Time now = ros::Time::now();
//     ros::Duration delta = now - _right_wheel.last_call;

//     _right_wheel.angle += _right_wheel.vel * delta.toSec();
//     _right_wheel.vel = cmd_vel.data;
//     _right_wheel.last_call = now;

//     std_msgs::Float64 angle;
//     angle.data = _right_wheel.angle;

//     _right_wheel_cur_vel_pub.publish(cmd_vel);
//     _right_wheel_angle_pub.publish(angle);
// }

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "hw_emulator");
	ros::NodeHandle node;

    HW_Emulator hw_emulator(node);

    ros::spin();

	return 0;
}