#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

void leftWheelCallback(const std_msgs::Float64& cmd_vel);
void rightWheelCallback(const std_msgs::Float64& cmd_vel);

ros::Publisher left_wheel_pub;
ros::Publisher right_wheel_pub;

ros::Subscriber left_wheel_sub;
ros::Subscriber right_wheel_sub;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "stm_topic_converter");
	ros::NodeHandle node;

    left_wheel_sub = node.subscribe("/dolly/left_wheel/cmd_vel64", 1, leftWheelCallback);
    right_wheel_sub = node.subscribe("/dolly/right_wheel/cmd_vel64", 1, rightWheelCallback);

    left_wheel_pub = node.advertise<std_msgs::Float32>("/dolly/left_wheel/cmd_vel", 1);
    right_wheel_pub = node.advertise<std_msgs::Float32>("/dolly/right_wheel/cmd_vel", 1);

    ros::spin();

	return 0;
}

void leftWheelCallback(const std_msgs::Float64& cmd_vel) {
    std_msgs::Float32 converted;
    converted.data = cmd_vel.data;

    left_wheel_pub.publish(converted);
}

void rightWheelCallback(const std_msgs::Float64& cmd_vel) {
    std_msgs::Float32 converted;
    converted.data = cmd_vel.data;

    right_wheel_pub.publish(converted);
}