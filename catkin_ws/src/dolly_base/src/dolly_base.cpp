#include <ros/ros.h>

#include <ros/callback_queue.h>

#include "dolly_hardware_interface.hpp"

void controlLoop(dolly_hw::DollyHW& hardware, controller_manager::ControllerManager& cm, ros::Time& prev_time) {
	ros::Time cur_time = ros::Time::now();
	ros::Duration delta_time = cur_time - prev_time;
	prev_time = cur_time;

    hardware.read(delta_time);
	cm.update(cur_time, delta_time);
	hardware.write();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dolly_base");
	ros::NodeHandle root_node;
	ros::NodeHandle robot_hw_node("~");

    // ros::CallbackQueue callback_queue;
	// ros::AsyncSpinner spinner(1, &callback_queue);

    dolly_hw::DollyHW hardware(root_node, robot_hw_node);
    controller_manager::ControllerManager cm(&hardware, root_node);

    ros::Time begin = ros::Time::now();

    ros::Timer timer = root_node.createTimer (
        ros::Duration(0.01), 
        [&](const ros::TimerEvent&){controlLoop(hardware, cm, begin);}
    );


    // ros::CallbackQueue callback_queue;
	// ros::AsyncSpinner spinner(2, &callback_queue);
    // spinner.start();
    // ros::spin();

    ros::MultiThreadedSpinner spinner(2); // Don't work without multithread. Probably, controller_manager eats full work time
	spinner.spin();

	return 0;
}