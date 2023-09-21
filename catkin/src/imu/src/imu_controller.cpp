#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>

int main(int argc, char** argv){
	ros::init(argc, argv, "imu");

	ros::NodeHandle n;

	ros::Publisher int_pub = n.advertise<std_msgs::Int32>("Chatter", 1000);

	ROS_INFO("Starting...");

	ros::Rate loop_rate(10);


	int count = 0;
	while(ros::ok()){

		std_msgs::Int32 msg;

		msg.data = count;
		int_pub.publish(msg);

		ros::spinOnce();

		count++;
		loop_rate.sleep();
	}
	ROS_INFO("Shutting down...");

	return 0;
	
}
