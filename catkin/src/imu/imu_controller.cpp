#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <sstream>

int main(int argc, char** argv){
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher int_pub = n.advertise<std_msgs::Int32>("Chatter", 1000);

	ros::Rate loop_rate(10);


	int count = 0;
	while(ros::ok()){

		int_pub.publish(count);

		ros::spinOnce();

		count++;
		loop_rate.sleep();
	}

	return 0;
	
}
