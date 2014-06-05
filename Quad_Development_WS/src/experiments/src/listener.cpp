#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"

void chatterCallback(const std_msgs::Time::ConstPtr& msg){
	ROS_INFO("Ding!\n");
}

int main(int argc, char **argv){

	ros::init(argc,argv,"listener");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("geometry_msgs:TwistStamped", 1000,chatterCallback);
	ros::spin();

	return 0;

}
