#include "ros/ros.h"
//#include< <ros/subscribe_options.h>
#include <sensor_msgs/Joy.h>

using namespace std;

class joystick{
public:
void callback(const sensor_msgs::Joy::ConstPtr& joy);
};

void joystick::callback(const sensor_msgs::Joy::ConstPtr& joy){
  for (int i =0 ;i<joy->buttons.size();i++){
	cout<< joy->buttons[i] << " ";
  }
  cout << endl;
  cout << "   " << joy->axes[0] << "  " << joy->axes[1] << "  "<< joy->axes[3] <<  "  " << joy->axes[4] << endl;
}

int main(int argc, char **argv){
  joystick listen;
  printf("Main begun\n");
  ros::init(argc, argv, "joystick_dem");
  ros::NodeHandle rosnode;
  system("rosrun joy joy_node&");

  printf("ROS started\n");

/*
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait){
    last_ros_time = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }
*/
  printf("Creating Subscriber\n");
  ros::Subscriber joy_sub = rosnode.subscribe<sensor_msgs::Joy>("joy",10,&joystick::callback,&listen);
  printf("Spinning...\n\n");
  ros::spin();
}
