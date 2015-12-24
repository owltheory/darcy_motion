#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <wiringPi.h>
#include "darcy_motion/Motion.h"


void teleopCallback(const darcy_motion::Motion::ConstPtr& msg) {
    ROS_INFO("Got %f", msg->wrist_piv);
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "darcy_motion");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("teleop_commands", 1000, teleopCallback);

    ros::spin();

    return 0;
}
