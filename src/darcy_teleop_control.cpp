#include <ros/ros.h>
#include "darcy_motion/Motion.h"
#include <sensor_msg/Joy.h>

class TeleopDarcy
{
public:
    TeleopDarcy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);


    float wrist_vel;
    ros::NodeHandle nh_;
    ros::Publisher mot_pub;
    ros::Subscriber joy_sub;

};

TeleopDarcy::TeleopTurtle() {

    mot_pub = nh_.advertise<darcy_motion::Motion>("d

}

