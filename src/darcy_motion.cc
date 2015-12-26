#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <wiringPi.h>
#include <darcy_motion/Motion.h>
#include <unistd.h> // for usleep
#include <stdlib.h> // for system
#include "stepper.h"
#include "pins.h"

class DarcyMotion
{
    public:
        DarcyMotion();
        operator()();
     
    private:
        void teleopCallback(const darcy_motion::Motion::ConstPtr& msg);
        
        ros::NodeHandle n;
        ros::Subscriber teleop_sub_;
        
        StepperDriver stepA;
};

DarcyMotion::DarcyMotion()
{
    //wiringPiSetupSys();
    wiringPiSetup();

    teleop_sub_ = n.subscribe<darcy_motion::Motion>("teleop_commands", 1000, 
            &DarcyMotion::teleopCallback, this);

    stepA (200, 2300, 1000, 4, 28);
}

void DarcyMotion::teleopCallback(const darcy_motion::Motion::ConstPtr& msg) {
    //ROS_INFO("Got %f", msg->wrist_piv);
    if (!stepA_.Offset(msg->wrist_piv))
        ROS_WARN("Movement failed");
}

int main (int argc, char **argv)
{
    // init
    ROS_INFO("Initializing node darcy_motion...");
    ros::init(argc, argv, "darcy_motion");
    DarcyMotion darcy_motion;
    ROS_INFO("darcy_motion initialized. Waiting for input ...");
    //ros::Rate loop_rate(100);

    /*while (ros::ok()) {

        ros::spinOnce();     
        loop_rate.sleep();
        ROS_INFO("Still okay...");

    }*/

    ros::spin();

    return 0;
}
