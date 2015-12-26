#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <wiringPi.h>
#include <darcy_motion/Motion.h>
#include <unistd.h> // for usleep
#include <stdlib.h> // for system
#include "stepper_driver.h"
#include "pins.h"

class DarcyMotion
{
    public:
        DarcyMotion();
     
    private:
        void teleopCallback(const darcy_motion::Motion::ConstPtr& msg);
        
        ros::NodeHandle n_;
        ros::Subscriber teleop_sub_;
        
        StepperDriver stepA_;
};

DarcyMotion::DarcyMotion():
    stepA_(200, 2300, 1000, STEP_A_STEP, STEP_A_DIR)
{
    //wiringPiSetupSys();
    wiringPiSetup();

    teleop_sub_ = n_.subscribe<darcy_motion::Motion>("teleop_commands", 1000, 
            &DarcyMotion::teleopCallback, this);

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
