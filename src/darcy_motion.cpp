#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <wiringPi.h>
#include <darcy_motion/Motion.h>
#include <unistd.h> // for usleep
#include <stdlib.h> // for system

class DarcyMotion
{
    public:
        DarcyMotion();
        void spin();
     
    private:
        void teleopCallback(const darcy_motion::Motion::ConstPtr& msg);
        
        int dir_, delayT_;
        long steps_;
        int stepPin = 4, dirPin = 28;

        ros::NodeHandle n;
        ros::Subscriber teleop_sub_;
};

DarcyMotion::DarcyMotion():
    dir_(0), delayT_(0), steps_(0)
{
    wiringPiSetupSys();
    //pinMode(stepPin,OUTPUT);
    //pinMode(dirPin,OUTPUT);
    system("gpio export 4 out");
    system("gpio export 28 out");

    teleop_sub_ = n.subscribe<darcy_motion::Motion>("teleop_commands", 1000, 
            &DarcyMotion::teleopCallback, this);
}

void DarcyMotion::teleopCallback(const darcy_motion::Motion::ConstPtr& msg) {
    //ROS_INFO("Got %f", msg->wrist_piv);
    delayT_ = (msg->wrist_piv < 1000) ? 1000 : msg->wrist_piv;
    dir_ = (msg->wrist_piv > 0) ? 0 : 1;
    steps_++;
}

void DarcyMotion::spin() {

    if (steps_ > 0) {
        ROS_INFO("Steps: %d Delay: %d", steps_, delayT_);

        digitalWrite(dirPin, dir_);

        digitalWrite(stepPin, HIGH);
        usleep(delayT_);
        digitalWrite(stepPin,LOW);
        usleep(delayT_);

        steps_--;
    }
}


int main (int argc, char **argv)
{
    // init
    ROS_INFO("Initializing node darcy_motion...");
    ros::init(argc, argv, "darcy_motion");
    DarcyMotion darcy_motion;
    ROS_INFO("darcy_motion initialized. Waiting for input ...");
    ros::Rate loop_rate(100);

    while (ros::ok()) {

        darcy_motion.spin();
        ros::spinOnce();     
        loop_rate.sleep();
        ROS_INFO("Still okay...");

    }

    return 0;
}
