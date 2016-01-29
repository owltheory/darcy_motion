#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <darcy_motion/Motion.h>

class TeleopDarcy
{
public:
    TeleopDarcy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;
    ros::Publisher mot_pub;
    ros::Subscriber joy_sub;

};

TeleopDarcy::TeleopDarcy() {

    nh_.setParam("/joy_node/deadzone", 0.5);
    nh_.setParam("/joy_node/autorepeat_rate", 20);
    mot_pub = nh_.advertise<darcy_motion::Motion>("teleop_commands", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("/joy",10,&TeleopDarcy::joyCallback,this);

}

void TeleopDarcy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    darcy_motion::Motion motion;
    motion.waist = joy->axes[0];
    motion.shoulder = joy->axes[1];
    motion.elbow = joy->axes[3];
    motion.grip = true;
    mot_pub.publish(motion);

}

int main(int argc, char** argv)
{
    ROS_INFO("darcy_teleop initializing...");
    ros::init(argc, argv, "darcy_teleop");
    TeleopDarcy teleop_darcy;
    ROS_INFO("darcy_teleop running");

    ros::spin();
}
