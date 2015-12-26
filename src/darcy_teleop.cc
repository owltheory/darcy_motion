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

    mot_pub = nh_.advertise<darcy_motion::Motion>("teleop_commands", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy",10,&TeleopDarcy::joyCallback,this);

}

void TeleopDarcy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    darcy_motion::Motion motion;
    motion.base = motion.shoulder = motion.elbow = motion.wrist_rot = 0.0;
    motion.wrist_piv = joy->axes[1];
    motion.grip_closed = true;
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
