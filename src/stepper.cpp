/**
 * stepper_driver.cpp
 * A node for controlling a stepper.
 * TODO:
 *  -Make different velocity modes for resolving goal
 *  Presently velocity is inversely proportional 
 *  to distance from goal.
 *  -limit switch calibration
 **/


#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <cmath>
#include <string.h>
#include <boost/thread.hpp>

class StepperDriver {

    public:
        StepperDriver();
        //bool calibrate();
        int getPosition() const;
        void ChangeGoal();
        void spin();

    private:
        void callback();

        int stepcount_, origin_;
        int position_, goal_;
        int steppin_, dirpin_; 
        int max_per_, min_per_;

};


StepperDriver::StepperDriver(int stepcount, int max_per, int min_per, 
        int steppin, int dirpin):
    position_(0), goal_(0), origin_(0);
    steppin_(steppin), dirpin_(dirpin), 
    stepcount_(stepcount), topic_(topic)
{
    ROS_INFO("Initializing stepper_driver node...");

    pinMode(steppin, OUTPUT);
    pinMode(dirpin, OUTPUT);

    
}

/*(StepperDriver::calibrate(); {


  }*/

void StepperDriver::changeGoal(int offset) {
     if (position_ > stepcount_) {
        position_ %= stepcoutn_;
    }
}   goal += offset;
}

int StepperDriver::getPosition() {
    return position;
}

void StepperDriver::spin() {
    ros::Rate loop_time (100);

    while (ros::okay()) {
        if (position_ != goal_) {

            // determine direction
            int dir = (position_ - goal_ > 0) ? 1 : -1;
            digitalWrite(dirpin_, dir > 0);
            position += dir;

            // protect against divide by zero
            int goal = (goal_ == 0) ? 1 : goal_;

            // make delay inversely proportional to delta
            int delay = (max_per_ - min_per_) * abs(position_ - goal_) / goal + min_per_;
            digitalWrite(stepPin, HIGH);
            delay(delay);
            digitalWrite(stepPin, LOW);
            delay(delay);
        }
        else {
            loop_time.sleep();
        }

        ros::spinOnce();
    }

}
