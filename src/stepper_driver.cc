/**
 * stepper_driver.cpp
 * A node for controlling a stepper.
 * TODO:
 *  -Make different velocity modes for resolving goal
 *  Presently velocity is inversely proportional 
 *  to distance from goal.
 *  -limit switch calibration
 **/

#include "stepper.h"

StepperDriver::StepperDriver(int stepcount, int max_per, int min_per, 
        int steppin, int dirpin):
    position_(0), goal_(0), origin_(0);
    steppin_(steppin), dirpin_(dirpin), 
    stepcount_(stepcount), topic_(topic)
{
    ROS_INFO("Initializing stepper_driver node...");

    //system("gpio export 4 out");
    //system("gpio export 28 out");
    pinMode(steppin, OUTPUT);
    pinMode(dirpin, OUTPUT);
   
    // lauch update thread 
    t_(update);
    
}
k_

void update() {
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

bool StepperDriver::Offset(float percent) {
    int ticks = percent * stepcount_;
    dir_ = ticks < 0 ? BACKWARD : FORWARD;

    // TODO: wrap / take into account absolute
    /*if (position_ > stepcount_) {
        position_ %= stepcoutn_;
    }*/

   return MoveGoal(ticks);
}

bool MoveGoal(int ticks) {
    boost::lock_guard<boost::mutex> guard(mtx_);
    goal_tmp = goal + ticks;
    if (goal_tmp >= 0 && goal_tmp <= stepcount_) {
        goal_ = goal_tmp;
        return true;
    } else {
        return false;
    }
}

bool SetDirection(int dir) {
    boost::lock_guard<boost::mutex> guard(mtx_);
    if (dir == FORWARD || dir == BACKWARD) {
        dir_ = dir;
        return true;
    } else {
        return false;
    }
}


/*
void Set() {


}*/

int StepperDriver::get_position() {
    return position;
}
