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
#include <boost/thread.hpp>
#include "stepper_driver.h"
#include <stdlib.h>

StepperDriver::StepperDriver(int stepcount, int max_per, int min_per, 
        int steppin, int dirpin):
    position_(0), goal_(0), origin_(0),
    steppin_(steppin), dirpin_(dirpin), 
    dir_(FORWARD), stepcount_(stepcount)
{
    ROS_INFO("Initializing stepper_driver node...");

    //system("gpio export 4 out");
    //system("gpio export 28 out");
    pinMode(steppin, OUTPUT);
    pinMode(dirpin, OUTPUT);
    
    boost::thread t (boost::bind(&StepperDriver::update, this));
    t_.swap(t); // retain handle on update thread
}

void StepperDriver::update() {
    ros::Rate loop_time (100);
    int step, goal, delay_t;

    while (ros::ok()) {
        if (position_ != goal_) {

            // determine direction
            step = (position_ - goal_ > 0) ? 1 : -1;
            digitalWrite(dirpin_, step > 0);
            position_ += step;

            // protect against divide by zero
            goal = (goal_ == 0) ? 1 : goal_;

            // make delay inversely proportional to delta
            delay_t = (max_per_ - min_per_) * abs(position_ - goal_) / goal + min_per_;
            digitalWrite(steppin_, HIGH);
            delay(delay_t);
            digitalWrite(steppin_, LOW);
            delay(delay_t);
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

bool StepperDriver::MoveGoal(int ticks) {
    boost::lock_guard<boost::mutex> guard(mtx_);
    int goal_tmp = goal_ + ticks;
    if (goal_tmp >= 0 && goal_tmp <= stepcount_) {
        goal_ = goal_tmp;
        return true;
    } else {
        return false;
    }
}

bool StepperDriver::SetDirection(int dir) {
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

int StepperDriver::get_position() const 
{
    return position_;
}
