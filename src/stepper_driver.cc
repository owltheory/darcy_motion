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
#include <unistd.h>
#include <wiringPi.h>

StepperDriver::StepperDriver(int stepcount, int max_per, int min_per, 
        int steppin, int dirpin, int limitpin):
    position_(0), goal_(0), origin_(0),
    steppin_(steppin), dirpin_(dirpin), limitpin_(limitpin),
    dir_(FORWARD), stepcount_(stepcount),
    min_per_(min_per), max_per_(max_per),
    mode_(AUTO_MODE) 
{
    ROS_INFO("Initializing stepper_driver ...");
    boost::thread t (boost::bind(&StepperDriver::update, this));
    t_.swap(t); // retain handle on update thread
    SetDirection(dir_);
}

void StepperDriver::update() {
    ros::Rate loop_time (100);
    int step, goal, delay_t;
    int limit_state;

    while (ros::ok()) {

        //limit_state = digitalRead(limitpin_);

        /*
         * AUTO_MODE update
         */
        switch (mode_) { 
            case AUTO_MODE:

                if (position_ != goal_) {

                    position_ += (dir_ == FORWARD) ? 1 : -1;

                    // protect against divide by zero
                    goal = (goal_ == 0) ? 1 : goal_;

                    // make delay inversely proportional to delta
                    delay_t = (max_per_ - min_per_) * abs(position_ - goal_) 
                        / stepcount_ + min_per_;

                    // step
                    digitalWrite(steppin_, HIGH);
                    usleep(delay_t);
                    digitalWrite(steppin_, LOW);
                    usleep(delay_t);
                }
                else {
                    ros::spinOnce();
                }
                break;

            /*
            * TELEOP update
            */
            case TELE_MODE: 

                if (period_ > 0) {

                    digitalWrite(steppin_, HIGH);
                    usleep(period_);
                    digitalWrite(steppin_, LOW);
                    usleep(period_);
                }

                break;        

            case CALIB_MODE:
                /**
                 * CALIBration update
                 */

                limit_state = digitalRead(limitpin_);

                if (limit_state) {
                    position_ = goal = 0;
                    ROS_INFO("Calibration successful.");
                    SetDirection(FORWARD);
                    SetMode(AUTO_MODE);
                }
                else if (position_ >= stepcount_) {
                    ROS_WARN("Calibration failed.");
                    SetMode(AUTO_MODE);
                    position_ = 0;
                }

                else  {

                    digitalWrite(steppin_, HIGH);
                    usleep(period_);
                    digitalWrite(steppin_, LOW);
                    usleep(period_);
                    position_++;
                }
                break;
        }
    }
}

void StepperDriver::Calibrate() {

    // move at 1/5 speed until we find the limit 
    period_ = (max_per_ - min_per_) * 0.8 + min_per_;
    SetMode(CALIB_MODE);
    ROS_INFO("Starting calibration...");
    position_ = 0;

}

/*
 * SetMode
 * Changes the current operation mode of this stepper driver.
 * Determines how stepper is updated.
 */
bool StepperDriver::SetMode(int mode) {
    if (mode == TELE_MODE || 
        mode == AUTO_MODE ||
        mode == CALIB_MODE) {

        ROS_DEBUG("Changing stepper to %i mode!", mode);
        mode_ = mode;
        return true;    
    } else {
        return false;
    }
}

/**
 * Set
 * Primarily a teleop function, this directly sets the tick period to the
 * passed rate. Direction determined by sign. Directly interfaceable with
 * controller.
 */
bool StepperDriver::Set(float rate) {

    // check for valid rate
    if (rate > 1 || rate < -1) {
        period_ = 0;
        return false;
    }
    // check for non-zero rate
    else if (rate == 0.0) {
        period_ = 0;
        return true;
    } else {
        // set rate according to sign
        dir_ = rate < 0 ? BACKWARD : FORWARD;
        digitalWrite(dirpin_, dir_);
        // update position
        position_ += dir_;
        // scale rate according to min/max periods
        rate = fabs(rate);
        period_ = ((max_per_ - min_per_) * fabs(1 - rate)) + min_per_; 
        ROS_INFO("Period: %d", period_);
        return true;
    }
}

/**
 * Takes in a percentage of total ticks / 16 with which
 * to offset the goal.
 * float percent: may be positive or negative.
 **/
bool StepperDriver::Offset(float percent) {
    if (percent == 0.0) return true;
    int ticks = percent * stepcount_ / 32;
    dir_ = ticks < 0 ? BACKWARD : FORWARD;

    // TODO: wrap / take into account absolute
    /*if (position_ > stepcount_) {
      position_ %= stepcoutn_;
      }*/

    return MoveGoal(ticks);
}

/**
 * This function should be used when offsetting
 * the goal member variable because it is thread
 * safe. Takes in a positive number of ticks that
 * must be less than this instance's value of 
 * total stepcount_
 */
bool StepperDriver::MoveGoal(int ticks) {
    boost::lock_guard<boost::mutex> guard(mtx_);
    int goal_tmp = goal_ + ticks;
    if (goal_tmp >= 0 && goal_tmp <= stepcount_) {
        goal_ = goal_tmp;
        ROS_WARN("Move goal successful.");
        return true;
    } else {
        ROS_WARN("Failed to change goal. \n Params: motor_steps: %d goal: %d attempted goal: %d",
                stepcount_, goal_, goal_tmp);
        return false;
    }
}

/**
 * GoTo (position)
 * Specify a position. Must be greater than -1 and less than stepcount_.
 **/
bool StepperDriver::GoTo(int new_position) {
    boost::lock_guard<boost::mutex> guard(mtx_);

    // check for valid tick
    if (new_position < 0 || new_position >= stepcount_) {
        ROS_WARN("Invalid position specified");
        return false;
    }
    
    if (new_position > position_) {
        SetDirection(FORWARD);
    } else if (new_position < position_) {
        SetDirection(BACKWARD);
    }
    
    goal_ = new_position;
    return true;
}

/**
 * SetDirection
 * Changes the direction of rotation.
 */
bool StepperDriver::SetDirection(int dir) {
    if (dir == FORWARD || dir == BACKWARD) {
        dir_ = dir;
        digitalWrite(dirpin_, dir);
        return true;
    } else {
        return false;
    }
}

int StepperDriver::get_position() const 
{
    return position_;
}
