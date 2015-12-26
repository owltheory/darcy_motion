#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <cmath>
#include <string.h>
#include <boost/thread.hpp>

#define FORWARD 1  
#define BACKWARD 0

class StepperDriver {
    public:
        StepperDriver();
        bool Offset(float percent);
        //void Set();
        int get_position() const;
    private:
        bool SetDirection(int dir);
        void MoveGoal(int ticks);
        void update();
        int stepcount_, origin_;
        int position_, goal_;
        int steppin_, dirpin_; 
        int max_per_, min_per_;
        boost::thread t_;
        boost::mutex mtx_;
}

