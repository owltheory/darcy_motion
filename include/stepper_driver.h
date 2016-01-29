#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <cmath>
#include <string.h>
#include <boost/thread.hpp>

#define FORWARD 1  
#define BACKWARD 0

#define TELE_MODE 0 
#define AUTO_MODE 1
#define CALIB_MODE 2

class StepperDriver {
    public:
        StepperDriver(int, int, int, int, int, int);
        bool Offset(float percent);
        bool Set(float rate);
        bool SetMode(int mode);
        void Calibrate();
        bool GoTo(int);
        int get_position() const;
    private:
        bool SetDirection(int dir);
        bool MoveGoal(int ticks);
        void update();
        int stepcount_, origin_, dir_;
        int position_, goal_;
        int steppin_, dirpin_, limitpin_; 
        int max_per_, min_per_; // in microseconds
        int mode_, period_; 
        boost::mutex mtx_;
        boost::thread t_;
};

