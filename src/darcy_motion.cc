#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <wiringPi.h>
#include <darcy_motion/Motion.h>
#include <unistd.h> // for usleep
#include <stdlib.h> // for system
#include <signal.h>
#include <string.h>
#include <exception>
#include "stepper_driver.h"
#include "pins.h"

#define CMD_BUFFER_SIZE 64

char *read_line (char *buf, size_t length, FILE *f);

class DarcyMotion
{
    public:
        DarcyMotion();
        void Calibrate();
        void GoTo(int position);
    private:
        void teleopCallback(const darcy_motion::Motion::ConstPtr& msg);

        ros::NodeHandle n_;
        ros::Subscriber teleop_sub_;
        
        StepperDriver step_waist_, step_shoulder_, step_fore_;

};

DarcyMotion::DarcyMotion():
    step_waist_(300, 8000, 1000, STEP_WAIST_STEP, STEP_WAIST_DIR, STEP_WAIST_LIM),
    step_shoulder_(400, 8000, 1000, STEP_SHOULDER_STEP, STEP_SHOULDER_DIR, STEP_SHOULDER_LIM),
    step_fore_(200, 8000, 1000, STEP_FOREARM_STEP, STEP_FOREARM_DIR, STEP_FOREARM_LIM)
{
    //wiringPiSetupSys();
    wiringPiSetup();

    // Initialize pins
    //system("gpio export 4 out");
    //system("gpio export 28 out");
    pinMode(STEP_WAIST_STEP, OUTPUT);
    pinMode(STEP_WAIST_DIR, OUTPUT);
    pinMode(STEP_WAIST_LIM, INPUT);
    pinMode(STEP_SHOULDER_STEP, OUTPUT);
    pinMode(STEP_SHOULDER_DIR, OUTPUT);
    pinMode(STEP_SHOULDER_LIM, INPUT);
    pinMode(STEP_FOREARM_STEP, OUTPUT); 
    pinMode(STEP_FOREARM_DIR, OUTPUT);
    pinMode(STEP_FOREARM_LIM, INPUT);


    // uncomment for teleop
    step_waist_.SetMode(TELE_MODE);
    step_shoulder_.SetMode(TELE_MODE);
    step_fore_.SetMode(TELE_MODE);

    teleop_sub_ = n_.subscribe<darcy_motion::Motion>("teleop_commands", 1000, 
            &DarcyMotion::teleopCallback, this);

}

void DarcyMotion::Calibrate() {
    //stepA_.Calibrate();
}

void DarcyMotion::GoTo(int position) {
    //stepA_.GoTo(position);
}

void DarcyMotion::teleopCallback(const darcy_motion::Motion::ConstPtr& msg) {

    //ROS_INFO("Got teleop_command message!");

    // uncomment for teleop 
    if (!step_waist_.Set(msg->waist)) ROS_WARN("Waist movement failed");
    if (!step_shoulder_.Set(msg->shoulder)) ROS_WARN("Shoulder movement failed");
    if (!step_fore_.Set(msg->elbow)) ROS_WARN("Forearm movement failed");

}

int main (int argc, char **argv)
{
    // init
    ROS_INFO("Initializing node darcy_motion...");
    ros::init(argc, argv, "darcy_motion");
    DarcyMotion darcy_motion;
    ROS_INFO("darcy_motion initialized. Waiting for input ...");
/*
    int exit = 0;
    char *p, buffer [CMD_BUFFER_SIZE];

    while (1) {
        printf("> ");
        p = read_line(buffer, CMD_BUFFER_SIZE, stdin);
    
        if (strcmp("e", buffer) == 0) {
            return 0;
        }
        else if (strcmp("c", buffer) == 0) {
            darcy_motion.Calibrate();
        }

        // check for numbers for positions
        else 
        {
            for (int i = 0; i < strlen(buffer); i++) {
                if (!isdigit(buffer[i])) continue;
            }

            int position = atoi(buffer);
            ROS_INFO("Going to position %d", position);
            darcy_motion.GoTo(position);
        }


        ros::spinOnce();
    }
    */
    ros::spin();
}

char *read_line (char *buf, size_t length, FILE *f)
/**** 
 * Read at most 'length'-1 characters from the file 'f' into
 * 'buf' and zero-terminate this character sequence. If the
 * line contains more characters, discard the rest.
 ****/
{
    char *p;

    if (p = fgets (buf, length, f)) {
        size_t last = strlen (buf) - 1;

        if (buf[last] == '\n') {
            /**** Discard the trailing newline */
            buf[last] = '\0';
        } else {
            /**** There's no newline in the buffer, therefore there must be
             *             more characters on that line: discard them!
             *                    */
            fscanf (f, "%*[^\n]");
            /**** And also discard the newline... */
            (void) fgetc (f);
        } /* end if */
    } /* end if */
    return p;
}
