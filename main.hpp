/*============================================================================*/

/* Libraries */
#include "mbed.h"
#include <std_msgs/String.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "as5600.hpp"
#include "pid.h"
#include "Matrix.h"
#include "esc.hpp"

/* Definitions */
#define killSwitchEnablePin PG_9
#define killSwitchMonitorPin PG_14

#define M_PER_STEP 0.0001288545
#define WHEEL_BASE_LENGTH_M 0.291
#define WHEEL_BASE_LENGTH_OVER_2_M 0.1455
#define WHEEL_RADIUS 0.0839999


/*============================= GLOBAL VARIABLES =============================*/

bool doPIDDebugging = 0;
bool remoteControl = 0;
float target_linear_vel = 0.0f;
float target_turn_vel = 0.0f;


/*============================== ROS CALLBACKS ===============================*/

void cmdVelCB(const geometry_msgs::Twist &twist) {
    target_linear_vel = twist.linear.x;
    target_turn_vel = twist.angular.z;
}

void controllerCB(const geometry_msgs::Twist &twist) {
    target_linear_vel = twist.linear.x;
    target_turn_vel = twist.angular.z;
}

/*============================================================================*/