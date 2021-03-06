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
#include "std_msgs/Char.h"
#include "as5600.hpp"
#include "pid.h"
#include "Matrix.h"
#include "esc.hpp"

/* Definitions */
#define M_PER_STEP                  0.0001288545
#define WHEEL_BASE_LENGTH_M         0.2957
#define WHEEL_BASE_LENGTH_OVER_2_M  0.14785
#define WHEEL_RADIUS                0.0839999

#define RELAY_CONNECT_AC        1
#define RELAY_CONNECT_SHORT     0
#define RELAY_DISCONNECT_SHORT  1



/*============================= GLOBAL VARIABLES =============================*/

bool doPIDDebugging = 1;            // Boolean to control whether PID debugging topics are published
bool remoteControl = 1;             // Boolean to control whether subscribed to /mtr_ctrl/cmd_vel for..
                                    // ..Xbox remote control.
bool useEKF = 0;                    // Boolean to control whether using the EKF package
                                    // This controls whether a transform from odom -> base_link..
                                    // ..is being published and changes the odom topic to /wheel_odom
bool testOdom = 0;                  // Boolean to control whether extra odom messages are published with the..
                                    // ..different localisation methods
bool useSensorFusion = 1;           // Boolean to decide whether using sensor fusion.
bool testSensorFusion = 1;          // Boolean to decide whether testing sensor fusion.

bool enableBrakes = 1;              // Boolean to decide whether brakes are being used.

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
