#include "mbed.h"
#include <std_msgs/String.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include "as5600.h"
#include "Matrix.h"

#define A_BUTTON    0
#define B_BUTTON    1
#define X_BUTTON    2
#define Y_BUTTON    3
#define LEFT_BUMP   4
#define RIGHT_BUMP  5
#define BACK        6
#define START       7
#define CENTRE      8
#define LEFT_STICK  9
#define RIGHT_STICK 10

#define LEFT_STICK_LR_INDEX     0
#define LEFT_STICK_UD_INDEX     1
#define RIGHT_STICK_LR_INDEX    2
#define RIGHT_STICK_UD_INDEX    3
#define RIGHT_TRIGGER_INDEX     4
#define LEFT_TRIGGER_INDEX      5
#define DPAD_LR_INDEX           6
#define DPAD_UP_INDEX           7

#define lMotorControlPin PA_4
#define rMotorControlPin PA_5

#define lMotorDirectionPin PF_13
#define rMotorDirectionPin PE_9

#define lMotorEnablePin PE_14
#define rMotorEnablePin PE_15

//2*pi*r / 4096 = 105.5575cm/4096 = 1.055575m/4096
#define M_PER_STEP 0.000257709
#define WHEEL_BASE_LENGTH_M 0.291
#define WHEEL_BASE_LENGTH_OVER_2_M 0.1455

/*==
LEDS
==*/
DigitalOut LED(LED1);
DigitalOut tickerLED(LED2);
DigitalOut debugLED(LED3);

/*==================
SPEED CONTROL (DACS)
==================*/
AnalogOut Left_Wheel(lMotorControlPin);
AnalogOut Right_Wheel(rMotorControlPin);

/*=============
MOTOR DIRECTION
=============*/
DigitalOut lMotorDirection(lMotorDirectionPin);
DigitalOut rMotorDirection(rMotorDirectionPin);

/*==========
MOTOR ENABLE
==========*/
DigitalOut lMotorEnable(lMotorEnablePin);
DigitalOut rMotorEnable(rMotorEnablePin);

/*=======
ENCODDERS
=======*/
AS5600 encoderLeft(PB_11, PB_10);
AS5600 encoderRight(I2C_SDA, I2C_SCL);



/*==================
CONTROLLER VARIABLES
==================*/
float left_vel = 1.0f;              //left motor velocity, start at max because ESC speed control is difference between Vref and control input.
float right_vel = 1.0f;             //right motor velocity
float right_trig = 0.0f;            //right trigger value
float left_trig = 0.0f;             //left trigger value
float turn = 0.0f;                  //turn value
float turn_tmp = 0.0f;              //temp turn
float right_remapped_trig = 0.0f;   //right remapped trigger
float left_remapped_trig = 0.0f;    //left remapped trigger
float turn_mag = 0.3f;              //magnitude of the turn

float max_speed = 0.0f;             //maximum motor speed

bool a_butt = false;                //enable boolean linked to button A
bool b_butt = false;

//Remap number from one range to another
float Remap(float value, float from1, float to1, float from2, float to2) {
    return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
}

//ROS Callback
void controllerCB(const sensor_msgs::Joy &joy_msg) {
    
    //split up joy_msg and take the right trigger and left stick components.
    right_trig = joy_msg.axes[RIGHT_TRIGGER_INDEX];
    left_trig = joy_msg.axes[LEFT_TRIGGER_INDEX];
    turn = joy_msg.axes[LEFT_STICK_LR_INDEX];
    
    //take button input from joy_msg to enable or disable the motors.
    if(joy_msg.buttons[A_BUTTON] == 1){
        a_butt = true;
    }else if(joy_msg.buttons[B_BUTTON] == 1){
        a_butt = false;
    }
    
}

/*=
ROS
=*/
ros::NodeHandle nh;
ros::Time current_time, last_time;
nav_msgs::Odometry odom_msg;
std_msgs::Int16 debug_left_msg, debug_right_msg;
ros::Subscriber<sensor_msgs::Joy> joy_sub("joy", &controllerCB);
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster odom_broadcaster;
ros::Publisher debug_left_pub("debug_left", &debug_left_msg);
ros::Publisher debug_right_pub("debug_right", &debug_right_msg);





int main() {
    
    nh.getHardware()->setBaud(921600);      //set ROSSERIAL baud rate
    nh.initNode();                          //initialise node
    nh.subscribe(joy_sub);                  //subscribe to ROS topic "joy"
    nh.advertise(odom_pub);                 //publish to ROS topic "odom"
    odom_broadcaster.init(nh);              //initialise Transform Broadcaster "oom_broadcaster"
    nh.advertise(debug_left_pub);
    nh.advertise(debug_right_pub);
    
    lMotorEnable = 0;                   //disable motors
    rMotorEnable = 0;   

    while(!nh.connected()){
        nh.spinOnce();
    }

    current_time = nh.now();                //grab current time
    last_time = nh.now();                   //initialise last_time to current time

    int16_t old_Pos_Left = encoderLeft.getAngleAbsolute();          //initialise encoder values
    int16_t old_Pos_Right = encoderRight.getAngleAbsolute();    
    int16_t new_Pos_Left = encoderLeft.getAngleAbsolute();
    int16_t new_Pos_Right = encoderRight.getAngleAbsolute();
    int16_t delta_Left_Pos = 0;             //change in encoder value for LEFT encoder
    int16_t delta_Right_Pos = 0;            //change in encoder value for RIGHT encoder

    double new_x = 0.0;                     //new (current) estimate of x coordinate of robot
    double new_y = 0.0;                     //new (current) esitamte of y coordinate of robot
    double new_th = 0.0;                    //new (current) estimate of theta (heading) of robot

    double vx = 0.0;                        //linear velocity in forwards direction
    double vth = 0.0;                       //angular rotation velocity of robot


    
   
    while(1){

        tickerLED = !tickerLED;

        current_time = nh.now();        //grab current time

        nh.spinOnce();                  //check for incoming messages

        //remap right trigger input
        right_remapped_trig = Remap(right_trig, -1.0f, 1.0f, max_speed, 1.0f);
        left_remapped_trig = Remap(left_trig, -1.0f, 1.0f, max_speed, 1.0f);
        turn_tmp = turn;
        
        //if the a button is pressed, motors are enabled. 
        if(a_butt == true){
            if((left_remapped_trig > 0.9f) && (right_remapped_trig < 0.9f)){
                //forwards
                lMotorDirection = 1;
                rMotorDirection = 0;
                left_vel = (right_remapped_trig + (turn * turn_mag));           //set left wheel velocity to remapped trigger value
                right_vel = 1 - (right_remapped_trig - (turn * turn_mag));      //set right wheel velocity to inverted remapped trigger value
            }else if((left_remapped_trig < 0.9f) && (right_remapped_trig > 0.9f)){
                //backwards
                lMotorDirection = 0;
                rMotorDirection = 1;
                left_vel = left_remapped_trig + (turn * turn_mag);           //set left wheel velocity to remapped trigger value
                right_vel = 1 - (left_remapped_trig - (turn * turn_mag));      //set right wheel velocity to inverted remapped trigger value
            }else{
                left_vel = 1.0f;
                right_vel = 0.0f;   
            }
            lMotorEnable = 1;                   //enable left motor
            rMotorEnable = 1;                   //enable right motor
            Left_Wheel.write(left_vel);         //set left control pin to left wheel velocity
            Right_Wheel.write(right_vel);       //set right control pin to right wheel velocity
            LED = 1;                            //turn onboard LED on to show motors enabled.
        }else{
            lMotorEnable = 0;                   //disable motors
            rMotorEnable = 0;                   
            LED = 0;                            //turn onboard LED off
        }


        if(encoderLeft.isMagnetPresent()){
            new_Pos_Left = encoderLeft.getAngleAbsolute();
        }
            
        
        if(encoderRight.isMagnetPresent()){
            new_Pos_Right = encoderRight.getAngleAbsolute();
        } 
      

        delta_Left_Pos = new_Pos_Left - old_Pos_Left;
        delta_Right_Pos = (new_Pos_Right - old_Pos_Right) * -1;
        
        //remove noise
        if(delta_Left_Pos == 1 || delta_Left_Pos == -1){
            delta_Left_Pos = 0;
        }
        if(delta_Right_Pos == 1 || delta_Right_Pos == -1){
            delta_Right_Pos = 0;
        }
        
        //if either wheel positions change, toggle the debug (red) LED
        if(delta_Left_Pos != 0 || delta_Right_Pos != 0){
            debugLED = !debugLED;
        }
        
        
        //calculations to deal with rollover of absolute encoder
        //returns true delta_Pos when rollover does occur
        if(delta_Left_Pos > 2047){
            delta_Left_Pos = delta_Left_Pos - 4096;
        }else if(delta_Left_Pos < -2047){
            delta_Left_Pos =  delta_Left_Pos + 4096;
        }
        if(delta_Right_Pos > 2047){
            delta_Right_Pos = delta_Right_Pos - 4096;
        }else if(delta_Right_Pos < -2047){
            delta_Right_Pos =  delta_Right_Pos + 4096;
        }
        
        debug_left_msg.data = delta_Left_Pos;
        debug_left_pub.publish(&debug_left_msg);
        debug_right_msg.data = delta_Right_Pos;
        debug_right_pub.publish(&debug_right_msg);   
        
        
        /*****************
        COMPUTE VELOCITIES
        *****************/
        double dt = current_time.toSec() - last_time.toSec();
        double v_left = (delta_Left_Pos * M_PER_STEP) / dt;        
        double v_right = (delta_Right_Pos * M_PER_STEP) / dt;
        vx = (v_left + v_right) / 2;
        vth = (v_right - v_left) / WHEEL_BASE_LENGTH_M;
        
        //compute odometry in a typical way given the velocities of the robot
        double delta_x = (vx * cos(new_th)) * dt;
        double delta_y = (vx * sin(new_th)) * dt;
        double delta_th = vth * dt;
        
        new_x += delta_x;
        new_y += delta_y;
        new_th += delta_th;
        
        
        
        /****************************
        SET TRANSFORM AND ODOM PARAMS
        ****************************/
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(new_th);
        
        //first publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "/odom";
        odom_trans.child_frame_id = "/base_link";

        odom_trans.transform.translation.x = new_x;
        odom_trans.transform.translation.y = new_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //now publish the odometry message over ROS
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "/odom";

        //set the position
        odom_msg.pose.pose.position.x = new_x;
        odom_msg.pose.pose.position.y = new_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        //set the velocity
        odom_msg.child_frame_id = "/base_link";
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0;        //will always be 0 as assuming there will be no "slippage"
        odom_msg.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(&odom_msg);     
                   
        
        old_Pos_Left = new_Pos_Left;
        old_Pos_Right = new_Pos_Right;
        last_time = current_time;
        
        wait_ms(1);





    }

}