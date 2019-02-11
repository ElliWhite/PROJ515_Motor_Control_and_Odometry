#include "mbed.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#define lMotorControlPin PA_4
#define rMotorControlPin PA_5

#define lMotorDirectionPin PF_13
#define rMotorDirectionPin PE_9

#define lMotorEnablePin PE_14
#define rMotorEnablePin PE_15

/*==================
Speed Control (DACS)
==================*/
AnalogOut Left_Wheel(lMotorControlPin);
AnalogOut Right_Wheel(rMotorControlPin);

/*=============
Motor Direction
=============*/
DigitalOut lMotorDirection(lMotorDirectionPin);
DigitalOut rMotorDirection(rMotorDirectionPin);

/*==========
Motor Enable
==========*/
DigitalOut lMotorEnable(lMotorEnablePin);
DigitalOut rMotorEnable(rMotorEnablePin);

DigitalOut led(LED1);
DigitalOut led1(LED2);

float left_vel = 0.0f;
float right_vel = 0.0f;
float target_vel = 0.0f;
float turn_vel = 0.0f;

//ROS Callback
void controllerCB(const geometry_msgs::Twist &twist) {
    target_vel = twist.linear.x;
    turn_vel = twist.angular.z;
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("mtr_ctrl/cmd_vel", &controllerCB);

int main() {
        
    //initialise ros node and subscribe to "joy" topic
    
    nh.initNode();
    nh.subscribe(sub);
    
    while(!nh.connected()){
        nh.spinOnce();
    }
    
    while(1){
        nh.spinOnce();
        
        if(target_vel > 0.1f){
            led = 1;
            led1 = 0;
            //forwards
            lMotorDirection = 1;
            rMotorDirection = 0;
            left_vel = 1 - target_vel;           //set left wheel velocity to remapped trigger value
            right_vel = target_vel;      //set right wheel velocity to inverted remapped trigger value
            lMotorEnable = 1;                   //enable left motor
            rMotorEnable = 1;                   //enable right motor
        }else if (target_vel < -0.1f){
            lMotorDirection = 0;
            rMotorDirection = 1;
            left_vel = 1 - (-1 * target_vel) ;           //set left wheel velocity to remapped trigger value
            right_vel = -1 * target_vel;      //set right wheel velocity to inverted remapped trigger value
            lMotorEnable = 1;                   //enable left motor
            rMotorEnable = 1;                   //enable right motor
        }else{
            lMotorEnable = 0;                   //enable left motor
            rMotorEnable = 0;                   //enable right motor   
        }
        
        
        Left_Wheel.write(left_vel);         //set left control pin to left wheel velocity
        Right_Wheel.write(right_vel);       //set right control pin to right wheel velocity
        
        wait_ms(1);
    }

}

