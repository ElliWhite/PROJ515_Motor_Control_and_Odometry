#include "mbed.h"
#include <std_msgs/String.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
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

#define lMotorEnablePin PB_10
#define rMotorEnablePin PB_11

//2*pi*r / 4096 = 105.5575cm/4096 = 1.055575m/4096
#define M_PER_STEP 0.000257709
#define WHEEL_BASE_LENGTH_M 0.291
#define WHEEL_BASE_LENGTH_OVER_2_M 0.1455

/*==
LEDS
==*/
DigitalOut LED(LED1);
DigitalOut tickerLED(LED2);

/*====
SERIAL
====*/
Serial PC(USBTX, USBRX);

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
AS5600 encoderLeft(I2C_SDA, I2C_SCL);
//AS5600 encoderRight(PA_10, PA_9);

/*=
ROS
=*/
ros::NodeHandle nh;
ros::Time current_time, last_time;
ros::Subscriber<sensor_msgs::Joy> joy_sub("joy", &controllerCB);
ros::Publisher odom_pub("odom", &odom_msg);
tf::TransformBroadcaster odom_broadcaster;
nav_msgs::Odometry odom_msg;

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






int main() {
    
    nh.getHardware()->setBaud(256000);      //set ROSSERIAL baud rate
    nh.initNode();                          //initialise node
    nh.subscribe(joy_sub);                  //subscribe to ROS topic "joy"
    nh.advertise(odom_pub);                 //publish to ROS topic "odom"
    odom_broadcaster.init(nh);              //initialise Transform Broadcaster "oom_broadcaster"


    current_time = nh.now();                //grab current time
    last_time = nh.now();                   //initialise last_time to current time

    int16_t old_Pos_Left = encoderLeft.getAngleAbsolute();          //initialise encoder values
    int16_t old_Pos_Right = 0;//encoderRight.getAngleAbsolute();    
    int16_t new_Pos_Left = encoderLeft.getAngleAbsolute();
    int16_t new_Pos_Right = 0; //encoderRight.getAngleAbsolute();
    int16_t delta_Left_Pos = 0;             //change in encoder value for LEFT encoder
    int16_t delta_Right_Pos = 0;            //change in encoder value for RIGHT encoder

    double old_x = 0.0;                     //old x coordinate of robot
    double old_y = 0.0;                     //old y coordinate of robot
    double old_th = 0.0;                    //old theta (heading) of robot
    double new_x = 0.0;                     //new (current) estimate of x coordinate of robot
    double new_y = 0.0;                     //new (current) esitamte of y coordinate of robot
    double new_th = 0.0;                    //new (current) estimate of theta (heading) of robot
    double angular_rot = 0.0;               //angular rotation velocity
    double ICC_x = 0.0;                     //x coordinate of common rotation point of wheels
    double ICC_y = 0.0;                     //y coordinate of common rotation point of wheels
    double R = 0.0;                         //radius between centre of wheelbase and common rotation point
    double vx = 0.0;                        //linear velocity in forwards direction
    double vth = 0.0;                       //angular rotation velocity of robot

    Matrix mat_delta_Pose(3,1);             //matrix to hold position estimates
    Matrix mat_rotation(3,3);               //rotational matrix
    Matrix mat_delta_ICC(3,1);              //matrix to hold difference between pose and ICC
    Matrix mat_ICC(3,1);                    //matrix to hold position of ICC and angular rotation

    //HERE FOR TESTING ONLY WHEN RIGHT ENCODER ISN'T CONNCETED
    old_Pos_Right = 0;
    //********************************************************
    
   
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
        }else{
            
        }
        //*****************************************************************
        //COMMENTED OUT FOR TESTING ONLY WHEN RIGHT ENCODER ISN'T CONNCETED
        //*****************************************************************
        /*
        if(encoderRight.isMagnetPresent()){
            new_Pos_Right = encoderRight.getAngleAbsolute();
        }else{
            PC.printf("[ERROR] Magnet Not Present - RIGHT\n\r");
        }*/
        
        //HERE FOR TESTING ONLY WHEN RIGHT ENCODER ISN'T CONNCETED
        new_Pos_Right = 0;
        //********************************************************

        delta_Left_Pos = new_Pos_Left - old_Pos_Left;
        delta_Right_Pos = new_Pos_Right - old_Pos_Right;

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
        
                
        if(delta_Left_Pos != 0 || delta_Right_Pos != 0){
            R = (WHEEL_BASE_LENGTH_OVER_2_M * (delta_Left_Pos+delta_Right_Pos)) / (delta_Right_Pos - delta_Left_Pos);
            //PC.printf("R = %f\n\r", R);
        }else{
            R = 0.0;
        }
        
        angular_rot = ((delta_Right_Pos - delta_Left_Pos) * M_PER_STEP) / WHEEL_BASE_LENGTH_M;
        
        ICC_x = old_x - (R*sin(old_th));
        ICC_y = old_y + (R*cos(old_th));

        mat_rotation.Clear();

        mat_rotation    << cos(angular_rot)     << -sin(angular_rot)    << 0
                        << sin(angular_rot)     << cos(angular_rot)     << 0
                        << 0                    << 0                    << 1;
                        
        mat_delta_ICC.Clear();

        mat_delta_ICC   << old_x - ICC_x
                        << old_y - ICC_y
                        << old_th;

        mat_ICC.Clear();

        mat_ICC         << ICC_x
                        << ICC_y
                        << angular_rot;

        mat_delta_Pose = (mat_rotation * mat_delta_ICC) + mat_ICC;

        new_x = mat_delta_Pose.getNumber(1,1);
        new_y = mat_delta_Pose.getNumber(2,1);
        new_th = mat_delta_Pose.getNumber(3,1);


        //PC.printf("%f %f\n\r", new_x, new_y);
        
        /*****************
        COMPUTE VELOCITIES
        *****************/
        double dt = current_time.toSec() - last_time.toSec();
        double v_left = (delta_Left_Pos * M_PER_STEP) / dt;        
        double v_right = (delta_Right_Pos * M_PER_STEP) / dt;
        vx = (v_left + v_right) / 2;
        vth = (v_right - v_left) / WHEEL_BASE_LENGTH_M;
        
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
        old_x = new_x;
        old_y = new_y;
        old_th = new_th;
        last_time = current_time;
        
        wait_ms(50);





    }

}