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
#include "as5600.h"
#include "pid.h"
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

#define lMotorSpeedControlPin PA_4
#define rMotorSpeedControlPin PA_5

#define lMotorDirectionPin PF_13
#define rMotorDirectionPin PE_9

#define lMotorEnablePin PE_14
#define rMotorEnablePin PE_15

#define killSwitchEnablePin PG_9
#define killSwitchMonitorPin PG_14

//2*pi*r / 4096 = 52.77875cm/4096 = 0.5277875m/4096
#define M_PER_STEP 0.0001288545
#define WHEEL_BASE_LENGTH_M 0.291
#define WHEEL_BASE_LENGTH_OVER_2_M 0.1455
#define WHEEL_RADIUS 0.0839999

/*==
LEDS
==*/
DigitalOut motorEnableLED(LED1);
DigitalOut tickerLED(LED2);
DigitalOut debugLED(LED3);

/*==================
SPEED CONTROL (DACS)
==================*/
AnalogOut Left_Motor_Speed(lMotorSpeedControlPin);
AnalogOut Right_Motor_Speed(rMotorSpeedControlPin);

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

/*=
PID
=*/
PID leftMotorPID(5000, 7, 200, 0, 1, 0.5);       //Kp, Ki, Kd, min, max, integral_limit
PID rightMotorPID(1500, 5.2, 200, 0, 1, 0.5);
//PID leftMotorPID(0.4, 9, 0.2, 0, 1, 0.1);       //Kp, Ki, Kd, min, max, integral_limit
//PID rightMotorPID(0, 0, 0, 0, 1, 0.5);
bool doPIDDebugging = 0;

/*=========
KILL SWITCH
=========*/
DigitalOut killSwitchEnable(killSwitchEnablePin);
DigitalIn  killSwitchMonitor(killSwitchMonitorPin);






float target_linear_vel = 0.0f;
float target_turn_vel = 0.0f;



void cmdVelCB(const geometry_msgs::Twist &twist) {
    target_linear_vel = twist.linear.x;
    target_turn_vel = twist.angular.z;
}

void controllerCB(const geometry_msgs::Twist &twist) {
    target_linear_vel = twist.linear.x;
    target_turn_vel = twist.angular.z;
}

/*=
ROS
=*/
ros::NodeHandle     nh;
ros::Time           current_time, last_time;

nav_msgs::Odometry  odom_msg;
std_msgs::Float64   vel_left_msg;
std_msgs::Float64   vel_right_msg;
std_msgs::Float64   motor_time_msg;
std_msgs::Float64   left_motor_val_msg;
std_msgs::Float64   right_motor_val_msg;
std_msgs::Float64   target_linear_vel_msg;



ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &cmdVelCB);
ros::Subscriber<geometry_msgs::Twist> controller_sub("mtr_ctrl/cmd_vel", &controllerCB);
bool remoteControl = 0;

ros::Publisher              odom_pub("odom", &odom_msg);
tf::TransformBroadcaster    odom_broadcaster;
ros::Publisher              vel_left_pub("left_vel", &vel_left_msg);
ros::Publisher              vel_right_pub("right_vel", &vel_right_msg);
ros::Publisher              motor_time_pub("motor_time", &motor_time_msg);
ros::Publisher              left_motor_val_pub("left_motor_val", &left_motor_val_msg);
ros::Publisher              right_motor_val_pub("right_motor_val", &right_motor_val_msg);
ros::Publisher              target_linear_vel_pub("target_vel", &target_linear_vel_msg);


 


int main() {
    
    nh.getHardware()->setBaud(921600);      //set ROSSERIAL baud rate
    nh.initNode();                          //initialise node
    nh.subscribe(twist_sub);
    nh.advertise(odom_pub);                 //publish to ROS topic "odom"
    odom_broadcaster.init(nh);              //initialise Transform Broadcaster "oom_broadcaster"
    nh.advertise(vel_left_pub);
    nh.advertise(vel_right_pub);
    
    if(doPIDDebugging){
        nh.advertise(motor_time_pub);
        nh.advertise(left_motor_val_pub);
        nh.advertise(right_motor_val_pub);
        nh.advertise(target_linear_vel_pub);
    }
    
    if(remoteControl){
        nh.subscribe(controller_sub);
    }
        
    
    killSwitchEnable = 1;
    killSwitchMonitor.mode(PullDown);

    
    lMotorEnable = 0;                   //disable motors
    rMotorEnable = 0; 
    motorEnableLED = 0;  

    //wait for ROS to connect to board
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
    int16_t previous_delta_Left_Pos = 0;    //holds the previous change in encoder value for LEFT encoder
    int16_t previous_delta_Right_Pos = 0;   //holds the previous change in encoder value for RIGHT encoder

    double new_x = 0.0;                     //new (current) estimate of x coordinate of robot
    double new_y = 0.0;                     //new (current) esitamte of y coordinate of robot
    double new_th = 0.0;                    //new (current) estimate of theta (heading) of robot

    double vx = 0.0;                        //linear velocity in forwards direction
    double vth = 0.0;                       //angular rotation velocity of robot
    
    Matrix mat_target_vels(2,1);            //matrix to hold target velocities, linear and angular
    Matrix mat_wheel_vels(2,1);             //matrix to hold angular velocities of the wheels
    Matrix mat_conversion(2,2);             //matrix to hold the conversion matrix to convert target velocities
                                            //to wheel angular velocities
                                            
    mat_conversion << 1/WHEEL_RADIUS    << WHEEL_BASE_LENGTH_M/(2*WHEEL_RADIUS)
                   << 1/WHEEL_RADIUS    << -WHEEL_BASE_LENGTH_M/(2*WHEEL_RADIUS);

    //if debugging PID we need to publish dummy velocity data for the wheels.
    //this is initially zero as the wheels aren't moving. This allows the pid_plotting.py
    //script to plot some initial values on the graph so we can more clearly see
    //how the PID controllers are performing
    if(doPIDDebugging){
        for(int i = 0; i < 50; i++){
            vel_left_msg.data = float(0);
            vel_right_msg.data = float(0);
            motor_time_msg.data = float(0.016);
            target_linear_vel_msg.data = float(0);
            vel_left_pub.publish(&vel_left_msg);
            vel_right_pub.publish(&vel_right_msg);
            motor_time_pub.publish(&motor_time_msg);
            target_linear_vel_pub.publish(&target_linear_vel_msg);
            wait_ms(30);
        }
    }
    
   
    while(nh.connected() && killSwitchMonitor == 1){

        tickerLED = !tickerLED;

        nh.spinOnce();                  //check for incoming messages
        
        current_time = nh.now();        //grab current time

        if(target_linear_vel > 0.0f){
            //forwards
            lMotorDirection = 1;
            rMotorDirection = 0;
            lMotorEnable = 1;                   //enable motors
            rMotorEnable = 1;
            motorEnableLED = 1;     
        }else if(target_linear_vel < 0.0f){
            //backwards
            lMotorDirection = 0;
            rMotorDirection = 1;
            lMotorEnable = 1;                   //enable motors
            rMotorEnable = 1; 
            motorEnableLED = 1;  
        }else if(target_linear_vel == 0.0f && target_turn_vel > 0.0f){
            //spin on the spot left
            lMotorDirection = 0;
            rMotorDirection = 0;
            lMotorEnable = 1;                   //enable motors
            rMotorEnable = 1; 
            motorEnableLED = 1;  
        }else if(target_linear_vel == 0.0f && target_turn_vel < 0.0f){
            //spin on the spot right
            lMotorDirection = 1;
            rMotorDirection = 1;
            lMotorEnable = 1;                   //enable motors
            rMotorEnable = 1; 
            motorEnableLED = 1;  
        }else{
            lMotorEnable = 0;                   //disable motors
            rMotorEnable = 0;
            motorEnableLED = 0;  
        } 
            

        if(encoderLeft.isMagnetPresent()){
            new_Pos_Left = encoderLeft.getAngleAbsolute();
        }   
        
        if(encoderRight.isMagnetPresent()){
            new_Pos_Right = encoderRight.getAngleAbsolute();
        }

        //find change in encoder position for both wheels
        delta_Left_Pos = new_Pos_Left - old_Pos_Left;
        delta_Right_Pos = (new_Pos_Right - old_Pos_Right) * -1;
        
        //remove noise
        if(delta_Left_Pos == 1 || delta_Left_Pos == -1){
            delta_Left_Pos = 0;
        }
        if(delta_Right_Pos == 1 || delta_Right_Pos == -1){
            delta_Right_Pos = 0;
        }

        //sometimes the encoder would read a value which results in the algorithm thinking
        //the motor has suddenly changed direction and is spinning fast. This is impossible
        //so to mitigate this error we keep track of the previous change in position
        //and use this to see if the change is too large for it to be possible
        if(abs(delta_Left_Pos - previous_delta_Left_Pos) > 100){
            delta_Left_Pos = previous_delta_Left_Pos;
        }else{
            previous_delta_Left_Pos = delta_Left_Pos;
        }
        if(abs(delta_Right_Pos - previous_delta_Right_Pos) > 100){
            delta_Right_Pos = previous_delta_Right_Pos;
        }else{
            previous_delta_Right_Pos = delta_Right_Pos;
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


       //disable motors if comms with encoders fails. We expect dt to stay below
        //15ms and when it loses communication dt increases, so can use this to 
        //control motors.
        if(dt > 0.018){
            lMotorEnable = 0;
            rMotorEnable = 0;
            motorEnableLED = 0;
        } else {
            lMotorEnable = 1;
            rMotorEnable = 1;
            motorEnableLED = 1;
        }


        /*******************************************************************
        CONVERTING OVERALL LINEAR AND ANGULAR TO INDIVIDUAL WHEEL VELOCITIES
        *******************************************************************/
        mat_target_vels.Clear();
        mat_target_vels << target_linear_vel    << target_turn_vel;

        mat_wheel_vels.Clear();
        mat_wheel_vels = (mat_conversion * mat_target_vels);

        double right_target_vel = WHEEL_RADIUS * double(mat_wheel_vels.getNumber(1,1));
        double left_target_vel = WHEEL_RADIUS * double(mat_wheel_vels.getNumber(2,1));

        
        /**************
        PID CONTROLLERS
        **************/
        //output of PID loop will be the value to write to the motors
        double new_v_left = leftMotorPID.calculate(abs(v_left), abs(left_target_vel), dt);
        double new_v_right = rightMotorPID.calculate(abs(v_right), abs(right_target_vel), dt);
        Left_Motor_Speed.write(1-new_v_left);
        Right_Motor_Speed.write(new_v_right);
        

        if(doPIDDebugging){
            left_motor_val_msg.data = float(new_v_left);
            right_motor_val_msg.data = float(new_v_right);
            left_motor_val_pub.publish(&left_motor_val_msg);
            right_motor_val_pub.publish(&right_motor_val_msg);
            
            target_linear_vel_msg.data = target_linear_vel;
            target_linear_vel_pub.publish(&target_linear_vel_msg);
            
            motor_time_msg.data = float(dt);
            motor_time_pub.publish(&motor_time_msg);
            

        }
        
                
        vel_left_msg.data = float(v_left);
        vel_right_msg.data = float(v_right);
        vel_left_pub.publish(&vel_left_msg);
        vel_right_pub.publish(&vel_right_msg);
        
        
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

        //now set up the odometry message
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

        //publish the message over ROS
        odom_pub.publish(&odom_msg);     
                   
        
        old_Pos_Left = new_Pos_Left;
        old_Pos_Right = new_Pos_Right;
        last_time = current_time;
        
        wait_ms(1);

    }
    
    //disable motors if lose connection to ROS master
    lMotorEnable = 0;
    rMotorEnable = 0;
    motorEnableLED = 0;
    tickerLED = 0;
    debugLED = 0;
    
    

}