/*============================================================================*/

/* Libraries */
#include "main.hpp"

/*=============================== DECLARATIONS ===============================*/

/* LEDS */
DigitalOut motorEnableLED(LED1);    // LED to visualise when the motors are enabled
DigitalOut tickerLED(LED2);         // LED to toggle every time the main control loop completes
DigitalOut debugLED(LED3);          // LED for debugging purposes

/* ESCS */
ESC leftESC(lMotorSpeedControlPin, lMotorDirectionPin, lMotorEnablePin);
ESC rightESC(rMotorSpeedControlPin, rMotorDirectionPin, rMotorEnablePin);

/* ENCODERS */
AS5600 encoderRight(PB_11, PB_10);
AS5600 encoderLeft(I2C_SDA, I2C_SCL);

/* PID CONTROLLERS */
PID leftMotorPID(0.01, 1.5, 0.0145, 0, 1, 0.5); //Kp, Ki, Kd, min, max, integral_limit
PID rightMotorPID(0.01, 1.3, 0.015, 0, 1, 0.5);

/* BRAKING RELAYS */
DigitalOut  leftBrakeRelay_1(PE_0);     // Blue
DigitalOut  leftBrakeRelay_2(PE_2);     // Purple
DigitalOut  leftBrakeRelay_3(PA_0);     // Green
PwmOut      leftBrakeRelay_4(PD_12);    // Yellow
PwmOut      leftBrakeRelay_5(PD_13);    // Orange
DigitalOut  rightBrakeRelay_1(PD_11);   // Blue
DigitalOut  rightBrakeRelay_2(PB_2);    // Purple
DigitalOut  rightBrakeRelay_3(PF_4);    // Green
PwmOut      rightBrakeRelay_4(PB_4);    // Yellow
PwmOut      rightBrakeRelay_5(PB_5);    // Orange

/* ROS */
ros::NodeHandle             nh;
ros::Time                   current_time, last_time;
nav_msgs::Odometry          odom_eulerian_msg;
nav_msgs::Odometry          odom_runge_kutta_msg;
nav_msgs::Odometry          odom_exact_msg;
std_msgs::Float64           vel_left_msg;
std_msgs::Float64           vel_right_msg;
std_msgs::Float64           motor_time_msg;
std_msgs::Float64           left_motor_val_msg;
std_msgs::Float64           right_motor_val_msg;
std_msgs::Float64           target_linear_vel_msg;
ros::Publisher              vel_left_pub("left_vel", &vel_left_msg);
ros::Publisher              vel_right_pub("right_vel", &vel_right_msg);
ros::Publisher              odom_pub("odom", &odom_exact_msg);
ros::Publisher              wheel_odom_eulerian_pub("wheel_odom/eulerian_odom", &odom_eulerian_msg);
ros::Publisher              wheel_odom_runge_kutta_pub("wheel_odom/runge_kutta_odom", &odom_runge_kutta_msg);
ros::Publisher              wheel_odom_exact_pub("wheel_odom/exact_odom", &odom_exact_msg);
ros::Publisher              motor_time_pub("motor_time", &motor_time_msg);
ros::Publisher              left_motor_val_pub("left_motor_val", &left_motor_val_msg);
ros::Publisher              right_motor_val_pub("right_motor_val", &right_motor_val_msg);
ros::Publisher              target_linear_vel_pub("target_vel", &target_linear_vel_msg);
tf::TransformBroadcaster    odom_broadcaster;
ros::Subscriber<geometry_msgs::Twist>   twist_sub("cmd_vel", &cmdVelCB);
ros::Subscriber<geometry_msgs::Twist>   controller_sub("mtr_ctrl/cmd_vel", &controllerCB);

Timer global_test_timer;


/*=================================== MAIN ===================================*/

int main() {

    nh.getHardware()->setBaud(921600);      // Set ROSSERIAL baud rate
    nh.initNode();                          // Initialise ROS node
    nh.subscribe(twist_sub);                // Subscribe to ROS topic "cmd_vel" topic for velocity control

    // If not using Robot Localisation EKF
    if (!useEKF) {
        odom_broadcaster.init(nh);              // Initialise Transform Broadcaster "odom_broadcaster"
        nh.advertise(odom_pub);                 // Publish to ROS topic "odom" for odometry from the wheels
    } else {
        nh.advertise(wheel_odom_exact_pub);     // Publish to ROS topic "wheel_odom" for odometry from the wheels
    }                                           // This is different as the output of the Robot Localisation EKF ..
                                                // .. uses the topic "odom"

    // If testing different odometry localisation algorithms
    if (testOdom) {
        nh.advertise(wheel_odom_runge_kutta_pub);
        nh.advertise(wheel_odom_eulerian_pub);
    }

    // If PID Debugging is desired
    if (doPIDDebugging) {
        //nh.advertise(motor_time_pub);           // Publish to ROS topic "motor_time" for time steps between each control loop
        nh.advertise(left_motor_val_pub);       // Publish to ROS topic "left_motor_val" for value written to left motor analog pin
        nh.advertise(right_motor_val_pub);      // Publish to ROS topic "right_motor_val" for value written to right motor analog pin
        nh.advertise(target_linear_vel_pub);    // Publish to ROS topic "target_vel" for target linear velocity
        nh.advertise(vel_left_pub);             // Publish to ROS topic "left_vel" for velocity of left motor
        nh.advertise(vel_right_pub);            // Publish to ROS topic "right_vel" for velocity of right motor
    }

    // If remote control operation is desired using Xbox controller
    if (remoteControl) {
        nh.subscribe(controller_sub);           // Subscribe to ROS topic "mtr_ctrl/cmd_vel" for velocity control
    }
    /************
    MOTOR TESTING
    ************/
    /*
     // Set relays to having brakes on initially
    // (setting pin low turns then 'on')
    //wait_ms(500);
    leftBrakeRelay_1 = RELAY_CONNECT_AC;
    //wait_ms(500);
    leftBrakeRelay_2 = RELAY_CONNECT_AC;
    //wait_ms(500);
    leftBrakeRelay_3 = RELAY_CONNECT_AC;
    //wait_ms(500);
    wait_ms(12);
    leftBrakeRelay_4.write(1.0f);
    leftBrakeRelay_4.period(0.1f);
    //wait_ms(500);
    leftBrakeRelay_5.write(1.0f);
    leftBrakeRelay_5.period(0.1f);
    //wait_ms(500);
    rightBrakeRelay_1 = RELAY_CONNECT_AC;
    //wait_ms(500);
    rightBrakeRelay_2 = RELAY_CONNECT_AC;
    //wait_ms(500);
    rightBrakeRelay_3 = RELAY_CONNECT_AC;
    //wait_ms(500);
    wait_ms(12);
    rightBrakeRelay_4.write(1.0f);
    rightBrakeRelay_4.period(0.1f);
    //wait_ms(500);
    rightBrakeRelay_5.write(1.0f);
    rightBrakeRelay_5.period(0.1f);
    //wait_ms(500);

    rightESC.setEnable(DISABLE);
    leftESC.setEnable(ENABLE);
    leftESC.setDirection(LEFT_MOTOR_FORWARDS);
    leftESC.setSpeed(1.0f);
    wait_ms(3000);
    leftESC.setDirection(LEFT_MOTOR_BACKWARDS);
    leftESC.setSpeed(0.5f);
    wait_ms(5000);
    leftESC.setEnable(DISABLE);

    wait_ms(2000);

    rightESC.setEnable(ENABLE);
    rightESC.setDirection(RIGHT_MOTOR_FORWARDS);
    rightESC.setSpeed(0.0f);
    wait_ms(3000);
    rightESC.setDirection(RIGHT_MOTOR_BACKWARDS);
    rightESC.setSpeed(0.5f);
    wait_ms(5000);
    rightESC.setEnable(DISABLE);

    wait_ms(2000);
    */



    // Set relays to having brakes on initially
    // (setting pin low turns then 'on')
    //wait_ms(500);
    leftBrakeRelay_1 = RELAY_CONNECT_SHORT;
    //wait_ms(500);
    leftBrakeRelay_2 = RELAY_CONNECT_SHORT;
    //wait_ms(500);
    leftBrakeRelay_3 = RELAY_CONNECT_SHORT;
    //wait_ms(500);
    wait_ms(12);
    leftBrakeRelay_4.write(0.0f);
    leftBrakeRelay_4.period(0.2f);
    //wait_ms(500);
    leftBrakeRelay_5.write(0.0f);
    leftBrakeRelay_5.period(0.2f);
    //wait_ms(500);
    rightBrakeRelay_1 = RELAY_CONNECT_SHORT;
    //wait_ms(500);
    rightBrakeRelay_2 = RELAY_CONNECT_SHORT;
    //wait_ms(500);
    rightBrakeRelay_3 = RELAY_CONNECT_SHORT;
    //wait_ms(500);
    wait_ms(12);
    rightBrakeRelay_4.write(0.0f);
    rightBrakeRelay_4.period(0.3f);
    //wait_ms(500);
    rightBrakeRelay_5.write(0.0f);
    rightBrakeRelay_5.period(0.3f);
    //wait_ms(500);
    


    // Disable motors
    leftESC.setEnable(DISABLE);
    rightESC.setEnable(DISABLE);
    leftESC.setSpeed(0);     // Set speed of motors to nothing (opposite for right motor)
    rightESC.setSpeed(1);
    leftESC.setDirection(LEFT_MOTOR_FORWARDS);
    rightESC.setDirection(RIGHT_MOTOR_FORWARDS);
    motorEnableLED = 0;                 // Turn off motor-enabled LED

    // Wait for ROS serial node to connect to board
    while (!nh.connected()) {
        nh.spinOnce();
    }

    current_time = nh.now();                // Grab current time
    last_time = nh.now();                   // Initialise last_time to current time

    Timer motor_stationary_timer;           // Timer acting as a timeout for motors if cmd_vel is still sending velocities
                                            // but the robot is not moving

    //global_test_timer.start();

    int16_t old_Pos_Left = encoderLeft.getAngleAbsolute();          // Initialise encoder values
    int16_t old_Pos_Right = encoderRight.getAngleAbsolute();
    int16_t new_Pos_Left = encoderLeft.getAngleAbsolute();
    int16_t new_Pos_Right = encoderRight.getAngleAbsolute();
    int16_t delta_Left_Pos = 0;             // Change in encoder value for LEFT encoder
    int16_t delta_Right_Pos = 0;            // Change in encoder value for RIGHT encoder
    int16_t previous_delta_Left_Pos = 0;    // Holds the previous change in encoder value for LEFT encoder
    int16_t previous_delta_Right_Pos = 0;   // Holds the previous change in encoder value for RIGHT encoder

    double new_x = 0.0;                     // New (current) estimate of x coordinate of robot
    double new_y = 0.0;                     // New (current) esitamte of y coordinate of robot
    double theta = 0.0;                     // Current estimate of theta (heading) of robot
    double new_theta = 0.0;                 // New estimate of theta (heading) of robot

    double new_x_runge_kutta = 0.0;         // New estimate of x coordinate of robot using Second Order Runge-Kutta estimations
    double new_y_runge_kutta = 0.0;         // New estimate of y coordinate of robot using Second Order Runge-Kutta estimations

    double new_x_exact = 0.0;               // New estimate of x coordinate of robot using Exact Integration
    double new_y_exact = 0.0;               // New estimate of y coordinate of robot using Exact Integration

    double vrobot = 0.0;                    // Linear velocity of robot (m/s)
    double vth = 0.0;                       // Angular rotational velocity of robot (rad/s)
    
    double old_v_right = 0.0;

    Matrix mat_target_vels(2, 1);           // Matrix to hold target velocities, linear and angular
    Matrix mat_target_wheel_vels(2, 1);     // Matrix to hold target angular velocities of the wheels
    Matrix mat_convert_target_vels(2, 2);   // Matrix to hold the conversion matrix to convert target velocities
                                            // to wheel angular velocities

    Matrix mat_wheel_vels(2, 1);            // Matrix to hold actual angular velocities of the wheels
    Matrix mat_robot_vels(2, 1);            // Matrix to hold linear and angular velocities of robot
    Matrix mat_convert_actual_vels(2, 2);   // Matrix to hold the conversion matrix to convert actual wheel angular velocities
                                            // to linear and angular velocities of robot

    // Fill "conversion from target velocities to wheel angular velocities" matrix
    mat_convert_target_vels << 1 / WHEEL_RADIUS    << WHEEL_BASE_LENGTH_M / (2 * WHEEL_RADIUS)
                            << 1 / WHEEL_RADIUS    << -WHEEL_BASE_LENGTH_M / (2 * WHEEL_RADIUS);

    // Fill "conversion from wheel angular velocities to robot velocities" matrix
    mat_convert_actual_vels << WHEEL_RADIUS / 2                       << WHEEL_RADIUS / 2
                            << WHEEL_RADIUS / WHEEL_BASE_LENGTH_M     << -WHEEL_RADIUS / WHEEL_BASE_LENGTH_M;


    // Fill odom_msg's covariance matrix
    for (int i = 0; i < 36; i++) {
        odom_exact_msg.pose.covariance[i] = 0;
    }

    // If debugging PID we need to publish dummy velocity data for the wheels.
    // This is initially zero as the wheels aren't moving. This allows the pid_plotting.py
    // script to plot some initial values on the graph so we can more clearly see how the PID controllers
    // are performing
    /*
    if(doPIDDebugging){
        for(int i = 0; i < 50; i++){
            vel_left_msg.data = float(0);               // Assign zero to velocities of left and right wheels
            vel_right_msg.data = float(0);
            motor_time_msg.data = float(0.016);         // Assign 16ms to the time for control loop to complete
            target_linear_vel_msg.data = float(0);      // Assign zero as the target velocity
            vel_left_pub.publish(&vel_left_msg);        // Publish velocity of left wheel
            vel_right_pub.publish(&vel_right_msg);      // Publish velocity of right wheel
            motor_time_pub.publish(&motor_time_msg);    // Publish motor time
            target_linear_vel_pub.publish(&target_linear_vel_msg);  // Publish target velocity
            wait_ms(30);
        }
    }*/

    while (true) {
        if (!nh.connected()) {
            nh.spinOnce();
        } else {

            // Main control loop. Only runs if the ROS serial node is connected
            while (nh.connected()) {

                tickerLED = !tickerLED;         // Toggle ticker LED

                nh.spinOnce();                  // Check for incoming messages

                current_time = nh.now();        // Grab current time

                /*
                if(global_test_timer.read_ms() > 4000 && global_test_timer.read_ms() < 4100){
                    target_linear_vel = 0.2f;
                }
                if(global_test_timer.read_ms() > 8000 && global_test_timer.read_ms() < 8100){
                    target_linear_vel = 0.7f;
                }
                if(global_test_timer.read_ms() > 12000){
                    target_linear_vel = 0.0f;
                    global_test_timer.reset();
                }*/


                if (encoderLeft.isMagnetPresent()) {                    // If a magnet is present by left encoder
                    new_Pos_Left = encoderLeft.getAngleAbsolute();      // grab the current absolute position of it
                }

                if (encoderRight.isMagnetPresent()) {                   // If a magnet is present by right encoder
                    new_Pos_Right = encoderRight.getAngleAbsolute();    // grab the current absolute position of it
                }

                // Find change in encoder position for both wheels between each control loop
                delta_Left_Pos = new_Pos_Left - old_Pos_Left;
                delta_Right_Pos = (new_Pos_Right - old_Pos_Right) * -1;     // Keep sign of delta relative to rotation. I.e forwards is positive

                // Remove small noise when robot is not moving
                if (delta_Left_Pos == 1 || delta_Left_Pos == -1) {
                    delta_Left_Pos = 0;
                }
                if (delta_Right_Pos == 1 || delta_Right_Pos == -1) {
                    delta_Right_Pos = 0;
                }

                // Sometimes the encoder would read a value which results in the algorithm thinking
                // the motor has suddenly changed direction and is spinning at a high velocity.
                // This is impossible so to mitigate this error we keep track of the previous change in position
                // and use this to see if the change in encoder value is too large for it to be possible
                if (abs(delta_Left_Pos - previous_delta_Left_Pos) > 100) {
                    delta_Left_Pos = previous_delta_Left_Pos;
                } else {
                    previous_delta_Left_Pos = delta_Left_Pos;
                }
                if (abs(delta_Right_Pos - previous_delta_Right_Pos) > 100) {
                    delta_Right_Pos = previous_delta_Right_Pos;
                } else {
                    previous_delta_Right_Pos = delta_Right_Pos;
                }

                // If either wheel positions change, toggle the debug (red) LED
                if (delta_Left_Pos != 0 || delta_Right_Pos != 0) {
                    debugLED = !debugLED;
                }

                // Calculations to deal with rollover of absolute encoder
                // Returns true delta_Pos when rollover does occur
                if (delta_Left_Pos > 2047) {
                    delta_Left_Pos = delta_Left_Pos - 4096;
                } else if (delta_Left_Pos < -2047) {
                    delta_Left_Pos =  delta_Left_Pos + 4096;
                }
                if (delta_Right_Pos > 2047) {
                    delta_Right_Pos = delta_Right_Pos - 4096;
                } else if (delta_Right_Pos < -2047) {
                    delta_Right_Pos =  delta_Right_Pos + 4096;
                }


                /*********************************************************************************
                CONVERTING OVERALL LINEAR AND ANGULAR TARGET TO INDIVIDUAL WHEEL TARGET VELOCITIES
                *********************************************************************************/
                mat_target_vels.Clear();                                        // Clear matrix
                mat_target_vels << target_linear_vel    << target_turn_vel;     // Fill matrix with target velocities

                mat_target_wheel_vels.Clear();                                              // Clear matrix
                mat_target_wheel_vels = (mat_convert_target_vels * mat_target_vels);        // Perform cross-product multiplication

                double right_target_vel = WHEEL_RADIUS * double(mat_target_wheel_vels.getNumber(1, 1)); // Calculate target velocity for right wheel (m/s)
                double left_target_vel = WHEEL_RADIUS * double(mat_target_wheel_vels.getNumber(2, 1)); // Calculate target velocity for left wheel (m/s)


                /*****************
                COMPUTE VELOCITIES
                *****************/
                double dt = current_time.toSec() - last_time.toSec();   // Calculate change in time
                double v_left = (delta_Left_Pos * M_PER_STEP) / dt;     // Calculate speed of left wheel (m/s)
                double v_right = (delta_Right_Pos * M_PER_STEP) / dt;   // Calculate speed of right wheel (m/s)

                mat_wheel_vels.Clear();                                             // Clear matrix
                mat_wheel_vels << v_right / WHEEL_RADIUS  << v_left / WHEEL_RADIUS; // Fill matrix with angular velocities of wheels (linear velocity = radius * angular velocity)

                mat_robot_vels.Clear();                                             // Clear matrix
                mat_robot_vels = (mat_convert_actual_vels * mat_wheel_vels);        // Perform cross-product multiplication

                vrobot = mat_robot_vels.getNumber(1, 1);        // Get linear velocity of robot
                vth = mat_robot_vels.getNumber(2, 1);           // Get angular velocity of robot


                /**********************************
                MOTOR DIRECTION AND BRAKING CONTROL
                **********************************/
                if (leftESC.braking_state == STATIONARY_BRAKES_ON && (target_linear_vel != 0 || target_turn_vel != 0)) {
                    leftESC.braking_state = NOT_BRAKING;
                    leftESC.stationary_braking_timer.start();
                    leftBrakeRelay_4.write(1.0f);
                    leftBrakeRelay_5.write(1.0f);
                } else if (leftESC.braking_state == NOT_BRAKING && leftESC.stationary_braking_timer.read_ms() > 12) {
                    leftBrakeRelay_1 = RELAY_CONNECT_AC;
                    leftBrakeRelay_2 = RELAY_CONNECT_AC;
                    leftBrakeRelay_3 = RELAY_CONNECT_AC;
                    leftESC.stationary_braking_timer.stop();
                    leftESC.stationary_braking_timer.reset();
                }

                if (rightESC.braking_state == STATIONARY_BRAKES_ON && (target_linear_vel != 0 || target_turn_vel != 0)) {
                    rightESC.braking_state = NOT_BRAKING;
                    rightESC.stationary_braking_timer.start();
                    rightBrakeRelay_4.write(1.0f);
                    rightBrakeRelay_5.write(1.0f);
                } else if (rightESC.braking_state == NOT_BRAKING && rightESC.stationary_braking_timer.read_ms() > 12) {
                    rightBrakeRelay_1 = RELAY_CONNECT_AC;
                    rightBrakeRelay_2 = RELAY_CONNECT_AC;
                    rightBrakeRelay_3 = RELAY_CONNECT_AC;
                    rightESC.stationary_braking_timer.stop();
                    rightESC.stationary_braking_timer.reset();
                }

                if (leftESC.braking_state == NOT_BRAKING && rightESC.braking_state == NOT_BRAKING) {
                    if (target_linear_vel > 0.0f) {                 // Set wheel directions to forwards if target linear velocity is positive
                        leftESC.setDirection(LEFT_MOTOR_FORWARDS);
                        rightESC.setDirection(RIGHT_MOTOR_FORWARDS);
                        leftESC.setEnable(ENABLE);                  // Enable motors
                        rightESC.setEnable(ENABLE);
                        motorEnableLED = 1;                         // Turn on motor-enabled LED
                    } else if (target_linear_vel < 0.0f) {           // Set wheel directions to backwards if target linear velocity is negative
                        leftESC.setDirection(LEFT_MOTOR_BACKWARDS);
                        rightESC.setDirection(RIGHT_MOTOR_BACKWARDS);
                        leftESC.setEnable(ENABLE);                  // Enable motors
                        rightESC.setEnable(ENABLE);
                        motorEnableLED = 1;                         // Turn on motor-enabled LED
                    } else if (target_linear_vel == 0.0f && target_turn_vel > 0.0f) { // Set wheel directions accordingly spin on the spot anti-clockwise
                        leftESC.setDirection(LEFT_MOTOR_BACKWARDS);
                        rightESC.setDirection(RIGHT_MOTOR_FORWARDS);
                        leftESC.setEnable(ENABLE);                  // Enable motors
                        rightESC.setEnable(ENABLE);
                        motorEnableLED = 1;                         // Turn on motor-enabled LED
                    } else if (target_linear_vel == 0.0f && target_turn_vel < 0.0f) { // Set wheel directions accordingly spin on the spot clockwise
                        leftESC.setDirection(LEFT_MOTOR_FORWARDS);
                        rightESC.setDirection(RIGHT_MOTOR_BACKWARDS);
                        leftESC.setEnable(ENABLE);                  // Enable motors
                        rightESC.setEnable(ENABLE);
                        motorEnableLED = 1;                         // Turn on motor-enabled LED
                    } else {                                        // Disable motors if all velocities are zero
                        leftESC.setEnable(DISABLE);                 // Disable motors
                        rightESC.setEnable(DISABLE);
                        leftESC.stationary_braking_timer.start();
                        rightESC.stationary_braking_timer.start();
                        leftBrakeRelay_1 = RELAY_CONNECT_SHORT;
                        leftBrakeRelay_2 = RELAY_CONNECT_SHORT;
                        leftBrakeRelay_3 = RELAY_CONNECT_SHORT;
                        rightBrakeRelay_1 = RELAY_CONNECT_SHORT;
                        rightBrakeRelay_2 = RELAY_CONNECT_SHORT;
                        rightBrakeRelay_3 = RELAY_CONNECT_SHORT;
                        leftESC.braking_state = STATIONARY;
                        rightESC.braking_state = STATIONARY;

                        motorEnableLED = 0;                         // Turn off motor-enabled LED
                    }

                }
                /*
                leftBrakeRelay_1 = RELAY_CONNECT_SHORT;
                leftBrakeRelay_2 = RELAY_CONNECT_SHORT;
                leftBrakeRelay_3 = RELAY_CONNECT_SHORT;
                rightBrakeRelay_1 = RELAY_CONNECT_SHORT;
                rightBrakeRelay_2 = RELAY_CONNECT_SHORT;
                rightBrakeRelay_3 = RELAY_CONNECT_SHORT;
                
                leftESC.braking_state = START_BRAKING;
                leftESC.braking_timer.start();
                rightESC.braking_state = START_BRAKING;
                rightESC.braking_timer.start();
                */
                switch (leftESC.braking_state) {

                case STATIONARY:
                    if (leftESC.stationary_braking_timer.read_ms() > 12) {
                        leftBrakeRelay_4.write(0.0f);
                        leftBrakeRelay_5.write(0.0f);
                        leftESC.braking_state = STATIONARY_BRAKES_ON;
                        leftESC.stationary_braking_timer.stop();
                        leftESC.stationary_braking_timer.reset();
                    }
                    break;

                case NOT_BRAKING:
                    if ( abs(v_left) > (abs(left_target_vel) + 0.20)) {
                        leftESC.setEnable(DISABLE);
                        leftESC.braking_timer.start();
                        leftBrakeRelay_1 = RELAY_CONNECT_SHORT;
                        leftBrakeRelay_2 = RELAY_CONNECT_SHORT;
                        leftBrakeRelay_3 = RELAY_CONNECT_SHORT;
                        leftESC.braking_state = START_BRAKING;
                    }
                    break;

                case START_BRAKING:
                    if (leftESC.braking_timer.read_ms() > 12) {
                        leftBrakeRelay_4.write(0.0f);
                        leftBrakeRelay_5.write(0.0f);
                        leftESC.braking_state = BRAKING;
                        leftESC.setEnable(DISABLE);
                        leftESC.braking_timer.reset();
                    }
                    break;

                case BRAKING:
                    if (abs(v_left) < (abs(left_target_vel) * 0.75)) {
                        if (leftESC.braking_timer.read_ms() > 200) {
                            leftESC.braking_timer.reset();
                            leftBrakeRelay_4.write(1.0f);
                            leftBrakeRelay_5.write(1.0f);
                            leftESC.braking_state = START_NOT_BRAKING;
                        }
                    } else {       
                                    
                                    // When going over bumps the reported speed would sometimes
                                    // stay really high so would never exist braking. This can
                                    // hopefully be fixed by resetting comms to encoders.
                        if (abs(v_left) > (abs(left_target_vel) + 0.5)) {
                            AS5600 encoderLeft(PB_11, PB_10);       // Try to re-establish comms to encoders
                        }

                    }
                    break;

                case START_NOT_BRAKING:
                    if (leftESC.braking_timer.read_ms() > 12) {
                        leftBrakeRelay_1 = RELAY_CONNECT_AC;
                        leftBrakeRelay_2 = RELAY_CONNECT_AC;
                        leftBrakeRelay_3 = RELAY_CONNECT_AC;
                        leftESC.braking_timer.stop();
                        leftESC.braking_timer.reset();
                        leftESC.braking_state = NOT_BRAKING;
                        //leftMotorPID.resetIntegralTerm();
                    }
                    break;

                default:
                    break;

                }


                switch (rightESC.braking_state) {

                case STATIONARY:
                    if (rightESC.stationary_braking_timer.read_ms() > 12) {
                        rightBrakeRelay_4.write(0.0f);
                        rightBrakeRelay_5.write(0.0f);
                        rightESC.braking_state = STATIONARY_BRAKES_ON;
                        rightESC.stationary_braking_timer.stop();
                        rightESC.stationary_braking_timer.reset();
                    }
                    break;

                case NOT_BRAKING:
                    if (abs(v_right) > (abs(right_target_vel) + 0.20)) {
                        rightESC.setEnable(DISABLE);
                        rightESC.braking_timer.start();
                        rightBrakeRelay_1 = RELAY_CONNECT_SHORT;
                        rightBrakeRelay_2 = RELAY_CONNECT_SHORT;
                        rightBrakeRelay_3 = RELAY_CONNECT_SHORT;
                        rightESC.braking_state = START_BRAKING;
                    }
                    break;

                case START_BRAKING:
                    if (rightESC.braking_timer.read_ms() > 12) {
                        rightBrakeRelay_4.write(0.0f);
                        rightBrakeRelay_5.write(0.0f);
                        rightESC.braking_state = BRAKING;
                        rightESC.setEnable(DISABLE);
                        rightESC.braking_timer.reset();
                    }
                    break;

                case BRAKING:
                    if (abs(v_right) < (abs(right_target_vel) * 0.75)) {
                        if (rightESC.braking_timer.read_ms() > 200) {
                            rightESC.braking_timer.reset();
                            rightBrakeRelay_4.write(1.0f);
                            rightBrakeRelay_5.write(1.0f);
                            rightESC.braking_state = START_NOT_BRAKING;
                        }
                    } else {
                        if(abs(v_right - old_v_right) < 0.001){
                            AS5600 encoderRight(I2C_SDA, I2C_SCL); 
                        }
                        old_v_right = v_right;
                                    // When going over bumps the reported speed would sometimes
                                    // stay really high so would never exist braking. This can
                                    // hopefully be fixed by resetting comms to encoders.
                        if (abs(v_right) > (abs(right_target_vel) + 0.5)) {
                            AS5600 encoderRight(I2C_SDA, I2C_SCL); // Try to re-establish comms to encoders
                        }
                    }
                    break;

                case START_NOT_BRAKING:
                    if (rightESC.braking_timer.read_ms() > 12) {
                        rightBrakeRelay_1 = RELAY_CONNECT_AC;
                        rightBrakeRelay_2 = RELAY_CONNECT_AC;
                        rightBrakeRelay_3 = RELAY_CONNECT_AC;
                        rightESC.braking_timer.stop();
                        rightESC.braking_timer.reset();
                        rightESC.braking_state = NOT_BRAKING;
                        //rightMotorPID.resetIntegralTerm();
                    }
                    break;

                default:
                    break;

                }


                // Reset ESCs if movement ceases when it should not
                if (target_linear_vel != 0 || target_turn_vel != 0) {
                    motor_stationary_timer.start();
                }
                if (motor_stationary_timer.read_ms() > 2000 && delta_Left_Pos == 0 && delta_Right_Pos == 0) {
                    // Reset motors if cmd_vel is sending velocities but robot isn't moving for more than 2000ms
                    leftESC.setEnable(DISABLE);
                    rightESC.setEnable(DISABLE);
                    leftESC.setSpeed(0);            // Set speed of motors to nothing (opposite for right motor)
                    rightESC.setSpeed(1);
                    leftESC.setDirection(LEFT_MOTOR_FORWARDS);
                    rightESC.setDirection(RIGHT_MOTOR_FORWARDS);
                    motorEnableLED = 0;                              // Turn off motor-enabled LED
                    motor_stationary_timer.stop();                   // Stop timer
                    motor_stationary_timer.reset();                  // Reset timer
                } else if (motor_stationary_timer.read_ms() > 2000) {
                    motor_stationary_timer.reset();
                }



                /**************
                PID CONTROLLERS
                **************/
                double speed_control_left = leftMotorPID.calculate(abs(v_left), abs(left_target_vel), dt);      // Calculate new value to write to left motor
                double speed_control_right = rightMotorPID.calculate(abs(v_right), abs(right_target_vel), dt);  // Calculate new value to write to right motor

                leftESC.setSpeed(speed_control_left);   // Set speed of motors
                rightESC.setSpeed(1 - speed_control_right); // Set speed of motors

                // If PID debugging is desired then publish the required data
                if (doPIDDebugging) {

                    left_motor_val_msg.data = float(vth);                   // Assign value written to left motor to relevant message
                    right_motor_val_msg.data = float(vrobot);               // Assign value written to right motor to relevant message
                    left_motor_val_pub.publish(&left_motor_val_msg);        // Publish message
                    right_motor_val_pub.publish(&right_motor_val_msg);      // Publish message

                    target_linear_vel_msg.data = target_linear_vel;         // Assign target linear velocity to relevant message
                    target_linear_vel_pub.publish(&target_linear_vel_msg);  // Publish message

                    //motor_time_msg.data = float(dt);                        // Assign time taken for control loop to complete to relevant message
                    //motor_time_pub.publish(&motor_time_msg);                // Publish message

                    vel_left_msg.data = float(v_left);
                    vel_right_msg.data = float(v_right);
                    vel_left_pub.publish(&vel_left_msg);
                    vel_right_pub.publish(&vel_right_msg);
                }

                // Disable motors if communication with encoders fails. We expect dt to stay below
                // 15ms and when it loses communication dt increases, so can use this to control motors
                if (dt > 0.020 && !testOdom) {
                    leftESC.setEnable(DISABLE);             // Disable motors
                    rightESC.setEnable(DISABLE);
                    motorEnableLED = 0;                     // Turn off motor-enabled LED
                    AS5600 encoderLeft(PB_11, PB_10);       // Try to re-establish comms to encoders
                    AS5600 encoderRight(I2C_SDA, I2C_SCL);

                } else if (dt > 0.050 && testOdom) {        // Need to check for different length of loop as publishing
                    // multiple odom messages takes a lot longer
                    leftESC.setEnable(DISABLE);             // Disable motors
                    rightESC.setEnable(DISABLE);
                    motorEnableLED = 0;                     // Turn off motor-enabled LED
                    AS5600 encoderLeft(PB_11, PB_10);       // Try to re-establish comms to encoders
                    AS5600 encoderRight(I2C_SDA, I2C_SCL);
                }

                /**************************************
                COMPUTE ODOMETRY - EULERIAN INTEGRATION
                **************************************/
                // This is needed as Exact method and Runge-Kutta method need new_theta
                // Compute odometry in a typical way given the velocities of the robot
                double delta_x = (vrobot * cos(theta)) * dt;
                double delta_y = (vrobot * sin(theta)) * dt;
                double delta_th = vth * dt;

                new_x += delta_x;           // Increase estimated x-coordinate position with new change in x
                new_y += delta_y;           // Increase estimated y-coordinate position with new change in y
                new_theta += delta_th;      // Increase estimated heading with new change in theta


                /***********************************
                COMPUTE ODOMETRY - EXACT INTEGRATION
                ***********************************/
                double delta_x_exact = 0.0;
                double delta_y_exact = 0.0;

                // Need to check if angular velocity doesn't equal zero as there is a critical division
                if (vth != 0.0) {
                    delta_x_exact = (vrobot / vth) * ( sin(new_theta) - sin(theta) );
                    delta_y_exact = (vrobot / vth) * ( cos(new_theta) - cos(theta) );
                }

                new_x_exact += delta_x_exact;
                new_y_exact -= delta_y_exact;


                /**********************************
                SET ODOM PARAMS - EXACT INTEGRATION
                **********************************/
                // Since all odometry is 6DOF we'll need a quaternion created from yaw
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);                // Since all odometry is 6DOF we'll need a quaternion created from yaw

                // Now set up the odometry message
                odom_exact_msg.header.stamp = current_time;
                odom_exact_msg.header.frame_id = "odom";

                // Set the position
                odom_exact_msg.pose.pose.position.x = new_x_exact;
                odom_exact_msg.pose.pose.position.y = new_y_exact;
                odom_exact_msg.pose.pose.position.z = 0.0;
                odom_exact_msg.pose.pose.orientation = odom_quat;

                // Set the velocity
                odom_exact_msg.child_frame_id = "base_link";
                odom_exact_msg.twist.twist.linear.x = vrobot;
                odom_exact_msg.twist.twist.linear.y = 0.0;        // Will always be zero as assuming there will be no "slippage"
                odom_exact_msg.twist.twist.angular.z = vth;

                if (!useEKF) {
                    // Publish the message over ROS
                    odom_pub.publish(&odom_exact_msg);
                } else {
                    wheel_odom_exact_pub.publish(&odom_exact_msg);
                }


                /******************************
                BROADCAST A TF IF NOT USING EKF
                ******************************/
                if (!useEKF) {
                    // First publish the transform over tf
                    geometry_msgs::TransformStamped odom_trans;
                    odom_trans.header.stamp = current_time;
                    odom_trans.header.frame_id = "odom";
                    odom_trans.child_frame_id = "base_link";

                    odom_trans.transform.translation.x = new_x_exact;
                    odom_trans.transform.translation.y = new_y_exact;
                    odom_trans.transform.translation.z = 0.0;
                    odom_trans.transform.rotation = odom_quat;

                    // Send the transform
                    odom_broadcaster.sendTransform(odom_trans);
                }


                // If testing different odometric localisation methods
                if (testOdom) {

                    /* delta_th is the same for all methods of odometric localisation */

                    /******************************************
                    COMPUTE ODOMETRY - SECOND ORDER RUNGE-KUTTA
                    ******************************************/
                    double delta_x_runge_kutta = vrobot * dt * cos( theta + (delta_th / 2) );
                    double delta_y_runge_kutta = vrobot * dt * sin( theta + (delta_th / 2) );

                    new_x_runge_kutta += delta_x_runge_kutta;
                    new_y_runge_kutta += delta_y_runge_kutta;

                    // Since all odometry is 6DOF we'll need a quaternion created from yaw
                    geometry_msgs::Quaternion odom_quat_testing = tf::createQuaternionFromYaw(theta);


                    /*****************************************
                    SET ODOM PARAMS - SECOND ORDER RUNGE-KUTTA
                    *****************************************/
                    // Now set up the odometry message
                    odom_runge_kutta_msg.header.stamp = current_time;
                    odom_runge_kutta_msg.header.frame_id = "odom";

                    // Set the position
                    odom_runge_kutta_msg.pose.pose.position.x = new_x_runge_kutta;
                    odom_runge_kutta_msg.pose.pose.position.y = new_y_runge_kutta;
                    odom_runge_kutta_msg.pose.pose.position.z = 0.0;
                    odom_runge_kutta_msg.pose.pose.orientation = odom_quat_testing;

                    // Set the velocity
                    odom_runge_kutta_msg.child_frame_id = "base_link";
                    odom_runge_kutta_msg.twist.twist.linear.x = vrobot;
                    odom_runge_kutta_msg.twist.twist.linear.y = 0.0;        // Will always be zero as assuming there will be no "slippage"
                    odom_runge_kutta_msg.twist.twist.angular.z = vth;

                    wheel_odom_runge_kutta_pub.publish(&odom_runge_kutta_msg);


                    /*************************************
                    SET ODOM PARAMS - EULERIAN INTEGRATION
                    *************************************/
                    // Now set up the odometry message
                    odom_eulerian_msg.header.stamp = current_time;
                    odom_eulerian_msg.header.frame_id = "odom";

                    // Set the position
                    odom_eulerian_msg.pose.pose.position.x = new_x;
                    odom_eulerian_msg.pose.pose.position.y = new_y;
                    odom_eulerian_msg.pose.pose.position.z = 0.0;
                    odom_eulerian_msg.pose.pose.orientation = odom_quat_testing;

                    // Set the velocity
                    odom_eulerian_msg.child_frame_id = "base_link";
                    odom_eulerian_msg.twist.twist.linear.x = vrobot;
                    odom_eulerian_msg.twist.twist.linear.y = 0.0;        // Will always be zero as assuming there will be no "slippage"
                    odom_eulerian_msg.twist.twist.angular.z = vth;

                    wheel_odom_eulerian_pub.publish(&odom_eulerian_msg);

                }

                // Assign old position of encoders to new positions
                old_Pos_Left = new_Pos_Left;
                old_Pos_Right = new_Pos_Right;

                // Assign old time to current time
                last_time = current_time;

                // Assign current heading to new estimated heading
                theta = new_theta;

                // Allow time for messages to be transmitted serially
                wait_ms(4);

            }

            // Disable motors if lose connection to ROS master
            leftESC.setEnable(DISABLE);
            rightESC.setEnable(DISABLE);
            leftESC.setSpeed(0);         // Set speed of motors to nothing (opposite for right motor)
            rightESC.setSpeed(1);
            // Turn off all LEDs
            motorEnableLED = 0;
            tickerLED = 0;
            debugLED = 0;
            leftBrakeRelay_1 = RELAY_CONNECT_SHORT;
            leftBrakeRelay_2 = RELAY_CONNECT_SHORT;
            leftBrakeRelay_3 = RELAY_CONNECT_SHORT;
            rightBrakeRelay_1 = RELAY_CONNECT_SHORT;
            rightBrakeRelay_2 = RELAY_CONNECT_SHORT;
            rightBrakeRelay_3 = RELAY_CONNECT_SHORT;
            wait_ms(12);
            leftBrakeRelay_4.write(0.0f);
            leftBrakeRelay_5.write(0.0f);
            rightBrakeRelay_4.write(0.0f);
            rightBrakeRelay_5.write(0.0f);
            leftESC.braking_state = NOT_BRAKING;    // Reset state machines
            rightESC.braking_state = NOT_BRAKING;
            
        }

    }

}

/*================================= END MAIN =================================*/

/*============================================================================*/