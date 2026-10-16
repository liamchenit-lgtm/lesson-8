/*
//================================ LemLib Library ==================================
- LemLibs is a library that provides improved autonomous and opControl for VEX V5 robots.
- It works with PROS and offers detalied documentation: https://lemlib.readthedocs.io/en/stable/about.html

//================== Installation Instructions =================
1) To begin, open the "Integrated Terminal" in the PROS extension and run the command:
    a) pros c add-depot LemLib https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json
        - This will install the LemLib library
        - This command only needs to be run once per machine
    b) pros c apply LemLib
        - This line actually applies LemLibs to your project
        - Run this command every time you create a new project
    c) To update LemLib, run:
        - pros c update LemLib
2) Finally, include the library in your code by adding:
    - #include "lemlib/api.hpp"

//================== Configuring LemLibs ==================
- The first thing we need to do is configure the basic drivetrain and sensors
- This is unique to every robot's size, motors, and ports
- There are many parameters that can be configured
    - Most can be left as default
- The following code is an example of how to configure the drivetrain and sensors
*/

#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "main.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>
#include "lemlib/api.hpp"
#include "sensor_loc.cpp"

/*----------------------Defines----------------------*/
inline std::string autonomousPath = "none"; // Default autonomous path
inline std::string teamColor = "none"; // Default team color

// Constants
constexpr double PI = 3.141592;

/*---Controller---*/
inline pros::Controller master(pros::E_CONTROLLER_MASTER);
inline pros::Controller slave(pros::E_CONTROLLER_PARTNER);

/*------------------Global Variables------------------*/
// provides a cleaner way to access controller buttons
namespace buttons {
    constexpr auto R1 = pros::E_CONTROLLER_DIGITAL_R1;
    constexpr auto R2 = pros::E_CONTROLLER_DIGITAL_R2;
    constexpr auto L1 = pros::E_CONTROLLER_DIGITAL_L1;
    constexpr auto L2 = pros::E_CONTROLLER_DIGITAL_L2;
    constexpr auto A = pros::E_CONTROLLER_DIGITAL_A;
    constexpr auto B = pros::E_CONTROLLER_DIGITAL_B;
    constexpr auto Y = pros::E_CONTROLLER_DIGITAL_Y;
    constexpr auto X = pros::E_CONTROLLER_DIGITAL_X;
    constexpr auto UP = pros::E_CONTROLLER_DIGITAL_UP;
    constexpr auto DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;
    constexpr auto LEFT = pros::E_CONTROLLER_DIGITAL_LEFT;
    constexpr auto RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT;
}

// provides a cleaner way to access motor brake modes
const pros::motor_brake_mode_e_t brake = pros::E_MOTOR_BRAKE_BRAKE; // quickly stops the motor but does not hold position
const pros::motor_brake_mode_e_t coast = pros::E_MOTOR_BRAKE_COAST; // spins freely when stopped
const pros::motor_brake_mode_e_t hold = pros::E_MOTOR_BRAKE_HOLD;   // holds the motor in place when stopped











//==============================================================================================================
//====================================== Start of LemLibs Configuration ========================================
//==============================================================================================================

/*-------------Define all configurations-------------*/
// Standard motor definitions, include the port and gearset
/*
Red Gearset: 100 RPM
Green Gearset: 200 RPM
Blue Gearset: 600 RPM
*/
inline pros::MotorGroup leftDT({1}, pros::v5::MotorGears::green); 
inline pros::MotorGroup rightDT({-5}, pros::v5::MotorGears::green); 
inline pros::Imu IMU(9);


/* ================== Chassis Configuration ==================
- Before we define the chassis, we need to define the wheel diameter and track width
- The track width is the distance between the middle of the left and right wheels
- Wheel diameter is the diameter of the wheels in inches
    - LemLibs has a set of constants for common wheel sizes
- Horizontal drift is a more advanced feature that allows   further tuning during autonomous
    - The default value of 2 is good for now
*/

// Drivetrain Settings
lemlib::Drivetrain drivetrain(&leftDT,                      // left motor group
                              &rightDT,                     // right motor group
                              11.75,                           // Track Width
                              lemlib::Omniwheel::NEW_4,     // Wheel Diameter
                              200,                          // Drivetrain rpm 
                              2                             // Horizontal Drift (default 2)
);


/* ================== Odometry Configuration ==================
- odometry is a way to track the position of the robot using sensors
- for the robot's heading (direction it is facing):
    - IMU is best
- for the robot's vertical position:
    - a vertical tracking wheel is best
    - the built in motor encoder is less ideal
- for the robot's horizontal position:
    - a horizontal tracking wheel is best
    - if not possible, at least use traction wheels to avoid slipping
- The most ideal setup is to have 1 imu, 1 vertical tracking wheel, and 1 horizontal tracking wheel

Tracking Wheels
    - These are iNDEPENTANT freely spinning wheels that are used to track the position of the robot
    - A optical shaft sensor or rotation sensor can be used for ths purpose
    - Tracking wheels are usually found under the robot and not on the drivetrain

- For now, we will not use any tracking wheels
- How to define the odometry sensors is shown below
*/

// Odometry Settings
lemlib::OdomSensors sensors(nullptr,    // vertical tracking wheel 1
                            nullptr,    // vertical tracking wheel 2
                            nullptr,    // horizontal tracking wheel 1
                            nullptr,    // horizontal tracking wheel 2,
                            &IMU        // inertial sensor
);



/* ================== PID Configuration ==================
- The last 2 classes we need to define are the PID controllers
- One is for lateral movement (moving left and right)
- The other is for angular movement (turning left and right)

- This lesson will not dive deep into PID's yet, for know know that:
    - PID 's are used to have precise and smooth movement
    - It adjusts the speed of motors depending on numerous factors
    - Without PID: Robot drives full speed and stops when it reaches the target
    - With PID: Robot accelerates to the target speed, then decelerates smoothly to target
- For now, the standard PID settings are sufficient
*/
// Lateral PID Controller
lemlib::ControllerSettings lateral_controller(10,   // proportional gain (kP)
                                              0,    // integral gain (kI)
                                              3,     // derivative gain (kD)
                                              3,    // anti windup
                                              1,    // small error range, in inches
                                              100,  // small error range timeout, in milliseconds
                                              3,    // large error range, in inches
                                              500,  // large error range timeout, in milliseconds
                                              20    // maximum acceleration (slew)
);

// Angular PID Controller
lemlib::ControllerSettings angular_controller(2,    // proportional gain (kP)
                                              0,    // integral gain (kI)
                                              10,   // derivative gain (kD)
                                              3,    // anti windup
                                              1,    // small error range, in degrees
                                              100,  // small error range timeout, in milliseconds
                                              3,    // large error range, in degrees
                                              500,  // large error range timeout, in milliseconds
                                              0     // maximum acceleration (slew)
);

// Finally, create the hassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


#endif