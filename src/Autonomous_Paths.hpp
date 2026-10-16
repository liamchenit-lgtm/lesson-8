#ifndef AUTON_HPP
#define AUTON_HPP

#include "globals.hpp"

/*===================== Angular Movement Examples ======================
- Angular motion is the simplest form of movement froom LemLibs
- It utilizes the angular PID as well as the IMU sensor
- These functions use absolute angles, NOT relative
    - Absolute angles: Angle readings remain the same regardless of the robot's position
        - 90 degrees now is the same as 90 degrees after 100 other turns
        - 360 degrees != 0 degrees
    - Relative angles: Angle readings depend on the robot's current position
        - 90 degrees now is different than 90 degrees after 100 other turns
        - If you want to use relative angles, use: chassis.getPose().theta

- The following website is helpful to visualize paths: https://path.jerryio.com/

1) chassis.turnToHeading(
    270,                // target heading in degrees
    4000,               // timeout in milliseconds
    {.maxSpeed = 120},  // Parameters ie: Max motor speed (default 120)
    true                // Nonblocking code (true by default)
); 
- The 2 mandatory parameters are the target heading and the timeout (max time allowed for this movement)
- The 3rd parameter is if you want to specify any additional parameters like max speed, min speed, direction, etc.
- The 4th parameter is whether the code should block (pause) until the movement is complete
For a full list of parameters: https://lemlib.readthedocs.io/en/stable/api/chassis.html#_CPPv4N6lemlib7Chassis13turnToHeadingEfi19TurnToHeadingParamsb

2) chassis.turnToPoint(
    45,                 // x location
    -45,                // y location
    1000,               // timeout in milliseconds
    {.maxSpeed = 120},  // Parameters ie: Max motor speed (default 120)
    true                // Nonblocking code (true by default)
);
- Very similar to turnToHeading, but instead of a heading, you specify a point in the field to face
- This is helpful for when you want to face a specific game element
- Tip: The further the point is away from the chassis, the more accurate it is 

3) chassis.swingToHeading(
    270,                // target heading in degrees
    4000,               // timeout in milliseconds
    {.maxSpeed = 120},  // Parameters ie: Max motor speed (default 120)
    true                // Nonblocking code (true by default)
);
- turnToHeading works by spinning one side of the drivetrain forward and one backward
- swingToHeading differs by turning one drivetrain on and the other OFF instead of backward

4) chassis.swingToPoint(
    45,                 // x location
    -45,                // y location
    1000,               // timeout in milliseconds
    {.maxSpeed = 120},  // Parameters ie: Max motor speed (default 120)
    true                // Nonblocking code (true by default)
);
- same as turnToPoint but one side of the drivetrain is powered off instead of backwards


Practice 
    1) After trying out both red auton paths, try experimenting with different values
    2) When can swing be helpful? How about turning?
    3) Open: https://lemlib.readthedocs.io/en/stable/api/chassis.html#_CPPv4N6lemlib17TurnToPointParamsE
        - Read about the available parameters for each of the 4 functions you just learned
        - Pro Tip: You can also Cntrl+Click on functions to see where they originate from,
            There is usually comments documenting how to use them!
        - Make predictions and experiment with the parameters to test your understanding
        - Write below what each parameters does in wording that you can understand
forwards:
direction:
maxSpeed:
minSpeed:
earlyExitRange:
*/
void redAuton1() {
    chassis.setPose(0, 0, 0); // set the position of the chassis to x = 0, y = 0, theta = 0

    // 1) turn to 90 degrees in maximum 5 seconds, no parameters and wait for the movement to complete
    chassis.turnToHeading(90, 5000, {}, false);
    pros::delay(500);
    // 2) ?
    chassis.turnToPoint(0, -100, 5000, {}, false);
    pros::delay(500);
    // 3) ?
    chassis.turnToHeading(90, 5000, {}, false);
}

void redAuton2() {
    chassis.setPose(0, 0, 0); // set the position of the chassis to x = 0, y = 0, theta = 0

    // 1) turn to 90 degrees in maximum 5 seconds, no parameters and wait for the movement to complete
    chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 5000, {}, false);
    pros::delay(500);
    // 2) ?
    chassis.swingToPoint(0, -100, lemlib::DriveSide::RIGHT, 5000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    pros::delay(500);
    // 3) ?
    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 5000, {}, false);
}


/*===================== Lateral Movement Examples ======================
- One step beyond angular motion is lateral motion (actually moving around)
- LemLibs offers 2 functions for this, and they require both the angular and lateral PID controller
- This means both PIDs will need to be tuned (more on this next lesson)

1) chassis.moveToPoint(
    10,             // absolute x location
    20,             // absolute y location
    5000,           // timeout 
    {},             // more parameters (optional)
    true            // Is Nonblocking? (optional)
);  
- The simplist and fastest way to move the robot to a certain position
- This function will move the robot to an absolute postion on the field
- the x and y values are in inches, and the heading is in degrees
- By default, the robot will attempt to face the point at all times

2) chasses.moveToPos(
    10,             // absolute x location
    20,             // absolute y location
    90,             // final heading direction
    5000,           // timeout 
    {},             // Parameters (optional)
    true            // is blocking? (optional)
);
- moveToPos offers a higher degree of control over the robot's movement at the cost of speed
- the advantage is that you can control what heading your robot ends up in
- In general, moveToPos instructions take longer than moveToPoints, so only use it if you require the accuracy
- This function uses the boomerang controller, which attempts to find a smooth route that results in the robot facing the desired angle
    - Take a look at this link: https://www.desmos.com/calculator/sptjw5szex
    - Play around with the sliders
    - How does each variable affect the path?
   
Practice 
    1) After trying out both blue auton paths, try experimenting with different values
    2) Can you think of another way to achieve the same result as moveToPoint? Try it out!
    3) Open: https://lemlib.readthedocs.io/en/stable/api/chassis.html#_CPPv4N6lemlib17TurnToPointParamsE
        - Read about the available parameters for both of the functions you just learned
        - Pro Tip: You can also Cntrl+Click on functions to see where they originate from,
            There is usually comments documenting how to use them!
        - Make predictions and experiment with the parameters to test your understanding
        - Write below what each parameters does in wording that you can understand
        - Pay special attention to lead and horizontal drift, which are unique to moveToPos
forwards:
maxSpeed:
minSpeed:
earlyExitRange:

lead:
horizontalDrift:
*/
void blueAuton1() { 
    // Task: Guess where the robot will end up after each line, were your predictions correct?
    chassis.setPose(0, 0, 0); // reset position of chassis
    // 1) ? 
    chassis.moveToPoint(20, 20, 5000, {}, false);
    pros::delay(1000);
    // 2) ?
    chassis.moveToPoint(-20, 20, 5000, {}, false);
    pros::delay(1000);
    // 3) ?
    chassis.moveToPoint(0, 0, 5000, {}, false);

}

void blueAuton2() {
    chassis.setPose(0, 0, 0); // reset position of chassis

    // 1) ?
    chassis.moveToPose(20, 20, 90, 5000, {}, false);
    pros::delay(500);
    
    // 2) ?
    chassis.moveToPose(0, 0, 0, 5000, {}, false);

}

/* =========================== Autonomous Challenge ===========================
Time to get creative!
    - Now that you know all the basic movements and turning functions, write an autonomous program to complete a task of your choosing!
    - try to encoporate a variety of functions, and simulate a route that is realistic to some degree
*/
void exampleAuton() {
}

#endif
