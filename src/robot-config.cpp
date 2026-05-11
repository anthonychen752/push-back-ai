/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       robot-config.cpp                                         */
/*    Author:       VEX                                                        */
/*    Description:  Robot configuration for VEX V5                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// Robot hardware devices
brain Brain;
competition Competition;
controller Controller1;

// Drive motors
motor LeftDrive1(PORT1, ratio18_1, false);
motor LeftDrive2(PORT2, ratio18_1, false);
motor LeftDrive3(PORT3, ratio18_1, false);
motor RightDrive1(PORT11, ratio18_1, true);
motor RightDrive2(PORT12, ratio18_1, true);
motor RightDrive3(PORT13, ratio18_1, true);

motor_group LeftDrive(LeftDrive1, LeftDrive2, LeftDrive3);
motor_group RightDrive(RightDrive1, RightDrive2, RightDrive3);

// Sensors
rotation TrackingWheel(PORT20, false);
inertial Inertial(PORT7);

// Mechanism motors
motor Intake(PORT4, ratio18_1, true);
motor Hood(PORT5, ratio18_1, false);

void vexcodeInit(void) {
    // Empty — devices are globally constructed above
}
