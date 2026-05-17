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

// VEX AI Jetson Nano communication (uses VEX's ai::jetson class)
ai::jetson jetson_comms;

// Drive motors (ratio6_1 = 450 RPM, confirmed for 3177B)
motor LeftDrive1(PORT1, ratio6_1, false);
motor LeftDrive2(PORT2, ratio6_1, false);
motor LeftDrive3(PORT3, ratio6_1, false);
motor RightDrive1(PORT11, ratio6_1, true);
motor RightDrive2(PORT12, ratio6_1, true);
motor RightDrive3(PORT13, ratio6_1, true);

motor_group LeftDrive(LeftDrive1, LeftDrive2, LeftDrive3);
motor_group RightDrive(RightDrive1, RightDrive2, RightDrive3);

// Mechanism motors
motor Intake(PORT4, ratio6_1, true);   // 1st stage intake
motor Flywheel(PORT6, ratio6_1, false); // flywheel
motor Hood(PORT5, ratio6_1, false);     // hood

// Sensors
rotation TrackingWheel(PORT20, false);
inertial Inertial(PORT7);

void vexcodeInit(void) {
    // Empty — devices are globally constructed above
}
