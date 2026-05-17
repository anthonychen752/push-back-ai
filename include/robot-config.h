/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       robot-config.h                                         */
/*    Description:  Robot configuration for VEX V5                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "vex.h"
#include "ai_jetson.h"

using namespace vex;

// VEX AI Jetson communication (uses VEX's built-in ai::jetson class)
extern ai::jetson jetson_comms;

extern brain Brain;
extern competition Competition;
extern controller Controller1;

extern motor LeftDrive1;
extern motor LeftDrive2;
extern motor LeftDrive3;
extern motor RightDrive1;
extern motor RightDrive2;
extern motor RightDrive3;
extern motor_group LeftDrive;
extern motor_group RightDrive;

extern rotation TrackingWheel;
extern inertial Inertial;

extern motor Intake;
extern motor Flywheel;
extern motor Hood;

#endif