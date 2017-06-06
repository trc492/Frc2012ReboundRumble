#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="RobotInfo.h" />
///
/// <summary>
///     This module contains the physical definitions of the robot.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _ROBOTINFO_H
#define _ROBOTINFO_H

//
// CAN IDs for Jaguars.
//
#define CANID_LEFTFRONT_JAG             2       //purple
#define CANID_LEFTREAR_JAG              3       //blue
#define CANID_RIGHTFRONT_JAG            4       //yellow
#define CANID_RIGHTREAR_JAG             5       //green

//
// Analog Channels.
//
#define AIN_GYRO                        1
#define AIN_SONAR_LEFT                  2
#define AIN_SONAR_FRONT                 3
#define AIN_SONAR_RIGHT                 4

//
// Digital Input Channels.
//
#define DIN_LEFT_LIGHTSENSOR            12  
#define DIN_CENTER_LIGHTSENSOR          13
#define DIN_RIGHT_LIGHTSENSOR           14

//
// Solenoid Channels.
//
#define SOL_LEFTLIGHT_RED               ((UINT32)1)
#define SOL_LEFTLIGHT_GREEN             ((UINT32)2)
#define SOL_LEFTLIGHT_BLUE              ((UINT32)3)
#define SOL_RIGHTLIGHT_RED              ((UINT32)4)
#define SOL_RIGHTLIGHT_GREEN            ((UINT32)5)
#define SOL_RIGHTLIGHT_BLUE             ((UINT32)6)
#define SOL_LIGHT_SENSOR_POWER          ((UINT32)7)

//
// Input Subsystems.
//
#define DEADBAND_INPUT_THRESHOLD        0.15
#define JSPORT_DRIVE_LEFT               1
#ifdef _USE_DUAL_JOYSTICKS
#define JSPORT_DRIVE_RIGHT              2
#endif
#define CAMERA_IP                       "10.4.92.11"

//
// DriveBase Subsystem.
//
#define DRIVE_ENCODER_PPR               250
#define DISTANCE_PER_REV                (6*PI)  //Wheel circumference

#define MOTOR_LEFT_FRONT_REVERSE        false
#define MOTOR_LEFT_REAR_REVERSE         false
#define MOTOR_RIGHT_FRONT_REVERSE       true
#define MOTOR_RIGHT_REAR_REVERSE        true

#define ENCODER_POLARITY_LEFT_FRONT     1.0
#define ENCODER_POLARITY_LEFT_REAR      1.0
#define ENCODER_POLARITY_RIGHT_FRONT    -1.0
#define ENCODER_POLARITY_RIGHT_REAR     -1.0

#define DRIVE_KP                        0.35
#define DRIVE_KI                        0.0
#define DRIVE_KD                        0.0
#define DRIVE_TOLERANCE                 0.5
#define DRIVE_SETTLING                  200

#define TURN_KP                         0.11
#define TURN_KI                         0.0     //0.001
#define TURN_KD                         1.04    //0.5
#define TURN_TOLERANCE                  0.005
#define TURN_SETTLING                   200

#define LNFOLLOW_KP                     0.15
#define LNFOLLOW_KI                     0.0
#define LNFOLLOW_KD                     0.0
#define LNFOLLOW_TOLERANCE              0.0
#define LNFOLLOW_SETTLING               200

#define LNFOLLOW_LS_NOLINE              0x0
#define LNFOLLOW_LS_Y                   0x5
#define LNFOLLOW_LS_T                   0x7

#define FIND_LINE_DRIVE                 0.3
#define FIND_LINE_TURN                  0.2
#define FOLLOW_LINE_MAXIMUM_SPEED       0.3
#define FOLLOW_LINE_MEDIUM_SPEED        0.2
#define FOLLOW_LINE_SLOW_SPEED          0.15

#define LNFOLLOW_DIR_STRAIGHT           0
#define LNFOLLOW_DIR_FORKLEFT           1
#define LNFOLLOW_DIR_FORKRIGHT          2

#define DRIVE_RANGE_MIN                 -1.0
#define DRIVE_RANGE_MAX                 1.0

#define DRIVE_SCALE_FACTOR              0.5

#endif  //ifndef _ROBOTINFO_H

