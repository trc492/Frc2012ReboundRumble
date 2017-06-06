#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="main.cpp" />
///
/// <summary>
///     This main module contains the entry point of the program.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#define _DBGTRACE_ENABLED

#define _USE_COLORFONT
#define _USE_TANK_DRIVE
//#define _USE_DUAL_JOYSTICKS
//#define _USE_LINE_FOLLOWER

//#define _DEBUG_TELEOP
//#define _DEBUG_JOYSTICK
//#define _DEBUG_DRIVEBASE

//#define _ENABLE_DATALOGGER
//#define _LOGDATA_PIDCTRL
//#define _LOGDATA_DRIVEBASE

#define PROGRAM_NAME            "Demo Sample"

//
// Library includes.
//
#include "WPILib.h"
#include "RobotInfo.h"          //Robot configurations
#include "TrcLib.h"

//
// Tracing info.
//
#define MOD_DRIVEBASE           TGenModId(1)

#define TRACE_MODULES           (MOD_MAIN)
#define TRACE_LEVEL             FUNC
#define MSG_LEVEL               INFO

//
// Project includes.
//
#include "DashboardDataFormat.h"

#include "DriveBase.h"          //DriveBase subsystem

#include "TrcRobot.h"           //Main Robot module
#include "Auto.h"               //Autonomous mode
#include "TeleOp.h"             //TeleOp mode

START_ROBOT_CLASS(TrcRobot);
