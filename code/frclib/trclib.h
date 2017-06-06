#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcLib.h" />
///
/// <summary>
///     This module includes all the TRC library modules and also contains the
///     implementation of the forward referenced functions.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCLIB_H
#define _TRCLIB_H

#include <math.h>
#include <hostlib.h>
//
// Common and Debugging modules.
//
#include "TrcDefs.h"
#include "Ansi.h"
#include "DbgTrace.h"
#include "Console.h"
#include "DataLogger.h"
#include "PerfData.h"
//
// Tasks, Events and State Machines.
//
#include "Task.h"
#include "CoopMTRobot.h"
#include "Event.h"
#include "TrcTimer.h"
#include "StateMachine.h"
//
// Inputs.
//
#include "KalmanFilter.h"
#include "IIRFilter.h"
#include "TrcJoystick.h"
#include "DigitalIn.h"
#include "AnalogIn.h"
#include "DSEnhDin.h"
#include "TrcAccel.h"
#include "VisionTask.h"
//
// Outputs.
//
#include "TrcSol.h"
#include "SolLight.h"
#include "CanJag.h"
#include "TrcPIDCtrl.h"
#include "TrcPIDMotor.h"
#include "TrcPIDDrive.h"
#include "LineFollower.h"

#endif  //#ifndef _TRCLIB_H
