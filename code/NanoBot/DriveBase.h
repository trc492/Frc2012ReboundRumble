#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DriveBase.h" />
///
/// <summary>
///     This module contains the definitions of the DriveBase class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DRIVEBASE_H
#define _DRIVEBASE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVEBASE
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DriveBase"

#define VARID_DRIVE_PID         (VARID_DATAPTR + 1)
#define VARID_DRIVE_KP          (VARID_DATAPTR + 2)
#define VARID_DRIVE_KI          (VARID_DATAPTR + 3)
#define VARID_DRIVE_KD          (VARID_DATAPTR + 4)
#define VARID_TURN_PID          (VARID_DATAPTR + 5)
#define VARID_TURN_KP           (VARID_DATAPTR + 6)
#define VARID_TURN_KI           (VARID_DATAPTR + 7)
#define VARID_TURN_KD           (VARID_DATAPTR + 8)

/**
 * This class defines and implements the DriveBase object. This object
 * inherits the RobotDrive object. It consists of a PIDDrive object and a
 * LineFollower object. It also inherits PIDInput to provide sensor readings
 * to the PID controllers.
 */
class DriveBase: public CoopTask, 
                 public RobotDrive,
                 public PIDInput,
                 public CmdHandler
{
private:
    //
    // Sensors.
    //
    Gyro                m_gyro;
#ifdef _USE_LINE_FOLLOWER
    DigitalInput        m_leftLightSensor;
    DigitalInput        m_centerLightSensor;
    DigitalInput        m_rightLightSensor;
#endif

    SpeedController    *m_leftFrontMotor;
    SpeedController    *m_leftRearMotor;
    SpeedController    *m_rightFrontMotor;
    SpeedController    *m_rightRearMotor;
    TrcPIDCtrl          m_pidCtrlXDrive;
    TrcPIDCtrl          m_pidCtrlYDrive;
    TrcPIDCtrl          m_pidCtrlTurn;
#ifdef _USE_LINE_FOLLOWER
    TrcPIDCtrl          m_pidCtrlLightSensor;
    LineFollower        m_lineFollower;
#endif
    TrcPIDDrive         m_pidDrive;
    float               m_lfInitPos;
    float               m_rfInitPos;
    float               m_lrInitPos;
    float               m_rrInitPos;
#ifdef _LOGDATA_DRIVEBASE
    float               m_heading;
    float               m_xPos;
    float               m_yPos;
    float               m_rotPos;
#endif

public:
    static VAR_ENTRY    m_varTable[];

    /**
     * This function stops the DriveBase subsystem.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

#ifdef _USE_LINE_FOLLOWER
        m_lineFollower.Stop();
#endif
        m_pidDrive.Stop();

        TExit();
    }   //Stop

    /**
     * This function resets the robot position.
     */
    void
    ResetPosition(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_lfInitPos = ((CANJaguar*)m_leftFrontMotor)->GetPosition();
        m_rfInitPos = ((CANJaguar*)m_rightFrontMotor)->GetPosition();
        m_lrInitPos = ((CANJaguar*)m_leftRearMotor)->GetPosition();
        m_rrInitPos = ((CANJaguar*)m_rightRearMotor)->GetPosition();

        TExit();
        return;
    }   //ResetPosition

    /**
     * This function returns the current robot position values.
     *
     * @param xPos Points to a variable to receive the x position.
     * @param yPos Points to a variable to receive the y position.
     * @param rotPos Points to a variable to receive the rotational position.
     */
    void
    GetPosition(
        float *xPos,
        float *yPos,
        float *rotPos
        )
    {
        TLevel(API);
        TEnterMsg(("xPos=%p,yPos=%p,rotPos=%p", xPos, yPos, rotPos));
        //
        // According to RobotDrive::MecanumDrive_Cartesian in WPILib:
        //
        // LF =  x + y + rot    RF = -x + y - rot
        // LR = -x + y + rot    RR =  x + y - rot
        //
        // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
        // => (LF + RR) - (RF + LR) = 4x
        // => x = ((LF + RR) - (RF + LR))/4
        //
        // LF + RF + LR + RR = 4y
        // => y = (LF + RF + LR + RR)/4
        //
        // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
        // => (LF + LR) - (RF + RR) = 4rot
        // => rot = ((LF + LR) - (RF + RR))/4
        //
        float lfEncoder, rfEncoder, lrEncoder, rrEncoder;

        lfEncoder = ENCODER_POLARITY_LEFT_FRONT*
                    (((CANJaguar*)m_leftFrontMotor)->GetPosition() -
                     m_lfInitPos);
        rfEncoder = ENCODER_POLARITY_RIGHT_FRONT*
                    (((CANJaguar*)m_rightFrontMotor)->GetPosition() -
                     m_rfInitPos);
        lrEncoder = ENCODER_POLARITY_LEFT_REAR*
                    (((CANJaguar*)m_leftRearMotor)->GetPosition() -
                     m_lrInitPos);
        rrEncoder = ENCODER_POLARITY_RIGHT_REAR*
                    (((CANJaguar*)m_rightRearMotor)->GetPosition() -
                     m_rrInitPos);
        *xPos = ((lfEncoder + rrEncoder) - (rfEncoder + lrEncoder))*
                DISTANCE_PER_REV/4.0;
        *yPos = (lfEncoder + rfEncoder + lrEncoder + rrEncoder)*
                DISTANCE_PER_REV/4.0;
        *rotPos = ((lfEncoder + lrEncoder) - (rfEncoder + rrEncoder))*
                  DISTANCE_PER_REV/4.0;

#ifdef _DEBUG_DRIVEBASE
        LCDPrintf((LCD_LINE2, "lf=%5.1f,rf=%5.1f", lfEncoder, rfEncoder));
        LCDPrintf((LCD_LINE3, "lr=%5.1f,rr=%5.1f", lrEncoder, rrEncoder));
        LCDPrintf((LCD_LINE4, "x=%5.1f,y=%5.1f", *xPos, *yPos));
        LCDPrintf((LCD_LINE5, "rot=%5.1f", *rotPos));
#endif

        TExitMsg(("!(x=%5.1f,y=%5.1f,rot=%5.1f)", *xPos, *yPos, *rotPos));
        return;
    }   //GetPosition

    /**
     * Constructor for the BaseDrive class.
     *
     * @param leftFrontMotor Specifies the speed controller for the left
     *        front motor.
     * @param leftRearMotor Specifies the speed controller for the left
     *        rear motor.
     * @param rightFrontMotor Specifies the speed controller for the right
     *        front motor.
     * @param rightRearMotor Specifies the speed controller for the right
     *        rear motor.
     */
    DriveBase(
        SpeedController *leftFrontMotor,
        SpeedController *leftRearMotor,
        SpeedController *rightFrontMotor,
        SpeedController *rightRearMotor
        ): RobotDrive(leftFrontMotor, leftRearMotor,
                      rightFrontMotor, rightRearMotor)
         , m_gyro(AIN_GYRO)
#ifdef _USE_LINE_FOLLOWER
         , m_leftLightSensor(DIN_LEFT_LIGHTSENSOR)
         , m_centerLightSensor(DIN_CENTER_LIGHTSENSOR)
         , m_rightLightSensor(DIN_RIGHT_LIGHTSENSOR)
#endif
         , m_leftFrontMotor(leftFrontMotor)
         , m_leftRearMotor(leftRearMotor)
         , m_rightFrontMotor(rightFrontMotor)
         , m_rightRearMotor(rightRearMotor)
         , m_pidCtrlXDrive("DriveX",
                           DRIVE_KP, DRIVE_KI, DRIVE_KD,
                           DRIVE_TOLERANCE, DRIVE_SETTLING)
         , m_pidCtrlYDrive("DriveY",
                           DRIVE_KP, DRIVE_KI, DRIVE_KD,
                           DRIVE_TOLERANCE, DRIVE_SETTLING)
         , m_pidCtrlTurn("DriveTurn",
                         TURN_KP, TURN_KI, TURN_KD,
                         TURN_TOLERANCE, TURN_SETTLING)
#ifdef _USE_LINE_FOLLOWER
         , m_pidCtrlLightSensor("LightSensor",
                                LNFOLLOW_KP, LNFOLLOW_KI, LNFOLLOW_KD,
                                LNFOLLOW_TOLERANCE, LNFOLLOW_SETTLING,
                                PIDCTRLO_ABS_SETPT)
         , m_lineFollower(this,
                          &m_pidCtrlLightSensor,
                          this,
                          LNFOLLOWO_MECANUM_DRIVE)
#endif
         , m_pidDrive(this,
                      &m_pidCtrlXDrive,
                      &m_pidCtrlYDrive,
                      &m_pidCtrlTurn,
                      this,
                      PIDDRIVEO_MECANUM_DRIVE)
    {
        TLevel(INIT);
        TEnterMsg(("leftFront=%p,leftRear=%p,rightFront=%p,rightRear=%p",
                   leftFrontMotor, leftRearMotor, rightFrontMotor,
                   rightRearMotor));
        //
        // Initialize RobotDrive.
        //
        SetInvertedMotor(RobotDrive::kFrontLeftMotor,
                         MOTOR_LEFT_FRONT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearLeftMotor,
                         MOTOR_LEFT_REAR_REVERSE);
#ifdef _USE_TANK_DRIVE
        SetInvertedMotor(RobotDrive::kFrontRightMotor,
                         MOTOR_RIGHT_FRONT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearRightMotor,
                         MOTOR_RIGHT_REAR_REVERSE);
#else
        SetInvertedMotor(RobotDrive::kFrontRightMotor,
                         !MOTOR_RIGHT_FRONT_REVERSE);
        SetInvertedMotor(RobotDrive::kRearRightMotor,
                         !MOTOR_RIGHT_REAR_REVERSE);
#endif
        //
        // Initialize PID controllers.
        //
        m_pidCtrlXDrive.SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlYDrive.SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
        m_pidCtrlTurn.SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
#ifdef _USE_LINE_FOLLOWER
        m_pidCtrlLightSensor.SetOutputRange(DRIVE_RANGE_MIN, DRIVE_RANGE_MAX);
#endif

#ifdef _LOGDATA_DRIVEBASE
        DataLogger *dataLogger = DataLogger::GetInstance();
        dataLogger->AddDataPoint(MOD_NAME, "", "heading", "%f",
                                 DataFloat, &m_heading);
        dataLogger->AddDataPoint(MOD_NAME, "", "xPos", "%f",
                                 DataFloat, &m_xPos);
        dataLogger->AddDataPoint(MOD_NAME, "", "yPos", "%f",
                                 DataFloat, &m_yPos);
        dataLogger->AddDataPoint(MOD_NAME, "", "rotPos", "%f",
                                 DataFloat, &m_rotPos);
#endif

#ifdef _LOGDATA_DRIVEBASE
        RegisterTask(MOD_NAME,
                     TASK_START_MODE | TASK_STOP_MODE | TASK_POST_PERIODIC);
#else
        RegisterTask(MOD_NAME, TASK_START_MODE | TASK_STOP_MODE);
#endif
        RegisterCmdHandler(MOD_NAME, NULL, m_varTable);

        SetExpiration(0.5);
        SetSafetyEnabled(false);

        TExit();
    }   //DriveBase

    /**
     * Destructor for the BaseDrive class.
     */
    virtual
    ~DriveBase(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        SetSafetyEnabled(false);
        Stop();
        UnregisterTask();

        TExit();
    }   //~DriveBase

    /**
     * This function is called by the TaskMgr to start the task.
     *
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStartMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            ResetPosition();
            SetSafetyEnabled(true);
        }

        TExit();
    }   //TaskStartMode

    /**
     * This function is called by the TaskMgr to stop the task.
     *
     * @param mode Specifies the calling mode (autonomous or teleop).
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        if (mode != MODE_DISABLED)
        {
            SetSafetyEnabled(false);
            Stop();
        }

        TExit();
    }   //TaskStopMode

    /**
     * This function overrides the one in the RobotDrive class. It seems
     * there is a bug in RobotDrive::ArcadeDrive where positive rotateValue
     * causes the robot to turn left instead of right. This is opposite to
     * convention. Therefore, we are correcting this bug by overriding the
     * function here.
     *
     * @param moveValue Specifies the value to use for fowards/backwards.
     * @param rotateValue Specifies the value to use for the rotate right/left.
     * @param squaredInputs If set, increases the sensitivity at low speeds.
     */
    void
    ArcadeDrive(
        float moveValue,
        float rotateValue,
        bool squaredInputs = true)
    {
        TLevel(API);
        TEnterMsg(("move=%f,rot=%f,squared=%x",
                   moveValue, rotateValue, squaredInputs));

        RobotDrive::ArcadeDrive(moveValue, -rotateValue, squaredInputs);

        TExit();
        return;
    }   //ArcadeDrive

    /**
     * This function sets PID drive target with the given drive distance and
     * turn angle setpoints.
     *
     * @param distXSetPoint Specifies the lateral target distance relative
     *        to current position.
     * @param distYSetPoint Specifies the forward target distance relative
     *        to current position.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     * @param fStopOnTarget If true, stop PIDDrive when target is reached.
     *        Otherwise, continue to monitor the target and readjust if
     *        necessary.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    DriveSetTarget(
        float  distXSetPoint,
        float  distYSetPoint,
        float  angleSetPoint,
        bool   fStopOnTarget = true,
        Event *notifyEvent = NULL,
        UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("distXSetPt=%f,distYSetPt=%f,angleSetPt=%f,"
                   "fStopOnTarget=%x,event=%p,timeout=%d",
                   distXSetPoint, distYSetPoint, angleSetPoint, fStopOnTarget,
                   notifyEvent, timeout));

        m_pidDrive.SetTarget(distXSetPoint,
                             distYSetPoint,
                             angleSetPoint,
                             fStopOnTarget,
                             notifyEvent,
                             timeout);

        TExit();
        return;
    }   //DriveSetTarget

#ifdef _USE_LINE_FOLLOWER
    /**
     * This function tells the line follower to follow the line.
     *
     * @param targetValue Specifies the center sensor value.
     * @param maxDrivePower Specifies the maximum drive power.
     * @param findDrivePower Specifies the maximum drive power when finding
     *        the target.
     * @param findTurnPower Specifies the maximum turn power when finding
     *        the target.
     * @param notifyEvent Specifies the optional notification event.
     * @param timeout Specific the timeout in msec. No timeout if zero.
     */
    void
    DriveFollowLine(
        float  targetValue,
        float  maxDrivePower,
        float  findDrivePower,
        float  findTurnPower,
        Event *notifyEvent = NULL,
        UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("target=%f,maxDrive=%f,findDrive=%f,findTurn=%f,notifyEvent=%p,timeout=%d",
                   targetValue, maxDrivePower, findDrivePower, findTurnPower,
                   notifyEvent, timeout));

        m_lineFollower.LineFollowStart(targetValue,
                                       maxDrivePower,
                                       findDrivePower,
                                       findTurnPower,
                                       notifyEvent,
                                       timeout);

        TExit();
    }   //DriveFollowLine

    /**
     * This function is called from the LineFollower to get the raw light
     * sensors value. It reads the three light sensors and combine them into
     * a raw bit value.
     *
     * @return Returns the combined bit value of the light sensors.
     */
    UINT32
    GetRawValue(
        void
        )
    {
        UINT32 value = 0;

        TLevel(HIFREQ);
        TEnter();

        value = m_leftLightSensor.Get() << 2;
        value |= m_centerLightSensor.Get() << 1;
        value |= m_rightLightSensor.Get();

        TExitMsg(("=%x", value));
        return value;
    }   //GetRawValue

    /**
     * This function is called to map the raw light sensor value to a value
     * can be used by the PID controller.
     * 
     * @param rawValue Specifies the raw value to lookup.
     *
     * @return Returns the mapped value.
     */
    float
    GetMappedValue(
        UINT32 rawValue
        )
    {
        float value;
        static const float inputMap[8] =
        {
            LNFOLLOW_INVALID_INPUT_VALUE,       //000: n/a (No light)
            -2.0,                               //001: Extreme right
            0.0,                                //010: Center
            -1.0,                               //011: Slight right
            2.0,                                //100: Extreme left
            LNFOLLOW_INVALID_INPUT_VALUE,       //101: n/a (At Y)
            1.0,                                //110: Slight left
            LNFOLLOW_INVALID_INPUT_VALUE        //111: n/a (At T)
        };

        TLevel(HIFREQ);
        TEnterMsg(("rawValue=%x", rawValue));

        value = inputMap[rawValue];

        TExitMsg(("=%f", value));
        return value;
    }   //GetMappedValue
#endif

    /**
     * This function is called from the PID controllers to get the PID input
     * value.
     *
     * @param pidCtrl Specifies the PID controller that needs to read its input
     *        sensor.
     *
     * @return Read and return the sensor value corresponding to the PID
     *         controller.
     */
    float
    GetInput(
        TrcPIDCtrl *pidCtrl
        )
    {
#ifdef _USE_LINE_FOLLOWER
        static float prevInput = 0.0;
        float input = prevInput;
#else
        float input = 0.0;
#endif

        TLevel(CALLBK);
        TEnterMsg(("pidCtrl=%p", pidCtrl));

        if (pidCtrl == &m_pidCtrlTurn)
        {
            input = m_gyro.GetAngle();
        }
#ifdef _USE_LINE_FOLLOWER
        else if (pidCtrl == &m_pidCtrlLightSensor)
        {
            UINT32 rawValue = GetRawValue();

            input = GetMappedValue(rawValue);
            if (input == LNFOLLOW_INVALID_INPUT_VALUE)
            {
                input = prevInput; 
            }
            prevInput = input;
        }
#endif
        else
        {
            float xPos, yPos, rotPos;

            GetPosition(&xPos, &yPos, &rotPos);
            if (pidCtrl == &m_pidCtrlXDrive)
            {
                input = xPos;
            }
            else if (pidCtrl == &m_pidCtrlYDrive)
            {
                input = yPos;
            }
        }

        TExitMsg(("=%f", input));
        return input;
    }   //GetInput

    /**
     * This function returns the heading of the robot.
     *
     * @return Returns the heading of the robot in degrees.
     */
    float
    GetHeading(
        void
        )
    {
        TLevel(API);
        TEnter();

        float heading = m_gyro.GetAngle();

        TExitMsg(("=%f", heading));
        return heading;
    }   //GetHeading

    /**
     * This function resets the robot heading.
     */
    void
    ResetHeading(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_gyro.Reset();

        TExit();
        return;
    }   //ResetHeading

    /**
     * This function prints the value of the variable.
     *
     * @param varEntry Points to the variable table entry.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    GetVariable(
        PVAR_ENTRY varEntry
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd;

        TLevel(CALLBK);
        TEnterMsg(("var=%s", varEntry->varName));

        switch (varEntry->varID)
        {
            case VARID_DRIVE_PID:
                m_pidCtrlYDrive.GetPID(&Kp, &Ki, &Kd);
                printf("Kp=%10.8f, Ki=%10.8f, Kd=%10.8f\n", Kp, Ki, Kd);
                break;

            case VARID_DRIVE_KP:
                m_pidCtrlYDrive.GetPID(&Kp, &Ki, &Kd);
                printf("Kp=%10.8f\n", Kp);
                break;

            case VARID_DRIVE_KI:
                m_pidCtrlYDrive.GetPID(&Kp, &Ki, &Kd);
                printf("Ki=%10.8f\n", Ki);
                break;

            case VARID_DRIVE_KD:
                m_pidCtrlYDrive.GetPID(&Kp, &Ki, &Kd);
                printf("Kd=%10.8f\n", Kd);
                break;

            case VARID_TURN_PID:
                m_pidCtrlTurn.GetPID(&Kp, &Ki, &Kd);
                printf("Kp=%10.8f, Ki=%10.8f, Kd=%10.8f\n", Kp, Ki, Kd);
                break;

            case VARID_TURN_KP:
                m_pidCtrlTurn.GetPID(&Kp, &Ki, &Kd);
                printf("Kp=%10.8f\n", Kp);
                break;

            case VARID_TURN_KI:
                m_pidCtrlTurn.GetPID(&Kp, &Ki, &Kd);
                printf("Ki=%10.8f\n", Ki);
                break;

            case VARID_TURN_KD:
                m_pidCtrlTurn.GetPID(&Kp, &Ki, &Kd);
                printf("Kd=%10.8f\n", Kd);
                break;

            default:
                printf("Error: invalid variable ID (var=%s,ID=%d).\n",
                       varEntry->varName, varEntry->varID);
                rc = ERR_ASSERT;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //GetVariable

    /**
     * This function sets the value of the variable.
     *
     * @param varEntry Points to the variable table entry.
     * @param varData Points to the data to be set to the variable.
     *
     * @return Success Returns ERR_SUCCESS.
     * @return Failure Returns error code.
     */
    int
    SetVariable(
        PVAR_ENTRY varEntry,
        void *varData
        )
    {
        int rc = ERR_SUCCESS;
        double Kp, Ki, Kd;

        TLevel(CALLBK);
        TEnterMsg(("var=%s,varData=%p", varEntry->varName, varData));

        switch (varEntry->varID)
        {
            case VARID_DRIVE_KP:
                m_pidCtrlYDrive.GetPID(&Kp, &Ki, &Kd);
                Kp = *(double *)varData;
                m_pidCtrlYDrive.SetPID(Kp, Ki, Kd);
                break;

            case VARID_DRIVE_KI:
                m_pidCtrlYDrive.GetPID(&Kp, &Ki, &Kd);
                Ki = *(double *)varData;
                m_pidCtrlYDrive.SetPID(Kp, Ki, Kd);
                break;

            case VARID_DRIVE_KD:
                m_pidCtrlYDrive.GetPID(&Kp, &Ki, &Kd);
                Kd = *(double *)varData;
                m_pidCtrlYDrive.SetPID(Kp, Ki, Kd);
                break;

            case VARID_TURN_KP:
                m_pidCtrlTurn.GetPID(&Kp, &Ki, &Kd);
                Kp = *(double *)varData;
                m_pidCtrlTurn.SetPID(Kp, Ki, Kd);
                break;

            case VARID_TURN_KI:
                m_pidCtrlTurn.GetPID(&Kp, &Ki, &Kd);
                Ki = *(double *)varData;
                m_pidCtrlTurn.SetPID(Kp, Ki, Kd);
                break;

            case VARID_TURN_KD:
                m_pidCtrlTurn.GetPID(&Kp, &Ki, &Kd);
                Kd = *(double *)varData;
                m_pidCtrlTurn.SetPID(Kp, Ki, Kd);
                break;

            default:
                printf("Error: invalid variable ID (var=%s,ID=%d).\n",
                       varEntry->varName, varEntry->varID);
                rc = ERR_ASSERT;
        }

        TExitMsg(("=%d", rc));
        return rc;
    }   //SetVariable

#ifdef _LOGDATA_DRIVEBASE
    /**
     * This function is called by the TaskMgr to update the data points.
     *
     * @param flags Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        m_heading = m_gyro.GetAngle();
        GetPosition(&m_xPos, &m_yPos, &m_rotPos);

        TExit();
        return;
    }   //TaskPostPeriodic
#endif

};  //class BaseDrive

VAR_ENTRY DriveBase::m_varTable[] =
{
    {"DrivePID", VARID_DRIVE_PID, VarDouble, NULL, 0, NULL,
     "Get Drive PID constants"},
    {"DriveKp",  VARID_DRIVE_KP,  VarDouble, NULL, 0, NULL,
     "Get/Set Drive Kp constant"},
    {"DriveKi",  VARID_DRIVE_KI,  VarDouble, NULL, 0, NULL,
     "Get/Set Drive Ki constant"},
    {"DriveKd",  VARID_DRIVE_KD,  VarDouble, NULL, 0, NULL,
     "Get/Set Drive Kd constant"},
    {"TurnPID",  VARID_TURN_PID,  VarDouble, NULL, 0, NULL,
     "Get Turn PID constants"},
    {"TurnKp",   VARID_TURN_KP,   VarDouble, NULL, 0, NULL,
     "Get/Set Turn Kp constant"},
    {"TurnKi",   VARID_TURN_KI,   VarDouble, NULL, 0, NULL,
     "Get/Set Turn Ki constant"},
    {"TurnKd",   VARID_TURN_KD,   VarDouble, NULL, 0, NULL,
     "Get/Set Turn Kd constant"},
    {NULL,       0,               VarNone,   NULL, 0, NULL, NULL}
};

#endif  //ifndef _DRIVEBASE_H
