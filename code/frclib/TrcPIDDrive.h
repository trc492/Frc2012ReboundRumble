#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDDrive.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDDrive class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDDRIVE_H
#define _TRCPIDDRIVE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDDRIVE
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDDrive"

//
// PIDDrive options
//
#define PIDDRIVEO_MECANUM_DRIVE 0x00000001

/**
 * This class defines and implements the TrcPIDDrive object. It supports the
 * capability of PID controlled drive by X and Y distance and/or PID controlled
 * turn by angle so that it can work with the mecanum drive train. After the
 * operation is completed, it has an option of notifying the caller by setting
 * an event. Therefore, it can work with the state machine to do autonomous
 * drive.
 * The PID Drive object consists of a drive object, 3 PID controllers (one
 * for X, one for Y and one for rotation) and a PID input object to provide
 * feedback.
 */
class TrcPIDDrive: public CoopTask
{
private:
    //
    // Flags
    //
    #define PIDDRIVEF_PIDDRIVE_ON       0x00000001
    #define PIDDRIVEF_STOP_ONTARGET     0x00000002
    #define PIDDRIVEF_TURN_ONLY         0x00000004
    #define PIDDRIVEF_MANUAL_DRIVE      0x00000008

    RobotDrive *m_drive;
    TrcPIDCtrl *m_pidCtrlXDrive;
    TrcPIDCtrl *m_pidCtrlYDrive;
    TrcPIDCtrl *m_pidCtrlTurn;
    PIDInput   *m_pidInput;
    UINT32      m_pidDriveOptions;
    UINT32      m_pidDriveFlags;
    Event      *m_notifyEvent;
    UINT32      m_expiredTime;
    float       m_xPower;
    float       m_yPower;

public:
    /**
     * This function stops the PID Drive object.
     */
    void
    Stop(
        void
        )
    {
        TLevel(API);
        TEnter();

        m_drive->StopMotor();
        m_pidCtrlYDrive->Reset();
        m_pidCtrlTurn->Reset();
        m_pidDriveFlags = 0;
        if (((m_pidDriveOptions & PIDDRIVEO_MECANUM_DRIVE) != 0) &&
            (m_pidCtrlXDrive != NULL))
        {
            m_pidCtrlXDrive->Reset();
        }

        TExit();
        return;
    }   //Stop

    /**
     * This function is called by TaskMgr to stop the PID drive task.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskStopMode(
        UINT32 mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        Stop();

        TExit();
        return;
    }   //TaskStopMode

    /**
     * Constructor: Create an instance of the TrcPIDDrive object that consists
     * of a RobotDrive object, a TrcPIDCtrl object for drive, a TrcPIDCtrl
     * object for turn and optionally a notification object for the callback.
     *
     * @param drive Points to the RobotDrive object.
     * @param pidCtrlXDrive Points to the TrcPIDCtrl object for driving
     *        sideway (only use if drive train is mecanum).
     * @param pidCtrlYDrive Points to the TrcPIDCtrl object for driving
     *        forward and backward.
     * @param pidCtrlTurn Points to the TrcPIDCtrl object for turning.
     * @param pidInput Specifies the PIDInput object.
     * @param pidDriveOptions Specifies PIDDrive options.
     */
    TrcPIDDrive(
        RobotDrive *drive,
        TrcPIDCtrl *pidCtrlXDrive,
        TrcPIDCtrl *pidCtrlYDrive,
        TrcPIDCtrl *pidCtrlTurn,
        PIDInput   *pidInput,
        UINT32      pidDriveOptions = 0
        ): m_drive(drive)
         , m_pidCtrlXDrive(NULL)
         , m_pidCtrlYDrive(pidCtrlYDrive)
         , m_pidCtrlTurn(pidCtrlTurn)
         , m_pidInput(pidInput)
         , m_pidDriveOptions(pidDriveOptions)
         , m_pidDriveFlags(0)
         , m_notifyEvent(NULL)
         , m_expiredTime(0)
         , m_xPower(0.0)
         , m_yPower(0.0)
    {
        TLevel(INIT);
        TEnterMsg(("drive=%p,pidCtrlXDrive=%p,pidCtrlYDrive=%p,pidCtrlTurn=%p,"
                   "pidInput=%p,options=%x",
                   drive, pidCtrlXDrive, pidCtrlYDrive, pidCtrlTurn, pidInput,
                   pidDriveOptions));

        if (m_pidDriveOptions & PIDDRIVEO_MECANUM_DRIVE)
        {
            //
            // pidCtrlXDrive is only valid if we have mecanum drive.
            // pidCtrlXDrive is optional, so it could be NULL.
            //
            m_pidCtrlXDrive = pidCtrlXDrive;
        }

        RegisterTask(MOD_NAME, TASK_STOP_MODE | TASK_POST_PERIODIC);

        TExit();
    }   //TrcPIDDrive

    /**
     * Destructor: Destroy an instance of the TrcPIDDrive object.
     */
    virtual
    ~TrcPIDDrive(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        Stop();
        UnregisterTask();

        TExit();
    }   //~TrcPIDDrive

    /**
     * This function sets PID drive target with the given drive distance and
     * turn angle setpoints.
     *
     * @param distXSetPoint Specifies the target distance relative to current
     *        distance (only use if drive train is mecanum).
     * @param distYSetPoint Specifies the target distance relative to current
     *        distance.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     * @param fStopOnTarget If true, stop PIDDrive when target is reached.
     *        Otherwise, continue to monitor the target and readjust if
     *        necessary.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    SetTarget(
        float  distXSetPoint,
        float  distYSetPoint,
        float  angleSetPoint,
        bool   fStopOnTarget = true,
        Event *notifyEvent = NULL,
        UINT32 timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("distXSetPt=%f,distYSetPt=%f,angleSetPt=%f,fStopOnTarget=%x,"
                   "event=%p,timeout=%d",
                   distXSetPoint, distYSetPoint, angleSetPoint, fStopOnTarget,
                   notifyEvent, timeout));

        if (m_pidCtrlXDrive != NULL)
        {
            m_pidCtrlXDrive->SetTarget(distXSetPoint,
                                       m_pidInput->GetInput(m_pidCtrlXDrive));

        }
        m_pidCtrlYDrive->SetTarget(distYSetPoint,
                                   m_pidInput->GetInput(m_pidCtrlYDrive));
        m_pidCtrlTurn->SetTarget(angleSetPoint,
                                 m_pidInput->GetInput(m_pidCtrlTurn));
        m_notifyEvent = notifyEvent;
        m_expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;

        m_pidDriveFlags = PIDDRIVEF_PIDDRIVE_ON;
        if (fStopOnTarget)
        {
            m_pidDriveFlags |= PIDDRIVEF_STOP_ONTARGET;
        }

        if ((distXSetPoint == 0.0) && (distYSetPoint == 0.0) &&
            (angleSetPoint != 0.0))
        {
            m_pidDriveFlags |= PIDDRIVEF_TURN_ONLY;
        }
        else
        {
            m_pidDriveFlags &= ~PIDDRIVEF_TURN_ONLY;
        }

        TExit();
        return;
    }   //SetTarget

    /**
     * This function sets PID drive angle target with specifies X and Y drive
     * powers. In other words, it allows manual drive with automatic PID
     * controlled heading. This only make sense for Mecanum drive train. If it
     * is not mecanum, this function does nothing.
     *
     * @param xPower Specifies the drive power of the X direction.
     * @param yPower Specifies the drive power of the Y direction.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     */
    void
    SetAngleTarget(
        float xPower,
        float yPower,
        float angleSetPoint
        )
    {
        TLevel(API);
        TEnterMsg(("xPower=%f,yPower=%f,angleSetPt=%f",
                   xPower, yPower, angleSetPoint));

        if (m_pidDriveOptions & PIDDRIVEO_MECANUM_DRIVE)
        {
            m_xPower = xPower;
            m_yPower = yPower;
            m_pidCtrlTurn->SetTarget(angleSetPoint,
                                     m_pidInput->GetInput(m_pidCtrlTurn));
            m_pidDriveFlags = PIDDRIVEF_PIDDRIVE_ON |
                              PIDDRIVEF_MANUAL_DRIVE;
        }

        TExit();
        return;
    }   //SetAngleTarget

    /**
     * This function is called by the TaskMgr to update the PIDDrive state
     * and check for completion.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        if (m_pidDriveFlags & PIDDRIVEF_PIDDRIVE_ON)
        {
            if (m_pidDriveFlags & PIDDRIVEF_MANUAL_DRIVE)
            {
                float turnPower = m_pidCtrlTurn->CalcPIDOutput(
                                    m_pidInput->GetInput(m_pidCtrlTurn));

                m_drive->MecanumDrive_Polar(MAGNITUDE(m_xPower, m_yPower),
                                            DIR_DEGREES(m_xPower, m_yPower),
                                            turnPower);
            }
            else if ((m_expiredTime != 0) &&
                     (GetMsecTime() >= m_expiredTime) ||
                     m_pidCtrlTurn->OnTarget() &&
                     ((m_pidDriveFlags & PIDDRIVEF_TURN_ONLY) ||
                      (m_pidCtrlYDrive->OnTarget()) &&
                      ((m_pidCtrlXDrive == NULL) ||
                       (m_pidCtrlXDrive->OnTarget()))))
            {
                if (m_pidDriveFlags & PIDDRIVEF_STOP_ONTARGET)
                {
                    Stop();
                    if (m_notifyEvent != NULL)
                    {
                        m_notifyEvent->SetEvent();
                    }
                }
                else if (m_pidDriveOptions & PIDDRIVEO_MECANUM_DRIVE)
                {
                    m_drive->MecanumDrive_Polar(0.0, 0.0, 0.0);
                }
                else
                {
                    m_drive->Drive(0.0, 0.0);
                }
            }
            else if (m_pidDriveOptions & PIDDRIVEO_MECANUM_DRIVE)
            {
                float xPower = (m_pidCtrlXDrive != NULL)?
                                    m_pidCtrlXDrive->CalcPIDOutput(
                                        m_pidInput->GetInput(m_pidCtrlXDrive)):
                                    0.0;
                float yPower = m_pidCtrlYDrive->CalcPIDOutput(
                                    m_pidInput->GetInput(m_pidCtrlYDrive));
                float turnPower = m_pidCtrlTurn->CalcPIDOutput(
                                    m_pidInput->GetInput(m_pidCtrlTurn));

                m_drive->MecanumDrive_Polar(MAGNITUDE(xPower, yPower),
                                            DIR_DEGREES(xPower, yPower),
                                            turnPower);
            }
            else
            {
                float y_input = m_pidInput->GetInput(m_pidCtrlYDrive); 
                float y = m_pidCtrlYDrive->CalcPIDOutput(y_input);
                float rot_input = m_pidInput->GetInput(m_pidCtrlTurn);
                float rot = m_pidCtrlTurn->CalcPIDOutput(rot_input);
                m_drive->ArcadeDrive(y, rot);
            }
        }

        TExit();
        return;
    }   //TaskPostPeriodic

};  //class TrcPIDDrive

#endif  //ifndef _TRCPIDDRIVE_H
