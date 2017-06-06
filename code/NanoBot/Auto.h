#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="auto.h" />
///
/// <summary>
///     This main module contains the autonomous mode code.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Main"

/**
 * This function is called before starting the autonomous mode.
 */
void
TrcRobot::AutonomousStart(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Do other initializations for Autonomous mode here.
    //
#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Start();
#endif
    m_autoSM.Start();

    TExit();
}   //AutonomousStart

/**
 * This function is called before exiting the autonmous mode.
 */
void
TrcRobot::AutonomousStop(
    void
    )
{
    TLevel(INIT);
    TEnter();

    //
    // Do clean up before exiting Autonomous mode here.
    //
    m_autoSM.Stop();
#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Stop();
#endif

    TExit();
}   //AutonomousStop

/**
 * This function is called periodically at fixed intervals in autonomous
 * mode.
 */
void
TrcRobot::AutonomousPeriodic(
    void
    )
{
    TLevel(HIFREQ);
    TEnter();

    //
    // Process the state machine if it is ready.
    //
    if (m_autoSM.IsReady())
    {
        UINT32 currState = m_autoSM.GetCurrentState();

        LCDPrintf((LCD_LINE2, "AutoState=%d", currState - SMSTATE_STARTED));
        switch (currState)
        {
        case SMSTATE_STARTED:
            //
            // Go forward 6 ft.
            //
            m_driveBase.DriveSetTarget(0.0, 72.0, 0.0, true, &m_driveEvent);
            m_autoSM.WaitForSingleEvent(&m_driveEvent, currState + 1);
            break;

        case SMSTATE_STARTED + 1:
            //
            // Turn right 90-degree.
            //
            m_driveBase.DriveSetTarget(0.0, 0.0, 90.0, true, &m_driveEvent);
            m_autoSM.WaitForSingleEvent(&m_driveEvent, currState + 1);
            break;

        case SMSTATE_STARTED + 2:
            //
            // Go diagonal for 5 ft (X=3ft, Y=4ft)
            //
            m_driveBase.DriveSetTarget(3.0, 4.0, 0.0, true, &m_driveEvent);
            m_autoSM.WaitForSingleEvent(&m_driveEvent, currState + 1, 0);
            break;

        case SMSTATE_STARTED + 3:
            //
            // Stop drive.
            //
            m_driveBase.Stop();
            //
            // Let it fall through.
            //
        default:
            //
            // Stop state machine.
            //
            m_autoSM.Stop();
            break;
        }
    }

    //
    // send the dashbaord data associated with the I/O ports
    //
    m_dashboardDataFormat.SendIOPortData();

    TExit();
}   //AutonomousPeriodic
