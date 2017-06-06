#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcRobot.h" />
///
/// <summary>
///     This main module contains the definitions and implementation of the
///     TrcRobot class.
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
 * This class defines and implements the main robot object. It inherits the
 * CoopMTRobot object which is similar to the IterativeRobot class from the
 * WPI library. It also inherits the ButtonNotify interface so that it can
 * provide the notification callback for joystick button events.
 */
class TrcRobot: public CoopMTRobot,
                public ButtonNotify     //providing the Button handler
{
private:
    DashboardDataFormat m_dashboardDataFormat;
    SolLight    m_leftRGBLights;
    SolLight    m_rightRGBLights;
    //
    // Input Subsystem.
    //
    TrcJoystick m_driveJoystickLeft;
#ifdef _USE_DUAL_JOYSTICKS
    TrcJoystick m_driveJoystickRight;
#endif
    DigitalIn   m_digitalIn;
    //
    // DriveBase Subsystem.
    //
    CanJag      m_leftFrontMotor;
    CanJag      m_leftRearMotor;
    CanJag      m_rightFrontMotor;
    CanJag      m_rightRearMotor;
    DriveBase   m_driveBase;
    Event       m_driveEvent;
#ifdef _USE_LINE_FOLLOWER
    //
    // Line follower.
    //
    Solenoid    m_lightSensorPower;
    Event       m_lineFollowEvent;
    int         m_lineFollowDir;
#endif
    //
    // Used by Autonomous.
    //
    StateMachine m_autoSM;
    //
    // Miscellaneous.
    //
    bool        m_fUseMecanum;

public:
    /**
     * Constructor for the TrcRobot class.
     * Create instances of all the components.
     */
    TrcRobot(
        void
        ): m_dashboardDataFormat()
         , m_leftRGBLights(SOL_LEFTLIGHT_RED,
                           SOL_LEFTLIGHT_GREEN,
                           SOL_LEFTLIGHT_BLUE)
         , m_rightRGBLights(SOL_RIGHTLIGHT_RED,
                            SOL_RIGHTLIGHT_GREEN,
                            SOL_RIGHTLIGHT_BLUE)
         , m_driveJoystickLeft(JSPORT_DRIVE_LEFT, this)
#ifdef _USE_DUAL_JOYSTICKS
         , m_driveJoystickRight(JSPORT_DRIVE_RIGHT, this)
#endif
         , m_digitalIn()
         , m_leftFrontMotor(CANID_LEFTFRONT_JAG, CANJaguar::kPercentVbus)
         , m_leftRearMotor(CANID_LEFTREAR_JAG, CANJaguar::kPercentVbus)
         , m_rightFrontMotor(CANID_RIGHTFRONT_JAG, CANJaguar::kPercentVbus)
         , m_rightRearMotor(CANID_RIGHTREAR_JAG, CANJaguar::kPercentVbus)
         , m_driveBase(&m_leftFrontMotor, &m_leftRearMotor,
                       &m_rightFrontMotor, &m_rightRearMotor)
         , m_driveEvent()
#ifdef _USE_LINE_FOLLOWER
         , m_lightSensorPower(SOL_LIGHT_SENSOR_POWER)
         , m_lineFollowEvent()
         , m_lineFollowDir(LNFOLLOW_DIR_STRAIGHT)
#endif
         , m_autoSM()
         , m_fUseMecanum(false)
    {
        TLevel(INIT);
        TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);
        TEnter();
        //
        // Initialize motor controllers.
        //
        m_leftFrontMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_leftRearMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_rightFrontMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_rightRearMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);

        m_leftFrontMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_leftRearMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_rightFrontMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_rightRearMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);

        m_leftFrontMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_leftRearMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_rightFrontMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_rightRearMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);

        m_leftFrontMotor.EnableControl();
        m_leftRearMotor.EnableControl();
        m_rightFrontMotor.EnableControl();
        m_rightRearMotor.EnableControl();

#ifdef _USE_LINE_FOLLOWER
        //
        // Turn on light sensor power.
        //
        m_lightSensorPower.Set(true);
#endif
        //
        // Set task loop period to 100msec.
        //
        SetPeriod(0.1);

        TExit();
    }   //TrcRobot

    /**
     * Destructor for the TrcRobot class.
     * Destroy instances of components that were created in the constructor.
     */
    ~TrcRobot(
        void
        )
    {
        TLevel(INIT);
        TEnter();
        TExit();
    }   //~TrcRobot

    //
    // The following functions are in auto.h
    //
    void
    AutonomousStart(
        void
        );
    
    void
    AutonomousStop(
        void
        );
    
    void
    AutonomousPeriodic(
        void
        );

    //
    // The following functions are in teleop.h
    //
    void
    TeleOpStart(
        void
        );
    
    void
    TeleOpStop(
        void
        );
    
    void
    TeleOpPeriodic(
        void
        );

    void
    NotifyButton(
        UINT32 port,
        UINT16 maskButton,
        bool   fPressed
        );
};  //class TrcRobot

