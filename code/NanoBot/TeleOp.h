#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="teleop.h" />
///
/// <summary>
///     This main module contains the teleoperated mode code.
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

#define ON_TIME                 0.5
#define LEFT_COLOR_BLACK        (UINT8)0
#define LEFT_COLOR_RED          SolID(SOL_LEFTLIGHT_RED)
#define LEFT_COLOR_GREEN        SolID(SOL_LEFTLIGHT_GREEN)
#define LEFT_COLOR_BLUE         SolID(SOL_LEFTLIGHT_BLUE)
#define LEFT_COLOR_CYAN         (UINT8)(LEFT_COLOR_GREEN | LEFT_COLOR_BLUE)
#define LEFT_COLOR_MAGENTA      (UINT8)(LEFT_COLOR_RED | LEFT_COLOR_BLUE)
#define LEFT_COLOR_YELLOW       (UINT8)(LEFT_COLOR_RED | LEFT_COLOR_GREEN)
#define LEFT_COLOR_WHITE        (UINT8)(LEFT_COLOR_YELLOW | LEFT_COLOR_BLUE)
#define RIGHT_COLOR_BLACK       (UINT8)0
#define RIGHT_COLOR_RED         SolID(SOL_RIGHTLIGHT_RED)
#define RIGHT_COLOR_GREEN       SolID(SOL_RIGHTLIGHT_GREEN)
#define RIGHT_COLOR_BLUE        SolID(SOL_RIGHTLIGHT_BLUE)
#define RIGHT_COLOR_CYAN        (UINT8)(RIGHT_COLOR_GREEN | RIGHT_COLOR_BLUE)
#define RIGHT_COLOR_MAGENTA     (UINT8)(RIGHT_COLOR_RED | RIGHT_COLOR_BLUE)
#define RIGHT_COLOR_YELLOW      (UINT8)(RIGHT_COLOR_RED | RIGHT_COLOR_GREEN)
#define RIGHT_COLOR_WHITE       (UINT8)(RIGHT_COLOR_YELLOW | RIGHT_COLOR_BLUE)

static SOL_STATE leftLightStates[] =
{
    {LEFT_COLOR_BLACK, ON_TIME},
    {LEFT_COLOR_BLUE, ON_TIME},
    {LEFT_COLOR_GREEN, ON_TIME},
    {LEFT_COLOR_CYAN, ON_TIME},
    {LEFT_COLOR_RED, ON_TIME},
    {LEFT_COLOR_MAGENTA, ON_TIME},
    {LEFT_COLOR_YELLOW, ON_TIME},
    {LEFT_COLOR_WHITE, ON_TIME}
};

static SOL_STATE rightLightStates[] =
{
    {RIGHT_COLOR_BLACK, ON_TIME},
    {RIGHT_COLOR_BLUE, ON_TIME},
    {RIGHT_COLOR_GREEN, ON_TIME},
    {RIGHT_COLOR_CYAN, ON_TIME},
    {RIGHT_COLOR_RED, ON_TIME},
    {RIGHT_COLOR_MAGENTA, ON_TIME},
    {RIGHT_COLOR_YELLOW, ON_TIME},
    {RIGHT_COLOR_WHITE, ON_TIME}
};
    
/**
 * This function is called before starting the teleop mode.
 */
void
TrcRobot::TeleOpStart(
    void
    )
{
    TLevel(INIT);
    TEnter();
    //
    // Do other initializations for TeleOp mode here.
    //
#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Start();
#endif
    m_fUseMecanum = false;

    TExit();
}   //TeleOpStart

/**
 * This function is called before exiting the teleop mode.
 */
void
TrcRobot::TeleOpStop(
    void
    )
{
    TLevel(INIT);
    TEnter();
    //
    // Do clean up before exiting TeleOp mode here.
    //
#ifdef _ENABLE_DATALOGGER
    DataLogger::GetInstance()->Stop();
#endif

    TExit();
}   //TeleOpStop

/**
 * This function is called periodically at fixed intervals in teleop mode.
 */
void
TrcRobot::TeleOpPeriodic(
    void
    )
{
    float leftX, leftY, leftZ, rightX, rightY;
    float x, y, rotation; 

    TLevel(HIFREQ);
    TEnter();
    //
    // Process input subsystems here.
    // (e.g. Reading joysticks, buttons, switches and sensors etc and
    //       computing the actions).
    //
    leftX = m_driveJoystickLeft.GetXWithDeadband()*DRIVE_SCALE_FACTOR;
    leftY = -m_driveJoystickLeft.GetYWithDeadband()*DRIVE_SCALE_FACTOR;
    leftZ = m_driveJoystickLeft.GetZWithDeadband()*DRIVE_SCALE_FACTOR;
#ifdef _USE_DUAL_JOYSTICK
    rightX = m_driveJoystickRight.GetXWithDeadband()*DRIVE_SCALE_FACTOR;
    rightY = -m_driveJoystickRight.GetYWithDeadband()*DRIVE_SCALE_FACTOR;
#else
    rightX = m_driveJoystickLeft.GetZWithDeadband()*DRIVE_SCALE_FACTOR;
    rightY = -m_driveJoystickLeft.GetTwistWithDeadband()*DRIVE_SCALE_FACTOR;
#endif

#ifdef _USE_TANK_DRIVE
    //
    // We only do crabbing if both the left and right X axes are the same
    // sign. So to prevent accidental crabbing during tank drive, one would
    // either toe-in or toe-out the left and right joysticks while tank
    // driving.
    //
    x = ((leftX < 0.0) && (rightX < 0.0) ||
         (leftX > 0.0) && (rightX > 0.0))? (leftX + rightX)/2.0: 0.0;
    y = (leftY + rightY)/2.0;
    rotation = (leftY - rightY)/2.0;
#else   //ArcadeDrive
    //
    // We only rotate when we are not crabbing. If we are crabbing, we kept
    // the same heading by ignoring rotation.
    //
  #ifdef _USE_DUAL_JOYSTICKS
    x = rightX;
  #else
    x = leftZ;
  #endif
    rotation = (x == 0.0)? leftX: 0.0;
    y = leftY;
#endif

    //
    // Perform output functions here.
    // (e.g. Programming motors, pneumatics and solenoids etc)
    //
    if (m_fUseMecanum)
    {
//        m_driveBase.MecanumDrive_Cartesian(x, y, rotation);
        m_driveBase.MecanumDrive_Polar(MAGNITUDE(x, y),
                                       DIR_DEGREES(x, y),
                                       rotation);
    }
    else
    {
#ifdef _USE_TANK_DRIVE
        m_driveBase.TankDrive(leftY, -rightY);
#else
        m_driveBase.ArcadeDrive(y, rotation);
#endif
    }

#ifdef _DEBUG_TELEOP
    LCDPrintf((LCD_LINE2, "D=%5.2f,T=%5.2f", y, rotation));
    LCDPrintf((LCD_LINE3, "Crab=%5.2f", x));
  #if defined(_DEBUG_JOYSTICK) && defined(_USE_TANK_DRIVE)
    LCDPrintf((LCD_LINE4, "LX=%5.2f,RX=%5.2f", leftX, rightX));
    LCDPrintf((LCD_LINE5, "LY=%5.2f,RY=%5.2f", leftY, rightY));
  #else
    LCDPrintf((LCD_LINE4, "LF=%5.2f,RF=%5.2f",
               m_leftFrontMotor.GetPosition(),
               m_rightFrontMotor.GetPosition()));
    LCDPrintf((LCD_LINE5, "LR=%5.2f,RR=%5.2f",
               m_leftRearMotor.GetPosition(),
               m_rightRearMotor.GetPosition()));
  #endif
    LCDPrintf((LCD_LINE6, "UseMecanum=%d", m_fUseMecanum));
#endif
    //
    // send the dashbaord data associated with the I/O ports
    //
    m_dashboardDataFormat.SendIOPortData();

    TExit();
}   //TeleopPeriodic

/**
 * This function is called by the TrcJoystick object when a button event
 * occurred.
 *
 * @param port Specifies the joystick port.
 * @param maskButton Specifies the button mask.
 * @param fPressed If true, specifies the button is pressed, false if
 *        released.
 */
void
TrcRobot::NotifyButton(
    UINT32 port,
    UINT16 maskButton,
    bool   fPressed
    )
{
    TLevel(EVENT);
    TEnterMsg(("port=%d,Button=0x%x,fPressed=%d",
               port, maskButton, fPressed));

    if (port == JSPORT_DRIVE_LEFT)
    {
        switch (maskButton)
        {
        case DualAction_BtnA:
            break;

        case DualAction_BtnB:
            break;

        case DualAction_BtnX:
            if (fPressed)
            {
                m_fUseMecanum = !m_fUseMecanum;
                if (m_fUseMecanum)
                {
                    m_leftRGBLights.Set(leftLightStates, 8, SOLO_REPEAT);
                    m_rightRGBLights.Set(rightLightStates, 8, SOLO_REPEAT);
                }
                else
                {
                    m_leftRGBLights.Set(false, LEFT_COLOR_WHITE);
                    m_rightRGBLights.Set(false, RIGHT_COLOR_WHITE);
                }
            }
            break;

        case DualAction_BtnY:
            break;
        }
    }
#ifdef _USE_DUAL_JOYSTICKS
    else if (port == JSPORT_DRIVE_RIGHT)
    {
        switch (maskButton)
        {
        }
    }
#endif

    TExit();
}	//NotifyButton
