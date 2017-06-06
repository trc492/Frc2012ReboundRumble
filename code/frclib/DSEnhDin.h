#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DSEnhDin.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     DSEnhDin class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DSENHDIN_H
#define _DSENHDIN_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DSENHDIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "DSEnhDin"

#define MAX_EIODIN_CHANNELS     16

/**
 * This abstract class defines the EIODinNotify object. The object is
 * a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to be notified
 * on the Driver Station Enhanced IO digital input events.
 */
class EIODinNotify
{
public:
    /**
     * This function is provided by the subclass to handle a Driver Station
     * Enhanced IO Digital Input Event notification.
     *
     * @param channel Specifies the Enhanced IO Digital Input channel that
     *        generated the event.
     * @param fActive If true, specifies the Digital Input channel is active,
     *        false otherwise.
     */
    virtual
    void
    NotifyEIODin(
        UINT32 channel,
        bool   fActive
        ) = 0;
};  //class EIODinNotify

/**
 * This class defines and implements the DSEnhDin object. This object provides
 * the support of detecting EnhancedIO Digital Input events and calling the
 * notification object. It also added the support for using some Enhanced IO
 * channels for BCD switches.
 */
class DSEnhDin: public CoopTask
{
private:
    DriverStationEnhancedIO& m_enhancedIO;
    UINT16          m_notifyMask[MAX_EIODIN_CHANNELS];
    EIODinNotify   *m_notify[MAX_EIODIN_CHANNELS];
    int             m_numNotifies;
    UINT16          m_channelMask;
    UINT16          m_prevEIODin;

public:
    /**
     * Constructor: Create an instance of the object.
     */
    DSEnhDin(
        void
        ): m_enhancedIO(DriverStation::GetInstance()->GetEnhancedIO())
    {
        TLevel(INIT);
        TEnter();

        for (int i = 0; i < MAX_EIODIN_CHANNELS; i++)
        {
            m_notifyMask[i] = 0;
            m_notify[i] = NULL;
        }
        m_numNotifies = 0;
        m_channelMask = 0;
        m_prevEIODin = m_enhancedIO.GetDigitals();

#ifdef _LOGDATA_DSENHDIN
        DataLogger *dataLogger = DataLogger::GetInstance();
        dataLogger->AddDataPoint(MOD_NAME, "", "EIODin", "0x%04x",
                                 DataInt16, &m_prevEIODin);

#endif

        RegisterTask(MOD_NAME, TASK_PRE_PERIODIC);

        TExit();
    }   //DSEnhDin

    /**
     * Destructor: Destroy an instance of the object.
     */
    virtual
    ~DSEnhDin(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        UnregisterTask();

        TExit();
    }   //~DSEnhDin

    /**
     * This function registers a notification handler for the specified
     * digital input channels.
     *
     * @param notifyMask Specifies the channel mask for the notification.
     * @param notify Points to the EIODinNotify object for the notification
     *        callback.
     */
    bool
    RegisterNotification(
        UINT16        notifyMask,
        EIODinNotify *notify
        )
    {
        bool fSuccess = false;

        TLevel(API);
        TEnterMsg(("mask=%x,notify=%p", notifyMask, notify));

        if (m_numNotifies < MAX_EIODIN_CHANNELS)
        {
            m_channelMask |= notifyMask;
            m_notifyMask[m_numNotifies] = notifyMask;
            m_notify[m_numNotifies] = notify;
            m_numNotifies++;
            fSuccess = true;
        }

        TExitMsg(("=%d", fSuccess));
        return fSuccess;
    }   //RegisterNotification

    /**
     * This function returns the BCD Switch state.
     *
     * @param lowChannel Specifies the low digital input channel that the
     *        BCD switch is connected to.
     *
     * @return Returns the state of the BCD switch.
     */
    UINT8
    GetBCDSwitch(
        UINT32 lowChannel
        )
    {
        UINT8 bcdSwitch;

        TLevel(API);
        TEnterMsg(("lowChannel=%d", lowChannel));

        bcdSwitch = (~m_enhancedIO.GetDigitals() >> lowChannel) & 0x0f;

        TExitMsg(("=%x", bcdSwitch));
        return bcdSwitch;
    }   //GetBCDSwitch

    /**
     * This function is called by the TaskMgr to check and process the
     * Digital Input events.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskPrePeriodic(
        UINT32 mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        UINT16 currEIODin = m_enhancedIO.GetDigitals();
        if (m_numNotifies > 0)
        {
            UINT16 changedDin = (m_prevEIODin^currEIODin) & m_channelMask;
            UINT16 maskDin;
            UINT32 channel;

            while (changedDin != 0)
            {
                //
                // maskDin contains the least significant set bit.
                //
                maskDin = changedDin & ~(changedDin^-changedDin);

                for (channel = 1; channel <= MAX_EIODIN_CHANNELS; channel++)
                {
                    if (maskDin == (1 << (channel - 1)))
                    {
                        break;
                    }
                }

                for (int idx = 0; idx < m_numNotifies; idx++)
                {
                    if (maskDin & m_notifyMask[idx])
                    {
                        m_notify[idx]->NotifyEIODin(
                                channel,
                                (currEIODin & maskDin) != 0);
                    }
                }

                //
                // Clear the least significant set bit.
                //
                changedDin &= ~maskDin;
            }
        }
        m_prevEIODin = currEIODin;

        TExit();
    }   //TaskPrePeriodic

};  //class DSEnhDin

#endif  //ifndef _DSENHDIN_H
