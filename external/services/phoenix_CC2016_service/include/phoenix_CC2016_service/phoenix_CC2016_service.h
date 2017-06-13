#ifndef PHOENIX_CC2016_SERVICE_H
#define PHOENIX_CC2016_SERVICE_H

#include <lms/service.h>
#include <lms/time.h>
namespace phoenix_CC2016_service {

enum class RemoteControlState{
    UNKOWN,DISCONNECTED,REMOTE_CONTROL_STATUS_SEMI_AUTONOMOUS,REMOTE_CONTROL_STATUS_AUTONOMOUS,REMOTE_CONTROL_STATUS_MANUAL

};
enum class CCDriveMode{
    IDLE,PARKING, FOH, FMH
};

/**
 * @brief LMS service phoenix_CC2016_service
 **/
class Phoenix_CC2016Service : public lms::Service {
    RemoteControlState m_state;
    RemoteControlState m_oldState;
    CCDriveMode m_driveMode;
    CCDriveMode m_last_driveMode;
    /**
     * @brief batteryVolate in mv
     */
    int m_batteryVoltage;
    lms::Time m_lastUpdate;

public:
    void update(RemoteControlState rcState, CCDriveMode driveMode, int batteryVoltage);
    void updateRcState(RemoteControlState state);
    bool rcStateChanged() const;
    bool driveModeChanged() const;
    RemoteControlState rcState() const;
    CCDriveMode driveMode() const;
    int batteryVoltage() const;
    lms::Time lastUpdate() const;
    void updateFromConfig();

    void logRcStates();
    /**
     * @brief isValid
     * @return true if the lastUpdate was in time
     */
    bool isValid() const;
    //system-methods
    bool init() override;
    void destroy() override;
};

std::ostream& operator << (std::ostream& os, RemoteControlState state);

} // namespace phoenix_CC2016_service

#endif // PHOENIX_CC2016_SERVICE_H
