/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connection for ardupilot version of FlightAxis
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  a FlightAxis simulator
 */
class FlightAxis : public Aircraft {
public:
    FlightAxis(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new FlightAxis(home_str, frame_str);
    }

    struct state {
        float m_airspeed_MPS;
        float m_altitudeASL_MTR;
        float m_altitudeAGL_MTR;
        float m_groundspeed_MPS;
        float m_pitchRate_DEGpSEC;
        float m_rollRate_DEGpSEC;
        float m_yawRate_DEGpSEC;
        float m_azimuth_DEG;
        float m_inclination_DEG;
        float m_roll_DEG;
        float m_aircraftPositionX_MTR;
        float m_aircraftPositionY_MTR;
        float m_velocityWorldU_MPS;
        float m_velocityWorldV_MPS;
        float m_velocityWorldW_MPS;
        float m_velocityBodyU_MPS;
        float m_velocityBodyV_MPS;
        float m_velocityBodyW_MPS;
        float m_accelerationWorldAX_MPS2;
        float m_accelerationWorldAY_MPS2;
        float m_accelerationWorldAZ_MPS2;
        float m_accelerationBodyAX_MPS2;
        float m_accelerationBodyAY_MPS2;
        float m_accelerationBodyAZ_MPS2;
        float m_windX_MPS;
        float m_windY_MPS;
        float m_windZ_MPS;
        float m_propRPM;
        float m_heliMainRotorRPM;
        float m_batteryVoltage_VOLTS;
        float m_batteryCurrentDraw_AMPS;
        float m_batteryRemainingCapacity_MAH;
        float m_fuelRemaining_OZ;
        float m_isLocked;
        float m_hasLostComponents;
        float m_anEngineIsRunning;
        float m_isTouchingGround;
        float m_currentAircraftStatus;
    } state;

    static const uint16_t num_keys = sizeof(state)/sizeof(float);
    
    struct keytable {
        const char *key;
        float &ref;
    } keytable[num_keys] = {
        { "m-airspeed-MPS", state.m_airspeed_MPS },
        { "m-altitudeASL-MTR", state.m_altitudeASL_MTR },
        { "m-altitudeAGL-MTR", state.m_altitudeAGL_MTR },
        { "m-groundspeed-MPS", state.m_groundspeed_MPS },
        { "m-pitchRate-DEGpSEC", state.m_pitchRate_DEGpSEC },
        { "m-rollRate-DEGpSEC", state.m_rollRate_DEGpSEC },
        { "m-yawRate-DEGpSEC", state.m_yawRate_DEGpSEC },
        { "m-azimuth-DEG", state.m_azimuth_DEG },
        { "m-inclination-DEG", state.m_inclination_DEG },
        { "m-roll-DEG", state.m_roll_DEG },
        { "m-aircraftPositionX-MTR", state.m_aircraftPositionX_MTR },
        { "m-aircraftPositionY-MTR", state.m_aircraftPositionY_MTR },
        { "m-velocityWorldU-MPS", state.m_velocityWorldU_MPS },
        { "m-velocityWorldV-MPS", state.m_velocityWorldV_MPS },
        { "m-velocityWorldW-MPS", state.m_velocityWorldW_MPS },
        { "m-velocityBodyU-MPS", state.m_velocityBodyU_MPS },
        { "m-velocityBodyV-MPS", state.m_velocityBodyV_MPS },
        { "m-velocityBodyW-MPS", state.m_velocityBodyW_MPS },
        { "m-accelerationWorldAX-MPS2", state.m_accelerationWorldAX_MPS2 },
        { "m-accelerationWorldAY-MPS2", state.m_accelerationWorldAY_MPS2 },
        { "m-accelerationWorldAZ-MPS2", state.m_accelerationWorldAZ_MPS2 },
        { "m-accelerationBodyAX-MPS2", state.m_accelerationBodyAX_MPS2 },
        { "m-accelerationBodyAY-MPS2", state.m_accelerationBodyAY_MPS2 },
        { "m-accelerationBodyAZ-MPS2", state.m_accelerationBodyAZ_MPS2 },
        { "m-windX-MPS", state.m_windX_MPS },
        { "m-windY-MPS", state.m_windY_MPS },
        { "m-windZ-MPS", state.m_windZ_MPS },
        { "m-propRPM", state.m_propRPM },
        { "m-heliMainRotorRPM", state.m_heliMainRotorRPM },
        { "m-batteryVoltage-VOLTS", state.m_batteryVoltage_VOLTS },
        { "m-batteryCurrentDraw-AMPS", state.m_batteryCurrentDraw_AMPS },
        { "m-batteryRemainingCapacity-MAH", state.m_batteryRemainingCapacity_MAH },
        { "m-fuelRemaining-OZ", state.m_fuelRemaining_OZ },
        { "m-isLocked", state.m_isLocked },
        { "m-hasLostComponents", state.m_hasLostComponents },
        { "m-anEngineIsRunning", state.m_anEngineIsRunning },
        { "m-isTouchingGround", state.m_isTouchingGround },
        { "m-currentAircraftStatus", state.m_currentAircraftStatus }
    };
    
private:
    char *soap_request(const char *action, const char *fmt, ...);
    void exchange_data(const struct sitl_input &input);
    void parse_reply(const char *reply);

    uint64_t start_time_us;
    bool controller_started = false;
};


} // namespace SITL
