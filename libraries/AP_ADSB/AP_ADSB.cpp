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
    AP_ADSB.cpp

    ADS-B RF based collision avoidance module
    https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast
*/

#include <AP_HAL/AP_HAL.h>
#include "AP_ADSB.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_ADSB::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable ADS-B
    // @User: Advanced
    AP_GROUPINFO("ENABLE",     0, AP_ADSB, _enable,    0),

    // @Param: BEHAVIOR
    // @DisplayName: Collision Avoidance Behavior
    // @Description: Collision Avoidance Behavior selector.
    // @User: Advanced
    AP_GROUPINFO("BEHAVIOR",   1, AP_ADSB, _behavior, BEHAVIOR_NONE),

    AP_GROUPEND
};

bool AP_ADSB::set_vehicle(uint16_t index, mavlink_adsb_vehicle_t vehicle)
{
    if (index >= VEHCILE_LIST_LENGTH) {
        return false;
    }
    memcpy(vehicle_list[index], vehicle, sizeof(vehicle));
}

void AP_ADSB::send_vehicle(mavlink_channel_t chan)
{
uint8_t cal_mask = get_cal_mask();

for (uint8_t compass_id=0; compass_id<COMPASS_MAX_INSTANCES; compass_id++) {
    uint8_t cal_status = _calibrator[compass_id].get_status();

    if (cal_status == COMPASS_CAL_WAITING_TO_START  ||
        cal_status == COMPASS_CAL_RUNNING_STEP_ONE ||
        cal_status == COMPASS_CAL_RUNNING_STEP_TWO) {
        uint8_t completion_pct = _calibrator[compass_id].get_completion_percent();
        uint8_t completion_mask[10];
        Vector3f direction(0.0f,0.0f,0.0f);
        uint8_t attempt = _calibrator[compass_id].get_attempt();

        memset(completion_mask, 0, sizeof(completion_mask));

        // ensure we don't try to send with no space available
        if (!HAVE_PAYLOAD_SPACE(chan, MAG_CAL_PROGRESS)) {
            return;
        }

        mavlink_msg_adsb_vehicle_send(
            chan,

        );
    }
}
}

