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
  ADSB peripheral simulator class
*/

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

class ADSB {
public:
    ADSB(const struct sitl_fdm &_fdm);
    void update(void);

private:
    const struct sitl_fdm &fdm;
    const char *target_address = "127.0.0.1";
    const uint16_t target_port = 5762;

    // reporting period in ms
    const float reporting_period_ms = 100;
    uint32_t last_report_us = 0;
    
    uint32_t last_heartbeat_ms = 0;
    bool seen_heartbeat = false;
    uint8_t vehicle_system_id;
    uint8_t vehicle_component_id;

    SocketAPM mav_socket { false };
    struct {
        // socket to telem2 on aircraft
        bool connected;
        mavlink_message_t rxmsg;
        mavlink_status_t status;
        uint8_t seq;
    } mavlink {};

    void send_report(void);
};

}  // namespace SITL
