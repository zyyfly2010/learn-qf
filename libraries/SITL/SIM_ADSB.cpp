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
  ADSB simulator class for MAVLink ADSB peripheral
*/

#include "SIM_ADSB.h"

#include <stdio.h>

#include "SIM_Aircraft.h"

extern const AP_HAL::HAL& hal;

namespace SITL {

ADSB::ADSB(const struct sitl_fdm &_fdm) : fdm(_fdm)
{
}


/*
  update the ADSB peripheral state
*/
void ADSB::update(void)
{
    // TODO: add update of simulated ADSB vehicle positions

    // see if we should do a report
    send_report();
}

/*
  send a report to the vehicle control code over MAVLink
*/
void ADSB::send_report(void)
{
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("ADSB connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        return;
    }

    // check for incoming MAVLink messages
    uint8_t buf[100];
    ssize_t ret;

    while ((ret=mav_socket.recv(buf, sizeof(buf), 0)) > 0) {
        for (uint8_t i=0; i<ret; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                                          buf[i],
                                          &msg, &status) == MAVLINK_FRAMING_OK) {
                switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    if (!seen_heartbeat) {
                        seen_heartbeat = true;
                        vehicle_component_id = msg.compid;
                        vehicle_system_id = msg.sysid;
                        ::printf("ADSB using srcSystem %u\n", (unsigned)vehicle_system_id);
                    }
                    break;
                }
                }
            }
        }
    }

    if (!seen_heartbeat) {
        return;
    }

    uint32_t now = hal.scheduler->millis();
    mavlink_message_t msg;
    uint16_t len;

    if (now - last_heartbeat_ms >= 1000) {
        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_ADSB;
        heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        heartbeat.base_mode = 0;
        heartbeat.system_status = 0;
        heartbeat.mavlink_version = 0;
        heartbeat.custom_mode = 0;

        /*
          save and restore sequence number for chan0, as it is used by
          generated encode functions
         */
        mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
        uint8_t saved_seq = chan0_status->current_tx_seq;
        chan0_status->current_tx_seq = mavlink.seq;
        len = mavlink_msg_heartbeat_encode(vehicle_system_id,
                                           vehicle_component_id,
                                           &msg, &heartbeat);
        chan0_status->current_tx_seq = saved_seq;

        mav_socket.send(&msg.magic, len);

        last_heartbeat_ms = now;
    }


    /*
      send a ADSB_VEHICLE messages
     */
    uint32_t now_us = hal.scheduler->micros();
    if (now_us - last_report_us > reporting_period_ms*1000UL) {
        mavlink_adsb_vehicle_t adsb_vehicle {};
        last_report_us = now_us;

        adsb_vehicle.ICAO_address = 1234;
        adsb_vehicle.lat = -35;
        adsb_vehicle.lon = 149;
        adsb_vehicle.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
        adsb_vehicle.altitude = 800;
        adsb_vehicle.heading = 120;
        adsb_vehicle.hor_velocity = 30;
        adsb_vehicle.ver_velocity = 1;
        snprintf(adsb_vehicle.callsign, sizeof(adsb_vehicle.callsign), "SITL1234");
        adsb_vehicle.emitterType = ADSB_EMITTER_TYPE_LARGE;
        adsb_vehicle.tslc = 1;
        adsb_vehicle.validFlags =
            ADSB_DATA_VALID_FLAGS_VALID_COORDS |
            ADSB_DATA_VALID_FLAGS_VALID_ALTITUDE |
            ADSB_DATA_VALID_FLAGS_VALID_HEADING |
            ADSB_DATA_VALID_FLAGS_VALID_VELOCITY |
            ADSB_DATA_VALID_FLAGS_VALID_CALLSIGN;

        mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
        uint8_t saved_seq = chan0_status->current_tx_seq;
        chan0_status->current_tx_seq = mavlink.seq;
        len = mavlink_msg_adsb_vehicle_encode(vehicle_system_id,
                                              MAV_COMP_ID_ADSB,
                                              &msg, &adsb_vehicle);
        chan0_status->current_tx_seq = saved_seq;

        mav_socket.send(&msg.magic, len);
    }
    
}

} // namespace SITL
