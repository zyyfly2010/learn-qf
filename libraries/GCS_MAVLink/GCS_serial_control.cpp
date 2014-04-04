// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  MAVLink SERIAL_CONTROL handling
 */

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


#include <AP_HAL.h>
#include <GCS.h>
#include <DataFlash.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

extern uint8_t mavlink_disable_mask;

/**
   handle a SERIAL_CONTROL message
 */
void GCS_MAVLINK::handle_serial_control(mavlink_message_t *msg)
{
    mavlink_serial_control_t packet;
    mavlink_msg_serial_control_decode(msg, &packet);

    AP_HAL::UARTDriver *port = NULL;
    uint8_t channel_mask;

    if (packet.flags & SERIAL_CONTROL_FLAG_REPLY) {
        // how did this packet get to us?
        return;
    }

    switch (packet.device) {
    case SERIAL_CONTROL_DEV_TELEM1:
        port = hal.uartC;
        channel_mask = (1U<<MAVLINK_COMM_1);
        break;
    case SERIAL_CONTROL_DEV_TELEM2:
        port = hal.uartD;
        channel_mask = (1U<<MAVLINK_COMM_2);
        break;
    default:
        // not supported yet
        return;
    }
    
    // lock or unlock exclusive access to this channel
    if (packet.flags & SERIAL_CONTROL_FLAG_EXCLUSIVE) {
        mavlink_disable_mask |= channel_mask;
    } else {
        mavlink_disable_mask &= ~channel_mask;        
    }

    // optionally change the baudrate
    if (packet.baudrate != 0) {
        port->begin(packet.baudrate);
    }

    // write the data
    if (packet.count != 0) {
        if ((packet.flags & SERIAL_CONTROL_FLAG_BLOCKING) == 0) {
            port->write(packet.data, packet.count);
        } else {
            const uint8_t *data = &packet.data[0];
            uint8_t count = packet.count;
            while (count > 0) {
                while (port->txspace() <= 0) {
                    hal.scheduler->delay(5);
                }
                uint8_t n = port->txspace();
                if (n > packet.count) {
                    n = packet.count;
                }
                port->write(data, n);                
                data += n;
                count -= n;
            }
        }
    }

    if ((packet.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
        // no response expected
        return;
    }

    uint8_t flags = packet.flags;

more_data:
    // sleep for the timeout
    while (packet.timeout != 0 && 
           port->available() < sizeof(packet.data)) {
        hal.scheduler->delay(1);
        packet.timeout--;
    }

    packet.flags = SERIAL_CONTROL_FLAG_REPLY;

    // work out how many bytes are available
    int16_t available = port->available();
    if (available < 0) {
        available = 0;
    }
    if (available > (int16_t)sizeof(packet.data)) {
        available = sizeof(packet.data);
    }

    // read any reply data
    packet.count = 0;
    while (available > 0) {
        packet.data[packet.count++] = (uint8_t)port->read();
        available--;
    }

    // and send the reply
    _mav_finalize_message_chan_send(chan, 
                                    MAVLINK_MSG_ID_SERIAL_CONTROL,
                                    (const char *)&packet,
                                    MAVLINK_MSG_ID_SERIAL_CONTROL_LEN,
                                    MAVLINK_MSG_ID_SERIAL_CONTROL_CRC);
    if ((flags & SERIAL_CONTROL_FLAG_MULTI) && packet.count != 0) {
        hal.scheduler->delay(1);
        goto more_data;
    }
}
