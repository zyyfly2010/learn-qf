// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

//
//  Septentrio GPS driver for ArduPilot.
//	Code by Michael Oborne
//

#include <AP_GPS.h>
#include "AP_GPS_SBF.h"
#include <DataFlash.h>

#if GPS_RTK_AVAILABLE

extern const AP_HAL::HAL& hal;

#define SBF_DEBUGGING 1

#if SBF_DEBUGGING
 # define Debug(fmt, args ...)                  \
do {                                            \
    hal.console->printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_SBF::AP_GPS_SBF(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port),
    crc_error_counter(0),
    last_hdop(999)
{
    sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
}

// Process all bytes available from the stream
//
bool
AP_GPS_SBF::read(void)
{
    while (port->available() > 0)
    {
        uint8_t temp = port->read();
        bool ret = parse(temp);
        if (ret == true)
            return true;
    }
    
    return false;
}

bool
AP_GPS_SBF::parse(uint8_t temp) 
{
    switch (sbf_msg.sbf_state)
    {
        default:
        case sbf_msg_parser_t::PREAMBLE1:
            if (temp == SBF_PREAMBLE1)
				sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE2;
            sbf_msg.read = 0;
            break;
        case sbf_msg_parser_t::PREAMBLE2:
            if (temp == SBF_PREAMBLE2)
            {
				sbf_msg.sbf_state = sbf_msg_parser_t::CRC1;
            }
            else
            {
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
            }
            break;
        case sbf_msg_parser_t::CRC1:
            sbf_msg.crc = temp;
			sbf_msg.sbf_state = sbf_msg_parser_t::CRC2;
            break;
        case sbf_msg_parser_t::CRC2:
            sbf_msg.crc += (uint16_t)(temp << 8);
			sbf_msg.sbf_state = sbf_msg_parser_t::BLOCKID1;
            break;
        case sbf_msg_parser_t::BLOCKID1:
            sbf_msg.blockid = temp;
			sbf_msg.sbf_state = sbf_msg_parser_t::BLOCKID2;
            break;
        case sbf_msg_parser_t::BLOCKID2:
            sbf_msg.blockid += (uint16_t)(temp << 8);
			sbf_msg.sbf_state = sbf_msg_parser_t::LENGTH1;
            break;
        case sbf_msg_parser_t::LENGTH1:
            sbf_msg.length = temp;
			sbf_msg.sbf_state = sbf_msg_parser_t::LENGTH2;
            break;
        case sbf_msg_parser_t::LENGTH2:
            sbf_msg.length += (uint16_t)(temp << 8);
			sbf_msg.sbf_state = sbf_msg_parser_t::DATA;
            if (sbf_msg.length % 4 != 0)
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
            break;
        case sbf_msg_parser_t::DATA:
            sbf_msg.data[sbf_msg.read] = temp;
            sbf_msg.read++;
            if (sbf_msg.read > (sbf_msg.length - 8))
            {
				uint16_t crc = crc16_ccitt((uint8_t*)&sbf_msg.blockid, 2, 0);
				crc = crc16_ccitt((uint8_t*)&sbf_msg.length, 2, crc);
				crc = crc16_ccitt((uint8_t*)&sbf_msg.data, sbf_msg.length - 8, crc);

                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                
                if (sbf_msg.crc == crc)
                {
                    return process_message();
                }
                else
                {
                    crc_error_counter++;
                }                
            }                        
            break;
    }
    
    return false;
}

bool
AP_GPS_SBF::process_message(void)
{
    uint32_t blockid = (sbf_msg.blockid & 4095);

    if (blockid == 4027) // obs
    {

    }
    if (blockid == 4007) // geo position
    {
        msg4007* temp = (msg4007*)sbf_msg.data;
        
        // Update time state
        state.time_week = temp->WNc;
        state.time_week_ms = (uint32_t)(temp->TOW/1000.0f);

        state.hdop = last_hdop;

        // Update velocity state
        state.velocity[0] = (float)(temp->Vn / 1000.0);
        state.velocity[1] = (float)(temp->Ve / 1000.0);
        state.velocity[2] = (float)(-temp->Vu / 1000.0);

        float ground_vector_sq = state.velocity[0] * state.velocity[0] + state.velocity[1] * state.velocity[1];
        state.ground_speed = (float)safe_sqrt(ground_vector_sq);

        state.ground_course_cd = (int32_t)(100 * ToDeg(atan2f(state.velocity[1], state.velocity[0])));
        if (state.ground_course_cd < 0)
        {
            state.ground_course_cd += 36000;
        }

        // Update position state

        state.location.lat = (int32_t)(ToDeg(temp->Latitude) * 1e7);
        state.location.lng = (int32_t)(ToDeg(temp->Longitude) * 1e7);
        state.location.alt = (int32_t)(temp->Height * 1e2);
        state.num_sats = temp->NrSV;

        switch (temp->Mode & 7)
        {
            case 0:
                state.status = AP_GPS::NO_FIX;
                break;
            case 1:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 2:
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;
            case 3:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 4:
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK;
                break;
            case 5:
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;
            case 6:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 7:
                state.status = AP_GPS::GPS_OK_FIX_3D_RTK;
                break;
            case 8:
                state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                break;
            case 9:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
            case 10:
                state.status = AP_GPS::GPS_OK_FIX_3D;
                break;
        }
        return true;
    }
    if (blockid == 4001) // dops
    {
        msg4001* temp = (msg4001*)sbf_msg.data;

        last_hdop = temp->HDOP;
    }

    return false;
}

void 
AP_GPS_SBF::inject_data(uint8_t *data, uint8_t len)
{

    if (port->txspace() > len) {
        last_injected_data_ms = hal.scheduler->millis();
        port->write(data, len);
    } else {
        Debug("SBF: Not enough TXSPACE");
    }
}

bool
AP_GPS_SBF::_detect(struct SBF_detect_state &state, uint8_t temp)
{
    switch (state.sbf_state)
    {
        default:
        case SBF_detect_state::PREAMBLE1:
            if (temp == SBF_PREAMBLE1)
				state.sbf_state = SBF_detect_state::PREAMBLE2;
            state.read = 0;
            break;
        case SBF_detect_state::PREAMBLE2:
            if (temp == SBF_PREAMBLE2)
            {
				state.sbf_state = SBF_detect_state::CRC1;
            }
            else
            {
                state.sbf_state = SBF_detect_state::PREAMBLE1;
            }
            break;
        case SBF_detect_state::CRC1:
            state.crc = temp;
			state.sbf_state = SBF_detect_state::CRC2;
            break;
        case SBF_detect_state::CRC2:
            state.crc += (uint16_t)(temp << 8);
			state.sbf_state = SBF_detect_state::BLOCKID1;
            break;
        case SBF_detect_state::BLOCKID1:
            state.blockid = temp;
			state.sbf_state = SBF_detect_state::BLOCKID2;
            break;
        case SBF_detect_state::BLOCKID2:
            state.blockid += (uint16_t)(temp << 8);
			state.sbf_state = SBF_detect_state::LENGTH1;
            break;
        case SBF_detect_state::LENGTH1:
            state.length = temp;
			state.sbf_state = SBF_detect_state::LENGTH2;
            break;
        case SBF_detect_state::LENGTH2:
            state.length += (uint16_t)(temp << 8);
			state.sbf_state = SBF_detect_state::DATA;
            if (state.length % 4 != 0)
                state.sbf_state = SBF_detect_state::PREAMBLE1;
            break;
        case SBF_detect_state::DATA:
            state.data[state.read] = temp;
            state.read++;
            if (state.read > (state.length - 8))
            {
				uint16_t crc = crc16_ccitt((uint8_t*)&state.blockid, 2, 0);
				crc = crc16_ccitt((uint8_t*)&state.length, 2, crc);
				crc = crc16_ccitt((uint8_t*)&state.data, state.length - 8, crc);

                state.sbf_state = SBF_detect_state::PREAMBLE1;
                
                if (state.crc == crc)
                {
                    return true;
                }               
            }                        
            break;
    }
    
    return false;
}

#endif // GPS_RTK_AVAILABLE
