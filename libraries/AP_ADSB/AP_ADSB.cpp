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

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_ADSB::var_info[] = {
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

void AP_ADSB::update(void)
{
    uint16_t index = 0;
    while (index < _vehicle_count) {
        // check list and drop stale vehicles
        if (!_enable ||
            hal.scheduler->millis() - _vehicle_list[index].last_update_ms > VEHCILE_TIMEOUT_MS) {
             // don't increment index, we want to check this same index again because the contents changed
            // also, if we're disabled then clear the list
            delete_vehicle(index);
        } else {
            index++;
        }
    }

    perform_threat_detection();
}

void AP_ADSB::perform_threat_detection(void)
{
    Location my_loc;
    if (_ahrs.get_position(my_loc) == false) {
        // nothing to do or current location is unknown so we can't calculate any collisions
        _another_vehicle_within_radius = false;
        return;
    }

    bool no_threat_within_radius = true;
    for (uint16_t index = 0; index < _vehicle_count; index++) {
        // TODO: perform more advanced threat detection
        Location vehicle_loc = get_location(_vehicle_list[index]);

        // if within radius, set flag and enforce a double radius to clear flag
        float threat_distance = get_distance(vehicle_loc, my_loc);
        if (threat_distance <= 2*VEHCILE_THREADT_RADIUS_M) {
            no_threat_within_radius = false;
            if (threat_distance <= VEHCILE_THREADT_RADIUS_M) {
                _another_vehicle_within_radius = true;
            }
        } // if get
    } // for

    if (no_threat_within_radius) {
        _another_vehicle_within_radius = false;
    }

}

Location AP_ADSB::get_location(const adsb_vehicle_t vehicle)
{
    Location loc;
    loc.alt = vehicle.info.altitude * 100;
    loc.lat = vehicle.info.lat * 1e7;
    loc.lng = vehicle.info.lon * 1e7;
    loc.flags.relative_alt = false;
    return loc;
}

/*
 *  delete a vehicle by copying last vehicle to
 *  current index then decrementing count
 */
void AP_ADSB::delete_vehicle(const uint16_t index)
{
    if (index < _vehicle_count) {
        memcpy(&_vehicle_list[index], &_vehicle_list[_vehicle_count-1], sizeof(adsb_vehicle_t));
        memset(&_vehicle_list[_vehicle_count-1], 0, sizeof(adsb_vehicle_t));
         _vehicle_count--;
    }
}

/*
 * Search _vehicle_list for the given vehicle. A match
 * depends on ICAO_ADDRESS. Returns index of vehicle.
 * If not found it returns an invalid index of -1
 */
int16_t AP_ADSB::find_index(const adsb_vehicle_t vehicle)
{
    for (uint16_t i = 0; i < _vehicle_count; i++) {
        if (_vehicle_list[i].info.ICAO_address == vehicle.info.ICAO_address) {
            return i;
        }
    }
    return -1;
}

/*
 * Update the vehicle list. If the vehicle is already in the
 * list then it will update it, otherwise it will be added.
 */
void AP_ADSB::update_vehicle(const mavlink_message_t* packet)
{
    if (!_enable) {
        return;
    }

    adsb_vehicle_t vehicle;
    mavlink_msg_adsb_vehicle_decode(packet, &vehicle.info);

    int32_t index = find_index(vehicle);
    if (index >= 0) {
        // found, update it
        set_vehicle((uint16_t)index, vehicle);
    } else if (_vehicle_count < VEHCILE_LIST_LENGTH-1) {
        // not found, add it if there's room
        set_vehicle(_vehicle_count, vehicle);
        _vehicle_count++;
    }
}

/*
 * Copy a vehicle's data into the list
 */
void AP_ADSB::set_vehicle(const uint16_t index, const adsb_vehicle_t vehicle)
{
    if (index < VEHCILE_LIST_LENGTH) {
        memcpy(&_vehicle_list[index], &vehicle, sizeof(adsb_vehicle_t));
        _vehicle_list[index].last_update_ms = hal.scheduler->millis();
    }
}

void AP_ADSB::print_vehicle(const adsb_vehicle_t vehicle)
{
    hal.console->printf("%d, %.4f, %.4f, %.1f, %.1f, %.1f, %d, %d, %s, %d  %d  %d",
          vehicle.info.ICAO_address,
          (double)vehicle.info.lat,
          (double)vehicle.info.lon,
          (double)vehicle.info.altitude,
          (double)vehicle.info.hor_velocity,
          (double)vehicle.info.ver_velocity,
          vehicle.info.heading,
          vehicle.info.altitude_type,
          vehicle.info.callsign,
          vehicle.info.emitterType,
          vehicle.info.tslc,
          vehicle.info.validFlags);
}


