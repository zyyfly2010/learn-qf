/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_ADSB_H
#define AP_ADSB_H
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
    ADS-B RF based collision avoidance module
    https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast

  Tom Pittenger, November 2015
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#define VEHCILE_LIST_LENGTH     200
#define VEHCILE_TIMEOUT_MS      10000

class AP_ADSB
{
public:
    enum behavior {
        BEHAVIOR_NONE = 0,
        BEHAVIOR_LOITER_1_TURN = 1
    };

    struct adsb_vehicle_t {
        mavlink_adsb_vehicle_t info; // the whole mavlink struct with all the juicy details. sizeof() == 31
        uint32_t last_update_ms; // last time this was refreshed, allows timeouts
    };


    // Constructor
    AP_ADSB(AP_AHRS &ahrs, AP_Mission &mission) :
        _ahrs(ahrs),
        _mission(mission)
    {
        AP_Param::setup_object_defaults(this, var_info);

        for (uint16_t i=0; i< VEHCILE_LIST_LENGTH; i++) {
            memset(&_vehicle_list[i], 0, sizeof(_vehicle_list[i]));
        }
    }

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    void update(void);
    void perform_threat_detection(void);

    uint16_t get_vehicle_count() { return _vehicle_count; }

    void update_vehicle(mavlink_message_t* msg);

    bool get_another_vehicle_within_radius()  { return _another_vehicle_within_radius; }

    bool get_is_evading_threat()  { return _is_evading_threat; }
    void set_is_evading_threat(bool is_evading) { _is_evading_threat = is_evading; }

private:

    Location get_location(const adsb_vehicle_t vehicle);

    // return index of given vehicle if ICAO_ADDRESS matches. return -1 if no match
    int16_t find_index(const adsb_vehicle_t vehicle);

    // remove a vehicle from the list
    void delete_vehicle(const uint16_t index);

    void set_vehicle(const uint16_t index, adsb_vehicle_t vehicle);

    void print_vehicle(const adsb_vehicle_t vehicle);

    // reference to AHRS, so we can ask for our position,
    // heading and speed
    AP_AHRS &_ahrs;

    // reference to AP_Mission, so we can ask preload terrain data for
    // all waypoints
    const AP_Mission &_mission;

    AP_Int8     _enable;
    AP_Int8     _behavior;
    adsb_vehicle_t _vehicle_list[VEHCILE_LIST_LENGTH];
    uint16_t    _vehicle_count = 0;
    bool        _another_vehicle_within_radius = false;
    bool        _is_evading_threat = false;
};
#endif // AP_ADSB_H
