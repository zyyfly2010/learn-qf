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
  ADS-B handling

  Tom Pittenger, November 2015
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <inttypes.h>

#define VEHCILE_LIST_LENGTH 10

class AP_ADSB
{
public:
    enum behavior {
        BEHAVIOR_NONE = 0,
        BEHAVIOR_LOITER_1_TURN = 1
    };

    // Constructor
    AP_ADSB()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8  _enable;
    AP_Int8  _behavior;
    mavlink_adsb_vehicle_t[VEHCILE_LIST_LENGTH] vehicle_list; // sizeof(mavlink_adsb_vehicle_t) = 31 bytes

};

#endif // AP_ADSB_H
