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
 *  NavEKF based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */
#include <AP_HAL.h>
#include <AP_AHRS.h>

#if AP_AHRS_NAVEKF_AVAILABLE

extern const AP_HAL::HAL& hal;

// return the smoothed gyro vector corrected for drift
const Vector3f &AP_AHRS_NavEKF::get_gyro(void) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        return ekf1._gyro_estimate;
    case AHRS_SELECTED_EKF2:
        return ekf2._gyro_estimate;
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::get_gyro();
    }
}

const Matrix3f &AP_AHRS_NavEKF::get_dcm_matrix(void) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        return ekf1._dcm_matrix;
    case AHRS_SELECTED_EKF2:
        return ekf2._dcm_matrix;
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::get_dcm_matrix();
    }
}

const Vector3f &AP_AHRS_NavEKF::get_gyro_drift(void) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        return ekf1._gyro_bias;
    case AHRS_SELECTED_EKF2:
        return ekf2._gyro_bias;
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::get_gyro_drift();
    }
}

// reset the current gyro drift estimate
//  should be called if gyro offsets are recalculated
void AP_AHRS_NavEKF::reset_gyro_drift(void)
{
    // update DCM
    AP_AHRS_DCM::reset_gyro_drift();

    // reset the EKF gyro bias states
    EKF1.resetGyroBias();
    EKF2.resetGyroBias();
}

void AP_AHRS_NavEKF::update(void)
{
    // we need to restore the old DCM attitude values as these are
    // used internally in DCM to calculate error values for gyro drift
    // correction
    roll = _dcm_attitude.x;
    pitch = _dcm_attitude.y;
    yaw = _dcm_attitude.z;
    update_cd_values();

    AP_AHRS_DCM::update();

    // keep DCM attitude available for get_secondary_attitude()
    _dcm_attitude(roll, pitch, yaw);

    _update_ekf1();
    _update_ekf2();
}


void AP_AHRS_NavEKF::_update_ekf1(void)
{
    if (!ekf1.started) {
        // if we have a GPS lock and more than 6 satellites, we can start the EKF
        if (get_gps().status() >= AP_GPS::GPS_OK_FIX_3D && get_gps().num_sats() >= _gps_minsats) {
            if (ekf1.start_time_ms == 0) {
                ekf1.start_time_ms = hal.scheduler->millis();
            }
            if (hal.scheduler->millis() - ekf1.start_time_ms > startup_delay_ms) {
                ekf1.started = true;
                EKF1.InitialiseFilterDynamic();
            }
        }
    }
    if (ekf1.started) {
        EKF1.UpdateFilter();
        EKF1.getRotationBodyToNED(ekf1._dcm_matrix);
        if (using_EKF() == AHRS_SELECTED_EKF1) {
            Vector3f eulers;
            EKF1.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF1.getGyroBias(ekf1._gyro_bias);
            ekf1._gyro_bias = -ekf1._gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            ekf1._gyro_estimate.zero();
            uint8_t healthy_count = 0;    
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i)) {
                    ekf1._gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                ekf1._gyro_estimate /= healthy_count;
            }
            ekf1._gyro_estimate += ekf1._gyro_bias;
        }
    }
}

void AP_AHRS_NavEKF::_update_ekf2(void)
{
    if (!ekf2.started) {
        // if we have a GPS lock and more than 6 satellites, we can start the EKF
        if (get_gps().status() >= AP_GPS::GPS_OK_FIX_3D && get_gps().num_sats() >= _gps_minsats) {
            if (ekf2.start_time_ms == 0) {
                ekf2.start_time_ms = hal.scheduler->millis();
            }
            if (hal.scheduler->millis() - ekf2.start_time_ms > startup_delay_ms) {
                ekf2.started = true;
                EKF2.InitialiseFilterDynamic();
            }
        }
    }
    if (ekf2.started) {
        EKF2.UpdateFilter();
        EKF2.getRotationBodyToNED(ekf2._dcm_matrix);
        if (using_EKF() == AHRS_SELECTED_EKF2) {
            Vector3f eulers;
            EKF2.getEulerAngles(eulers);
            roll  = eulers.x;
            pitch = eulers.y;
            yaw   = eulers.z;

            update_cd_values();
            update_trig();

            // keep _gyro_bias for get_gyro_drift()
            EKF2.getGyroBias(ekf2._gyro_bias);
            ekf2._gyro_bias = -ekf2._gyro_bias;

            // calculate corrected gryo estimate for get_gyro()
            ekf2._gyro_estimate.zero();
            uint8_t healthy_count = 0;    
            for (uint8_t i=0; i<_ins.get_gyro_count(); i++) {
                if (_ins.get_gyro_health(i)) {
                    ekf2._gyro_estimate += _ins.get_gyro(i);
                    healthy_count++;
                }
            }
            if (healthy_count > 1) {
                ekf2._gyro_estimate /= healthy_count;
            }
            ekf2._gyro_estimate += ekf2._gyro_bias;
        }
    }
}

void AP_AHRS_NavEKF::reset(bool recover_eulers)
{
    AP_AHRS_DCM::reset(recover_eulers);
    if (ekf1.started) {
        EKF1.InitialiseFilterBootstrap();        
    }
    if (ekf2.started) {
        EKF2.InitialiseFilterBootstrap();        
    }
}

// reset the current attitude, used on new IMU calibration
void AP_AHRS_NavEKF::reset_attitude(const float &_roll, const float &_pitch, const float &_yaw)
{
    AP_AHRS_DCM::reset_attitude(_roll, _pitch, _yaw);
    if (ekf1.started) {
        EKF1.InitialiseFilterBootstrap();        
    }
    if (ekf2.started) {
        EKF2.InitialiseFilterBootstrap();        
    }
}

// dead-reckoning support
bool AP_AHRS_NavEKF::get_position(struct Location &loc) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        return EKF1.getLLH(loc);
    case AHRS_SELECTED_EKF2:
        return EKF2.getLLH(loc);
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::get_position(loc);
    }
}

// status reporting of estimated errors
float AP_AHRS_NavEKF::get_error_rp(void)
{
    return AP_AHRS_DCM::get_error_rp();
}

float AP_AHRS_NavEKF::get_error_yaw(void)
{
    return AP_AHRS_DCM::get_error_yaw();
}

// return a wind estimation vector, in m/s
Vector3f AP_AHRS_NavEKF::wind_estimate(void)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1: {
        Vector3f wind;
        EKF1.getWind(wind);
        return wind;
    }
    case AHRS_SELECTED_EKF2: {
        Vector3f wind;
        EKF2.getWind(wind);
        return wind;
    }
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::wind_estimate();
    }
}

// return an airspeed estimate if available. return true
// if we have an estimate
bool AP_AHRS_NavEKF::airspeed_estimate(float *airspeed_ret) const
{
    return AP_AHRS_DCM::airspeed_estimate(airspeed_ret);
}

// true if compass is being used
bool AP_AHRS_NavEKF::use_compass(void)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        return EKF1.use_compass();
    case AHRS_SELECTED_EKF2:
        return EKF2.use_compass();
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::use_compass();
    }
}


// return secondary attitude solution if available, as eulers in radians
bool AP_AHRS_NavEKF::get_secondary_attitude(Vector3f &eulers)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF2:
        // return DCM attitude
        eulers = _dcm_attitude;
        return true;
    case AHRS_SELECTED_DCM:
    default:
        if (ekf1.started) {
            // EKF1 is secondary
            EKF1.getEulerAngles(eulers);
            return true;
        }
        if (ekf2.started) {
            // EKF1 is secondary
            EKF2.getEulerAngles(eulers);
            return true;
        }
    }
    // no secondary available
    return false;
}

// return secondary position solution if available
bool AP_AHRS_NavEKF::get_secondary_position(struct Location &loc)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
    case AHRS_SELECTED_EKF2:
        // return DCM position
        AP_AHRS_DCM::get_position(loc);
        return true;
    case AHRS_SELECTED_DCM:
    default:
        if (ekf1.started) {
            // EKF1 is secondary
            EKF1.getLLH(loc);
            return true;
        }
        if (ekf2.started) {
            // EKF1 is secondary
            EKF2.getLLH(loc);
            return true;
        }
    }
    // no secondary available
    return false;
}

// EKF has a better ground speed vector estimate
Vector2f AP_AHRS_NavEKF::groundspeed_vector(void)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1: {
        Vector3f vec;
        EKF1.getVelNED(vec);
        return Vector2f(vec.x, vec.y);
    }
    case AHRS_SELECTED_EKF2: {
        Vector3f vec;
        EKF2.getVelNED(vec);
        return Vector2f(vec.x, vec.y);
    }
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::groundspeed_vector();
    }
}

void AP_AHRS_NavEKF::set_home(const Location &loc)
{
    AP_AHRS_DCM::set_home(loc);
}

// return true if inertial navigation is active
bool AP_AHRS_NavEKF::have_inertial_nav(void) const 
{
    return using_EKF() != AHRS_SELECTED_DCM;
}

// return a ground velocity in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_velocity_NED(Vector3f &vec) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        EKF1.getVelNED(vec);
        return true;
    case AHRS_SELECTED_EKF2:
        EKF2.getVelNED(vec);
        return true;
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return false;
}

// return a relative ground position in meters/second, North/East/Down
// order. Must only be called if have_inertial_nav() is true
bool AP_AHRS_NavEKF::get_relative_position_NED(Vector3f &vec) const
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        return EKF1.getPosNED(vec);
    case AHRS_SELECTED_EKF2:
        return EKF2.getPosNED(vec);
    case AHRS_SELECTED_DCM:
    default:
        break;
    }
    return false;
}

AP_AHRS_NavEKF::AHRS_selected AP_AHRS_NavEKF::using_EKF(void) const
{
    if (_ekf_use == 1 && ekf1.started) {
        if (EKF1.healthy()) {
            return AHRS_SELECTED_EKF1;
        }
    } else if (_ekf_use == 2 && ekf2.started) {
        if (EKF2.healthy()) {
            return AHRS_SELECTED_EKF2;
        }
    }
    return AHRS_SELECTED_DCM;
}

/*
  check if the AHRS subsystem is healthy
*/
bool AP_AHRS_NavEKF::healthy(void)
{
    switch (using_EKF()) {
    case AHRS_SELECTED_EKF1:
        return ekf1.started && EKF1.healthy();
    case AHRS_SELECTED_EKF2:
        return ekf2.started && EKF2.healthy();
    case AHRS_SELECTED_DCM:
    default:
        return AP_AHRS_DCM::healthy();    
    }
}

// true if the AHRS has completed initialisation
bool AP_AHRS_NavEKF::initialised(void) const
{
    // initialisation complete 10sec after ekf has started
    return (ekf1.started && 
            (hal.scheduler->millis() - ekf1.start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS) &&
            ekf2.started && 
            (hal.scheduler->millis() - ekf2.start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
}

#endif // AP_AHRS_NAVEKF_AVAILABLE

