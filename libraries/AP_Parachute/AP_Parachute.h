// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AP_Parachute.h
/// @brief	Parachute release library

#ifndef AP_PARACHUTE_H
#define AP_PARACHUTE_H

#include <AP_Param.h>
#include <AP_Common.h>
#include <AP_Relay.h>

#define AP_PARACHUTE_TRIGGER_TYPE_SERVO        0
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY        1

#define AP_PARACHUTE_TRIGGER_DEFAULT_TRIGGER_TYPE  AP_PARACHUTE_TRIGGER_TYPE_SERVO    // default is to use servo to trigger camera

#define AP_PARACHUTE_RELEASE_DURATION_MS       1000    // when parachute is released, servo or relay stay at their released position/value for 1000ms (1second)

#define AP_PARACHUTE_SERVO_ON_PWM_DEFAULT      1300    // default PWM value to move servo to when shutter is activated
#define AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT     1100    // default PWM value to move servo to when shutter is deactivated

/// @class	Camera
/// @brief	Object managing a Photo or video camera
class AP_Parachute {

public:

    /// Constructor
    AP_Parachute(AP_Relay *obj_relay) :
        _release_time(0)
    {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
        _apm_relay = obj_relay;
    }

    /// enabled - enable or disable parachute release
    void enabled(bool on_off) { _enabled = on_off; }

    /// enabled - returns true if parachute release is enabled
    bool enabled() const { return _enabled; }

    /// release - release parachute
    void release();

    /// update - shuts off the trigger should be called at about 10hz
    void update();

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int8     _release_type;  // 0:Servo,1:Relay
    AP_Int16    _servo_on_pwm;  // PWM value to move servo to when shutter is activated
    AP_Int16    _servo_off_pwm; // PWM value to move servo to when shutter is deactivated

    // internal variables
    uint32_t    _release_time;  // count of number of cycles servo or relay has been at on position
    AP_Relay*   _apm_relay;     // pointer to relay object from the base class Relay. The subclasses could be AP_Relay_APM1 or AP_Relay_APM2
};

#endif /* AP_PARACHUTE_H */
