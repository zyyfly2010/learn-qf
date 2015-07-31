/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

#define COMPASS_CAL_STICK_DELAY 2.0f // 2 seconds
#define COMPASS_CAL_DELAY 5.0f

void Tracker::compass_cal_update() {
    compass.compass_cal_update();
    /* No stick Calibration for Antenna Tracker
    if (compass.is_calibrating()) {
        if(g.rc_4.control_in < -4000 && g.rc_3.control_in > 900) {
            compass.cancel_calibration_all();
        }
        return;
    }

    bool stick_gesture_detected = g.rc_4.control_in > 4000 && g.rc_3.control_in > 900;
    static uint32_t stick_gesture_begin = 0;
    uint32_t tnow = millis();

    if(!stick_gesture_detected) {
        stick_gesture_begin = tnow;
    } else {
        if(tnow-stick_gesture_begin > 1000*COMPASS_CAL_STICK_DELAY) {
            compass.start_calibration_all(true,true,COMPASS_CAL_DELAY);
        }
    }
    */
}
