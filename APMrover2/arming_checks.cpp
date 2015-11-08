// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for rover
 */
#include "Rover.h"

const AP_Param::GroupInfo AP_Arming_Rover::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    AP_GROUPEND
};


/*
  additional arming checks for rover
 */
bool AP_Arming_Rover::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    return ret;
}

bool AP_Arming_Rover::ins_checks(bool report)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(report)) {
        return false;
    }

    // additional rover specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!ahrs.healthy()) {
            if (report) {
                const char *reason = ahrs.prearm_failure_reason();
                if (reason) {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: %s", reason);
                } else {
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "PreArm: AHRS not healthy");
                }
            }
            return false;
        }
    }

    return true;
}
