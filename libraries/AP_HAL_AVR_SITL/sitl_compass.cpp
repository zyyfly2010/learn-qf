/*
  SITL handling

  This simulates a compass

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"
#include "HAL_AVR_SITL_Class.h"

#include <AP_Math.h>
#include "../AP_Compass/AP_Compass.h"
#include "../AP_Declination/AP_Declination.h"
#include "../SITL/SITL.h"

#include <stdio.h> // added

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

/*
  setup the compass with new input
  all inputs are in degrees
 */

uint8_t storeIndexMag;
uint32_t lastStoreTimeMag;
VectorN<uint32_t,50> timeStampMag;
VectorN<Vector3f,50> storedDataMag;
uint32_t timeDeltaMag;
uint32_t delayed_time_Mag;
Vector3f mag_data;
Vector3f new_mag_data;

Vector3f current_data;
uint32_t checktime;

void SITL_State::_update_compass(float rollDeg, float pitchDeg, float yawDeg)
{
	if (_compass == NULL) {
		// no compass in this sketch
		return;
	}
	yawDeg += _sitl->mag_error;
	if (yawDeg > 180.0f) {
		yawDeg -= 360.0f;
	}
	if (yawDeg < -180.0f) {
		yawDeg += 360.0f;
	}
	_compass->setHIL(radians(rollDeg), radians(pitchDeg), radians(yawDeg));
	Vector3f noise = _rand_vec3f() * _sitl->mag_noise;
    Vector3f motor = _sitl->mag_mot.get() * _current;

    // _compass->setHIL(_compass->getHIL() + noise+motor);
    new_mag_data = _compass->getHIL() + noise + motor;

    uint32_t now = hal.scheduler->millis();

    // add delay
	uint32_t bestTimeDeltaMag = 2000; // initialise large time representing buffer entry closest to current time - delay.
	uint8_t bestIndexMag = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
	if (now - lastStoreTimeMag >= 10) { // store data every 10 ms.
        lastStoreTimeMag = now;
        if (storeIndexMag > 49) { // reset buffer index if index greater than size of buffer
            storeIndexMag = 0;
        }
        storedDataMag[storeIndexMag] = new_mag_data; // add data to current index
        timeStampMag[storeIndexMag] = lastStoreTimeMag; // add timeStampMag to current index
        storeIndexMag = storeIndexMag + 1; // increment index
	}

	// return delayed measurement
	delayed_time_Mag = now - _sitl->mag_delay; // get time corresponding to delay
	// find data corresponding to delayed time in buffer
	for (uint8_t i=0; i<=49; i++)
    {
        timeDeltaMag = delayed_time_Mag - timeStampMag[i]; // find difference between delayed time and measurement in buffer
        // if this difference is smaller than last delta, store this time
        if (timeDeltaMag < bestTimeDeltaMag)
        {
            bestIndexMag = i;
            bestTimeDeltaMag = timeDeltaMag;
        }
	}
	if (bestTimeDeltaMag < 200) // only output stored state if < 200 msec retrieval error
	{
        mag_data = storedDataMag[bestIndexMag];
	}

    _compass->setHIL(mag_data);

//	if (now - checktime >= 10) { // store data every 10 ms.
//        current_data = _compass->getHIL();
//        printf("(%f, %f, %f); ", current_data[0], current_data[1], current_data[2]);
//        current_data = new_mag_data;
//        printf("(%f, %f, %f) \n", current_data[0], current_data[1], current_data[2]);
//        checktime = now;
//	}

    // _compass->setHIL(_compass->getHIL() + noise+motor);
}

#endif
