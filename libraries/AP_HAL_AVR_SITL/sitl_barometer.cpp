/*
  SITL handling

  This simulates a barometer

  Andrew Tridgell November 2011
 */

#include <AP_HAL.h>
#include <AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL

#include "AP_HAL_AVR_SITL.h"

using namespace AVR_SITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/*
  setup the barometer with new input
  altitude is in meters
 */

uint8_t storeIndexBaro;
uint32_t lastStoreTimeBaro = 10;
VectorN<uint32_t,50> timeStampBaro;
VectorN<float,50> storedDataBaro;
uint32_t timeDeltaBaro;
uint32_t delayed_time_baro;

void SITL_State::_update_barometer(float altitude)
{
	static uint32_t last_update;

	float sim_alt = altitude;

	if (_barometer == NULL) {
		// this sketch doesn't use a barometer
		return;
	}

        if (_sitl->baro_disable) {
            // barometer is disabled
            return;
        }

	// 80Hz, to match the real APM2 barometer
    uint32_t now = hal.scheduler->millis();
	if ((now - last_update) < 12) {
		return;
	}
	last_update = now;

	sim_alt += _sitl->baro_drift * now / 1000;
	sim_alt += _sitl->baro_noise * _rand_float();

	// add baro glitch
	sim_alt += _sitl->baro_glitch;

    // add delay
	uint32_t bestTimeDeltaBaro = 200; // initialise large time representing buffer entry closest to current time - delay.
	uint8_t bestIndexBaro = 0; // initialise number representing the index of the entry in buffer closest to delay.

    // storing data from sensor to buffer
	if (now - lastStoreTimeBaro >= 10) { // store data every 10 ms.
        lastStoreTimeBaro = now;
        if (storeIndexBaro > 49) { // reset buffer index if index greater than size of buffer
            storeIndexBaro = 0;
        }
        storedDataBaro[storeIndexBaro] = sim_alt; // add data to current index
        timeStampBaro[storeIndexBaro] = lastStoreTimeBaro; // add timeStampBaro to current index
        storeIndexBaro = storeIndexBaro + 1; // increment index
	}

	// return delayed measurement
	delayed_time_baro = now - _sitl->baro_delay; // get time corresponding to delay
	// find data corresponding to delayed time in buffer
	for (uint8_t i=0; i<=49; i++)
    {
        timeDeltaBaro = delayed_time_baro - timeStampBaro[i]; // find difference between delayed time and measurement in buffer
        // if this difference is smaller than last delta, store this time
        if (timeDeltaBaro < bestTimeDeltaBaro)
        {
            bestIndexBaro = i;
            bestTimeDeltaBaro = timeDeltaBaro;
        }
	}
	if (bestTimeDeltaBaro < 200) // only output stored state if < 200 msec retrieval error
	{
        sim_alt = storedDataBaro[bestIndexBaro];
	}

	_barometer->setHIL(sim_alt);
}

#endif
