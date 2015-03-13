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

uint8_t storeBaroIndex;
uint32_t lastBaroStoreTime;
VectorN<uint32_t,50> baroTimeStamp;
VectorN<float,50> storedBaro;
uint32_t timeBaroDelta;
uint32_t delayed_baro_time;

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
///////////////////////////////////////// add baro delay ////////////////////////////////////////
	uint32_t bestTimeBaroDelta = 200;
	uint8_t bestBaroIndex = 0;

	if (now - lastBaroStoreTime >= 10) {
        lastBaroStoreTime = now;
        if (storeBaroIndex > 49) {
            storeBaroIndex = 0;
        }
        storedBaro[storeBaroIndex] = sim_alt;
        baroTimeStamp[storeBaroIndex] = lastBaroStoreTime;
        storeBaroIndex = storeBaroIndex + 1;
	}
	delayed_baro_time = now - _sitl->baro_delay;
	for (uint8_t i=0; i<=49; i++)
    {
        timeBaroDelta = delayed_baro_time - baroTimeStamp[i];
        if (timeBaroDelta < bestTimeBaroDelta)
        {
            bestBaroIndex = i;
            bestTimeBaroDelta = timeBaroDelta;
        }
	}
	if (bestTimeBaroDelta < 200) // only output stored state if < 200 msec retrieval error
	{
        sim_alt = storedBaro[bestBaroIndex];
	}
///////////////////////////////////////// add baro delay ////////////////////////////////////////
	_barometer->setHIL(sim_alt);
}

#endif
