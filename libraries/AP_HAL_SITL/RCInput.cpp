#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "RCInput.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void SITLRCInput::init()
{
    clear_overrides();
}

bool SITLRCInput::new_input()
{
    if (_sitlState->new_rc_input) {
        _sitlState->new_rc_input = false;
        return true;
    }
    return false;
}

uint16_t SITLRCInput::read(uint8_t ch)
{
    if (ch >= SITL_RC_INPUT_CHANNELS) {
        return 0;
    }
    return _override[ch]? _override[ch] : _sitlState->pwm_input[ch];
}

uint8_t SITLRCInput::read(uint16_t* periods, uint8_t len)
{
    if (len > SITL_RC_INPUT_CHANNELS) {
        len = SITL_RC_INPUT_CHANNELS;
    }
    for (uint8_t i=0; i<len; i++) {
        periods[i] = _override[i]? _override[i] : _sitlState->pwm_input[i];
    }
    return 8;
}

bool SITLRCInput::set_overrides(int16_t *overrides, uint8_t len)
{
    bool res = false;
    if (len > SITL_RC_INPUT_CHANNELS) {
        len = SITL_RC_INPUT_CHANNELS;
    }
    for (uint8_t i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool SITLRCInput::set_override(uint8_t channel, int16_t override)
{
    if (override < 0) return false; /* -1: no change. */
    if (channel < SITL_RC_INPUT_CHANNELS) {
        _override[channel] = override;
        if (override != 0) {
            return true;
        }
    }
    return false;
}

void SITLRCInput::clear_overrides()
{
    memset(_override, 0, sizeof(_override));
}
#endif
