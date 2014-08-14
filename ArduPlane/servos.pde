/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
  servo control functions
 */


/*
  implement a software VTail or elevon mixer. There are 4 different mixing modes
 */
static void channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out)
{
    int16_t c1, c2;
    int16_t v1, v2;

    // first get desired elevator and rudder as -500..500 values
    c1 = chan1_out - 1500;
    c2 = chan2_out - 1500;

    v1 = (c1 - c2) * g.mixing_gain;
    v2 = (c1 + c2) * g.mixing_gain;

    // now map to mixed output
    switch (mixing_type) {
    case MIXING_DISABLED:
        return;

    case MIXING_UPUP:
        break;

    case MIXING_UPDN:
        v2 = -v2;
        break;

    case MIXING_DNUP:
        v1 = -v1;
        break;

    case MIXING_DNDN:
        v1 = -v1;
        v2 = -v2;
        break;
    }

    // scale for a 1500 center and 900..2100 range, symmetric
    v1 = constrain_int16(v1, -600, 600);
    v2 = constrain_int16(v2, -600, 600);

    chan1_out = 1500 + v1;
    chan2_out = 1500 + v2;
}

/*
  setup flaperon output channels
 */
static void flaperon_update(int8_t flap_percent)
{
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon1) ||
        !RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon2)) {
        return;
    }
    int16_t ch1, ch2;
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps. Note that negative manual flap percentates
      are allowed, which give spoilerons
     */
    ch1 = channel_roll->radio_out;

    // 1500 is used here as the neutral value for the output
    // mixer. User can still trim the flaps on the input side using
    // the TRIM of the flap input channel. The *5 is to take a
    // percentage to a value from -500 to 500 for the mixer
    ch2 = 1500 - flap_percent * 5;
    channel_output_mixer(g.flaperon_output, ch1, ch2);
    RC_Channel_aux::set_radio(RC_Channel_aux::k_flaperon1, ch1);
    RC_Channel_aux::set_radio(RC_Channel_aux::k_flaperon2, ch2);
}

/*
  setup servos for idle mode
  Idle mode is used during balloon launch to keep servos still, apart
  from occasional wiggle to prevent freezing up
 */
static void set_servos_idle(void)
{
    if (auto_state.idle_wiggle_stage == 0) {
        RC_Channel::output_trim_all();
        return;
    }
    int16_t servo_value;
    // move over full range for 2 seconds
    auto_state.idle_wiggle_stage += 2;
    if (auto_state.idle_wiggle_stage < 50) {
        servo_value = auto_state.idle_wiggle_stage * (4500 / 50);
    } else if (auto_state.idle_wiggle_stage < 100) {
        servo_value = (100 - auto_state.idle_wiggle_stage) * (4500 / 50);        
    } else if (auto_state.idle_wiggle_stage < 150) {
        servo_value = (100 - auto_state.idle_wiggle_stage) * (4500 / 50);        
    } else if (auto_state.idle_wiggle_stage < 200) {
        servo_value = (auto_state.idle_wiggle_stage-200) * (4500 / 50);        
    } else {
        auto_state.idle_wiggle_stage = 0;
    }
    channel_roll->servo_out = servo_value;
    channel_pitch->servo_out = servo_value;
    channel_rudder->servo_out = servo_value;
    channel_roll->calc_pwm();
    channel_pitch->calc_pwm();
    channel_rudder->calc_pwm();
    channel_roll->output();
    channel_pitch->output();
    channel_throttle->output();
    channel_rudder->output();
    channel_throttle->output_trim();
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = channel_throttle->radio_out;

    if (control_mode == AUTO && auto_state.idle_mode) {
        set_servos_idle();
        return;
    }

    /*
      see if we are doing ground steering.
     */
    if (!steering_control.ground_steering) {
        // we are not at an altitude for ground steering. Set the nose
        // wheel to the rudder just in case the barometer has drifted
        // a lot
        steering_control.steering = steering_control.rudder;
    } else if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_steering)) {
        // we are within the ground steering altitude but don't have a
        // dedicated steering channel. Set the rudder to the ground
        // steering output
        steering_control.rudder = steering_control.steering;
    }
    channel_rudder->servo_out = steering_control.rudder;

    // clear ground_steering to ensure manual control if the yaw stabilizer doesn't run
    steering_control.ground_steering = false;

    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_rudder, steering_control.rudder);
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_steering, steering_control.steering);

    if (control_mode == MANUAL) {
        // do a direct pass through of radio values
        if (g.mix_mode == 0 || g.elevon_output != MIXING_DISABLED) {
            channel_roll->radio_out                = channel_roll->radio_in;
            channel_pitch->radio_out               = channel_pitch->radio_in;
        } else {
            channel_roll->radio_out                = channel_roll->read();
            channel_pitch->radio_out               = channel_pitch->read();
        }
        channel_throttle->radio_out    = channel_throttle->radio_in;
        channel_rudder->radio_out              = channel_rudder->radio_in;

        // setup extra channels. We want this to come from the
        // main input channel, but using the 2nd channels dead
        // zone, reverse and min/max settings. We need to use
        // pwm_to_angle_dz() to ensure we don't trim the value for the
        // deadzone of the main aileron channel, otherwise the 2nd
        // aileron won't quite follow the first one
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, channel_roll->pwm_to_angle_dz(0));
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, channel_pitch->pwm_to_angle_dz(0));

        // this variant assumes you have the corresponding
        // input channel setup in your transmitter for manual control
        // of the 2nd aileron
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input);

        if (g.mix_mode == 0 && g.elevon_output == MIXING_DISABLED) {
            // set any differential spoilers to follow the elevons in
            // manual mode. 
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler1, channel_roll->radio_out);
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler2, channel_pitch->radio_out);
        }
    } else {
        if (g.mix_mode == 0) {
            // both types of secondary aileron are slaved to the roll servo out
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, channel_roll->servo_out);
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron_with_input, channel_roll->servo_out);

            // both types of secondary elevator are slaved to the pitch servo out
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator, channel_pitch->servo_out);
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_elevator_with_input, channel_pitch->servo_out);
        }else{
            /*Elevon mode*/
            float ch1;
            float ch2;
            ch1 = channel_pitch->servo_out - (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->servo_out);
            ch2 = channel_pitch->servo_out + (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->servo_out);

			/* Differential Spoilers
               If differential spoilers are setup, then we translate
               rudder control into splitting of the two ailerons on
               the side of the aircraft where we want to induce
               additional drag.
             */
			if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) && RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
				float ch3 = ch1;
				float ch4 = ch2;
				if ( BOOL_TO_SIGN(g.reverse_elevons) * channel_rudder->servo_out < 0) {
				    ch1 += abs(channel_rudder->servo_out);
				    ch3 -= abs(channel_rudder->servo_out);
				} else {
					ch2 += abs(channel_rudder->servo_out);
				    ch4 -= abs(channel_rudder->servo_out);
				}
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler1, ch3);
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler2, ch4);
			}

            // directly set the radio_out values for elevon mode
            channel_roll->radio_out  =     elevon.trim1 + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0f/ SERVO_MAX));
            channel_pitch->radio_out =     elevon.trim2 + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0f/ SERVO_MAX));
        }

        // push out the PWM values
        if (g.mix_mode == 0) {
            channel_roll->calc_pwm();
            channel_pitch->calc_pwm();
        }
        channel_rudder->calc_pwm();

#if THROTTLE_OUT == 0
        channel_throttle->servo_out = 0;
#else
        // convert 0 to 100% into PWM
        channel_throttle->servo_out = constrain_int16(channel_throttle->servo_out, 
                                                       aparm.throttle_min.get(), 
                                                       aparm.throttle_max.get());

        if (suppress_throttle()) {
            // throttle is suppressed in auto mode
            channel_throttle->servo_out = 0;
            if (g.throttle_suppress_manual) {
                // manual pass through of throttle while throttle is suppressed
                channel_throttle->radio_out = channel_throttle->radio_in;
            } else {
                channel_throttle->calc_pwm();                
            }
        } else if (g.throttle_passthru_stabilize && 
                   (control_mode == STABILIZE || 
                    control_mode == TRAINING ||
                    control_mode == ACRO ||
                    control_mode == FLY_BY_WIRE_A ||
                    control_mode == AUTOTUNE)) {
            // manual pass through of throttle while in FBWA or
            // STABILIZE mode with THR_PASS_STAB set
            channel_throttle->radio_out = channel_throttle->radio_in;
        } else if (control_mode == GUIDED && 
                   guided_throttle_passthru) {
            // manual pass through of throttle while in GUIDED
            channel_throttle->radio_out = channel_throttle->radio_in;
        } else {
            // normal throttle calculation based on servo_out
            channel_throttle->calc_pwm();
        }
#endif
    }

    // Auto flap deployment
    uint8_t auto_flap_percent = 0;
    int8_t manual_flap_percent = 0;

    // work out any manual flap input
    RC_Channel *flapin = RC_Channel::rc_channel(g.flapin_channel-1);
    if (flapin != NULL && !failsafe.ch3_failsafe && failsafe.ch3_counter == 0) {
        flapin->input();
        manual_flap_percent = flapin->percent_input();
    }

    if (auto_throttle_mode) {
        int16_t flapSpeedSource = 0;
        if (airspeed.use()) {
            flapSpeedSource = target_airspeed_cm * 0.01f;
        } else {
            flapSpeedSource = aparm.throttle_cruise;
        }
        if ( g.flap_1_speed != 0 && flapSpeedSource > g.flap_1_speed) {
            auto_flap_percent = 0;
        } else if (g.flap_2_speed != 0 && flapSpeedSource <= g.flap_2_speed) {
            auto_flap_percent = g.flap_2_percent;
        } else {
            auto_flap_percent = g.flap_1_percent;
        }
    }

    // manual flap input overrides auto flap input
    if (abs(manual_flap_percent) > auto_flap_percent) {
        auto_flap_percent = manual_flap_percent;
    }

    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, auto_flap_percent);
    RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap, manual_flap_percent);

    if (control_mode >= FLY_BY_WIRE_B) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit(last_throttle);
    }

    if (control_mode == TRAINING) {
        // copy rudder in training mode
        channel_rudder->radio_out   = channel_rudder->radio_in;
    }

    if (g.flaperon_output != MIXING_DISABLED && g.elevon_output == MIXING_DISABLED && g.mix_mode == 0) {
        flaperon_update(auto_flap_percent);
    }
    if (g.vtail_output != MIXING_DISABLED) {
        channel_output_mixer(g.vtail_output, channel_pitch->radio_out, channel_rudder->radio_out);
    } else if (g.elevon_output != MIXING_DISABLED) {
        channel_output_mixer(g.elevon_output, channel_pitch->radio_out, channel_roll->radio_out);
    }

    //send throttle to 0 or MIN_PWM if not yet armed
    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) { 
            case AP_Arming::YES_MIN_PWM:
                channel_throttle->radio_out = channel_throttle->radio_min;
            break;
            case AP_Arming::YES_ZERO_PWM:
                channel_throttle->radio_out = 0;
            break;
            default:
                //keep existing behavior: do nothing to radio_out
                //(don't disarm throttle channel even if AP_Arming class is)
            break;
        }
    }

#if OBC_FAILSAFE == ENABLED
    // this is to allow the failsafe module to deliberately crash 
    // the plane. Only used in extreme circumstances to meet the
    // OBC rules
    obc.check_crash_plane();
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    // get the servos to the GCS immediately for HIL
    if (comm_get_txspace(MAVLINK_COMM_0) >= 
        MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
        send_servo_out(MAVLINK_COMM_0);
    }
    if (!g.hil_servos) {
        return;
    }
#endif

    // send values to the PWM timers for output
    // ----------------------------------------
    channel_roll->output();
    channel_pitch->output();
    channel_throttle->output();
    channel_rudder->output();
    RC_Channel_aux::output_ch_all();
}


/*
  demo_servos is used on startup to give the user an indication that
  the aircraft has finished starting up
 */
static void demo_servos(uint8_t i) 
{
    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
        servo_write(1, 1400);
        mavlink_delay(400);
        servo_write(1, 1600);
        mavlink_delay(200);
        servo_write(1, 1500);
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}


/* We want to supress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.

   Disable throttle if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode or takeoff speed/accel not yet reached
   *       OR
   *       5 - Home location is not set
*/
static bool suppress_throttle(void)
{
    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (!auto_throttle_mode) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    if (control_mode==AUTO && g.auto_fbw_steer) {
        // user has throttle control
        return false;
    }

    if (control_mode==AUTO && 
        auto_state.takeoff_complete == false && 
        auto_takeoff_check()) {
        // we're in auto takeoff 
        throttle_suppressed = false;
        if (steer_state.hold_course_cd != -1) {
            // update takeoff course hold, if already initialised
            steer_state.hold_course_cd = ahrs.yaw_sensor;
            gcs_send_text_fmt(PSTR("Holding course %ld"), steer_state.hold_course_cd);
        }
        return false;
    }
    
    if (relative_altitude_abs_cm() >= 1000) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D && 
        gps.ground_speed() >= 5) {
        // if we have an airspeed sensor, then check it too, and
        // require 5m/s. This prevents throttle up due to spiky GPS
        // groundspeed with bad GPS reception
        if (!airspeed.use() || airspeed.get_airspeed() >= 5) {
            // we're moving at more than 5 m/s
            throttle_suppressed = false;
            return false;        
        }
    }

    // throttle remains suppressed
    return true;
}


