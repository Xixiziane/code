/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ecl_yaw_controller.cpp
 * Implementation of a simple orthogonal coordinated turn yaw PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "ecl_yaw_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

using matrix::wrap_pi;

float ECL_YawController::control_attitude(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.euler_pitch_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_constrained))) {

		return _body_rate_setpoint;
	}

	float constrained_roll;
	bool inverted = false;

	/* roll is used as feedforward term and inverted flight needs to be considered */
	if (fabsf(ctl_data.roll) < math::radians(90.0f)) {
		/* not inverted, but numerically still potentially close to infinity */
		constrained_roll = math::constrain(ctl_data.roll, math::radians(-80.0f), math::radians(80.0f));

	} else {
		inverted = true;

		// inverted flight, constrain on the two extremes of -pi..+pi to avoid infinity
		//note: the ranges are extended by 10 deg here to avoid numeric resolution effects
		if (ctl_data.roll > 0.0f) {
			/* right hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(100.0f), math::radians(180.0f));

		} else {
			/* left hemisphere */
			constrained_roll = math::constrain(ctl_data.roll, math::radians(-180.0f), math::radians(-100.0f));
		}
	}

	constrained_roll = math::constrain(constrained_roll, -fabsf(ctl_data.roll_setpoint), fabsf(ctl_data.roll_setpoint));


	if (!inverted) {
		/* Calculate desired yaw rate from coordinated turn constraint / (no side forces) */
		_euler_rate_setpoint = tanf(constrained_roll) * cosf(ctl_data.pitch) * CONSTANTS_ONE_G / ctl_data.airspeed_constrained;

		/* Transform setpoint to body angular rates (jacobian) */
		const float yaw_body_rate_setpoint_raw = -sinf(ctl_data.roll) * ctl_data.euler_pitch_rate_setpoint +
				cosf(ctl_data.roll) * cosf(ctl_data.pitch) * _euler_rate_setpoint;
		_body_rate_setpoint = math::constrain(yaw_body_rate_setpoint_raw, -_max_rate, _max_rate);
	}

	if (!PX4_ISFINITE(_body_rate_setpoint)) {
		PX4_WARN("yaw rate sepoint not finite");
		_body_rate_setpoint = 0.0f;
	}

	/* Chain-wing heading hold for yaw stabilization via differential thrust.
	 *
	 * The coordinated-turn formula (line 88) computes yaw rate from
	 * tan(roll) * g / airspeed.  At low airspeed this value is large
	 * even for tiny roll angles, but on the ground the aircraft has full
	 * differential-thrust authority.  The result is aggressive ground
	 * spinning whenever the roll stick is not perfectly centred.
	 *
	 * Fix: scale the coordinated-turn body-rate by airspeed_ratio² so it
	 * fades to near-zero on the ground, and clamp the heading-hold gain
	 * floor to 0.5 so heading corrections remain authoritative.
	 */
	if (_heading_hold_gain > FLT_EPSILON &&
	    PX4_ISFINITE(ctl_data.yaw_setpoint) && PX4_ISFINITE(ctl_data.yaw)) {

		const float airspeed_ratio = math::constrain(
						     ctl_data.airspeed_constrained / math::max(_trim_airspeed, 1.f),
						     0.0f, 1.0f);

		/* 1. Scale coordinated-turn contribution by airspeed_ratio² to
		 *    suppress spurious yaw commands at low airspeed / on ground. */
		_body_rate_setpoint *= airspeed_ratio * airspeed_ratio;

		/* 2. Heading-hold correction with raised minimum gain floor.
		 *    Floor 0.5 → minimum gain = 0.25 (vs old 0.01 with floor 0.1).
		 *    This ensures the heading hold can always overpower the
		 *    (now-scaled) coordinated-turn residual. */
		const float heading_error = wrap_pi(ctl_data.yaw_setpoint - ctl_data.yaw);
		const float heading_gain_ratio = math::constrain(airspeed_ratio, 0.5f, 1.0f);
		const float scaled_gain = _heading_hold_gain * heading_gain_ratio * heading_gain_ratio;

		const float heading_rate_correction = heading_error * scaled_gain;
		_body_rate_setpoint += math::constrain(heading_rate_correction, -_max_rate, _max_rate);
		_body_rate_setpoint = math::constrain(_body_rate_setpoint, -_max_rate, _max_rate);
	}

	return _body_rate_setpoint;
}
