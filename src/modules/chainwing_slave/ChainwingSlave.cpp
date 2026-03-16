/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file ChainwingSlave.cpp
 *
 * Chain-wing slave controller: estimates hinge angle via IMU integration
 * and computes elevator trim corrections to maintain coplanarity between
 * master and slave units.
 *
 * Control law:
 *   δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
 *   δ_total = clamp(δ_master + δ_trim, -1.0, 1.0)
 *
 * Communication architecture (hardware):
 *   Master → Slave: UART + MAVLink v2 (overall pitch/throttle commands)
 *   Slave → Master: UART + MAVLink v2 (hinge status feedback)
 *
 * In simulation (GZ SITL):
 *   Single PX4 instance controls all 3 units.
 *   Hinge angles estimated from IMU angular velocity integration
 *   (complementary filter with attitude feedback).
 */

#include "ChainwingSlave.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

ChainwingSlave::ChainwingSlave() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	updateParams();
}

bool ChainwingSlave::init()
{
	ScheduleOnInterval(20000_us); // 50 Hz
	return true;
}

void ChainwingSlave::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Check if slave controller is enabled
	if (_param_enable.get() == 0) {
		return;
	}

	// Calculate dt
	const hrt_abstime now = hrt_absolute_time();

	if (_last_run == 0) {
		_last_run = now;
		return;
	}

	// Calculate dt, constrained to [1ms, 100ms] to handle timing edge cases
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.1f);
	_last_run = now;

	// Update hinge angle estimate from IMU
	updateHingeEstimate(dt);

	// Compute trim corrections
	const float trim_left = computeTrim(_hinge_angle_left, _hinge_rate_left);
	const float trim_right = computeTrim(_hinge_angle_right, _hinge_rate_right);

	// Publish hinge status (includes trim values for GZMixingInterfaceServo)
	chainwing_hinge_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.hinge_angle_left = _hinge_angle_left;
	status.hinge_angle_right = _hinge_angle_right;
	status.hinge_rate_left = _hinge_rate_left;
	status.hinge_rate_right = _hinge_rate_right;
	status.trim_left = trim_left;
	status.trim_right = trim_right;
	status.data_valid = _ref_initialized;
	_hinge_status_pub.publish(status);
}

void ChainwingSlave::updateHingeEstimate(float dt)
{
	// Read current angular velocity from IMU
	vehicle_angular_velocity_s angular_vel{};

	if (!_vehicle_angular_velocity_sub.copy(&angular_vel)) {
		return;
	}

	// Read current attitude for complementary filter
	vehicle_attitude_s attitude{};
	bool attitude_valid = _vehicle_attitude_sub.copy(&attitude);

	// Extract roll rate (X-axis in body frame).
	// The hinge axis is aligned with X (forward direction), so relative pitch
	// rotation between units around the hinge is sensed as roll rate by the IMU.
	const float roll_rate = angular_vel.xyz[0];

	// Initialize reference on first valid attitude
	if (!_ref_initialized && attitude_valid) {
		// Extract pitch from quaternion
		const matrix::Quatf q(attitude.q);
		const matrix::Eulerf euler(q);
		_pitch_ref = euler.theta();
		_ref_initialized = true;
		PX4_INFO("Slave reference pitch initialized: %.2f deg", (double)math::degrees(_pitch_ref));
	}

	if (!_ref_initialized) {
		return;
	}

	// Low-pass filter coefficient for rate signal
	const float lp_freq = _param_lp_freq.get();
	const float alpha = (lp_freq > 0.0f) ? dt / (dt + 1.0f / (2.0f * M_PI_F * lp_freq)) : 1.0f;

	// IMU integration method for relative hinge angle estimation:
	// In a real multi-controller setup, each slave has its own IMU.
	// The master sends its pitch via MAVLink.
	// Relative angle = slave_pitch - master_pitch.
	//
	// In single-instance simulation, we estimate from roll rate
	// (the hinge axis is approximately the X/forward axis).
	// The hinge angle is integrated from the differential angular velocity
	// with a complementary filter to prevent drift.

	// Update hinge rates with low-pass filter
	_hinge_rate_left = (1.0f - alpha) * _hinge_rate_left + alpha * roll_rate;
	// Right hinge uses negated roll rate: positive roll rate (left wing up)
	// corresponds to negative hinge deflection for the right slave unit.
	_hinge_rate_right = (1.0f - alpha) * _hinge_rate_right + alpha * (-roll_rate);

	// Integrate angle with decay factor (complementary filter).
	// tau_decay prevents unbounded drift from integration errors.
	// At tau=2s, integrated angle decays to 37% after 2s without new input.
	const float tau_decay = 2.0f; // seconds
	const float decay = expf(-dt / tau_decay);

	_hinge_angle_left = decay * (_hinge_angle_left + _hinge_rate_left * dt);
	_hinge_angle_right = decay * (_hinge_angle_right + _hinge_rate_right * dt);

	// Attitude-based correction: if we have valid attitude, use pitch deviation
	// from reference as a coarse hinge angle estimate (complementary filter)
	if (attitude_valid) {
		const matrix::Quatf q(attitude.q);
		const matrix::Eulerf euler(q);
		const float pitch_error = euler.theta() - _pitch_ref;

		// Blend IMU-integrated angle with attitude-based estimate
		// This corrects long-term drift while keeping high-frequency response
		const float cf_alpha = 0.02f; // complementary filter weight (low = trust integration more)
		_hinge_angle_left = (1.0f - cf_alpha) * _hinge_angle_left + cf_alpha * pitch_error;
		_hinge_angle_right = (1.0f - cf_alpha) * _hinge_angle_right + cf_alpha * (-pitch_error);
	}
}

float ChainwingSlave::computeTrim(float angle, float rate)
{
	// PD controller: δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
	const float kp = _param_kp.get();
	const float kd = _param_kd.get();
	const float trim_max = _param_trim_max.get();

	float trim = kp * angle + kd * rate;

	// Clamp trim to maximum allowed
	trim = math::constrain(trim, -trim_max, trim_max);

	return trim;
}

int ChainwingSlave::task_spawn(int argc, char *argv[])
{
	ChainwingSlave *instance = new ChainwingSlave();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ChainwingSlave::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ChainwingSlave::print_status()
{
	PX4_INFO("Chain-wing slave controller");
	PX4_INFO("  Enabled: %s", (_param_enable.get() != 0) ? "YES" : "NO");
	PX4_INFO("  Reference initialized: %s", _ref_initialized ? "YES" : "NO");
	PX4_INFO("  Hinge angles: left=%.3f deg, right=%.3f deg",
		 (double)math::degrees(_hinge_angle_left),
		 (double)math::degrees(_hinge_angle_right));
	PX4_INFO("  Hinge rates:  left=%.3f, right=%.3f rad/s",
		 (double)_hinge_rate_left, (double)_hinge_rate_right);
	PX4_INFO("  PD gains: Kp=%.2f, Kd=%.2f, max_trim=%.2f",
		 (double)_param_kp.get(), (double)_param_kd.get(), (double)_param_trim_max.get());
	return 0;
}

int ChainwingSlave::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Chain-wing slave controller module for maintaining coplanarity between
linked fixed-wing units.

Estimates the relative hinge angle between master and slave units using
IMU angular velocity integration (with complementary filter correction).
Computes PD-based elevator trim corrections that are applied on top of
the master's overall pitch command.

### Communication (Hardware)
Master → Slave: UART + MAVLink v2 protocol
  - Overall pitch setpoint and throttle command
Slave → Master: UART + MAVLink v2 protocol
  - Hinge status feedback

### Control Law
  δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
  δ_total = clamp(δ_master + δ_trim, -1.0, 1.0)

### Implementation
The module runs at 50 Hz and publishes ChainwingHingeStatus containing
the computed trim values. The GZMixingInterfaceServo reads these trim
values and adds them to the slave elevator servo outputs (servo_0 for
left slave, servo_2 for right slave).

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("chainwing_slave", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int chainwing_slave_main(int argc, char *argv[])
{
	return ChainwingSlave::main(argc, argv);
}
