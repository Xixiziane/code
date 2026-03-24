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
 * Control law (corrects relative ROLL between wing units):
 *   δ_trim = Kp × θ_hinge + Kd × θ̇_hinge  (hinge axis = X = roll)
 *   δ_total = clamp(δ_master + δ_trim, -1.0, 1.0)
 *
 * Communication architecture (hardware):
 *   Master → Slave: UART + MAVLink v2 (overall pitch/throttle/roll commands)
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

	// Publish hinge status (includes trim values for GZMixingInterfaceServo or PWM overlay)
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

	// Hardware PWM trim overlay (Scheme C):
	// Read actuator_servos from control_allocator, add trim to elevon channels,
	// and re-publish so that PWMOut receives the trimmed values.
	// This replaces GZMixingInterfaceServo for real hardware.
	if (_param_pwm_enable.get() != 0 && _ref_initialized) {
		actuator_servos_s servos{};

		if (_actuator_servos_sub.copy(&servos)) {
			// Apply hinge trim correction to left and right elevon channels
			servos.control[0] = math::constrain(servos.control[0] + trim_left, -1.0f, 1.0f);
			servos.control[2] = math::constrain(servos.control[2] + trim_right, -1.0f, 1.0f);

			servos.timestamp = hrt_absolute_time();
			_actuator_servos_pub.publish(servos);
		}
	}

	// MAVLink communication: publish hinge data and receive master commands
	if (_param_comm_enable.get() != 0) {
		publishDebugArray();
		processMasterCommands();
	}
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
	// The hinge axis is aligned with X (forward direction), so relative ROLL
	// rotation between units around the hinge is sensed as roll rate by the IMU.
	const float roll_rate = angular_vel.xyz[0];

	// Initialize reference on first valid attitude
	if (!_ref_initialized && attitude_valid) {
		// Extract roll from quaternion (hinge axis = X = roll axis)
		const matrix::Quatf q(attitude.q);
		const matrix::Eulerf euler(q);
		_roll_ref = euler.phi();
		_ref_initialized = true;
		PX4_INFO("Slave reference roll initialized: %.2f deg", (double)math::degrees(_roll_ref));
	}

	if (!_ref_initialized) {
		return;
	}

	// Low-pass filter coefficient for rate signal
	const float lp_freq = _param_lp_freq.get();
	const float alpha = (lp_freq > 0.0f) ? dt / (dt + 1.0f / (2.0f * M_PI_F * lp_freq)) : 1.0f;

	// IMU integration method for relative hinge angle estimation:
	// In a real multi-controller setup, each slave has its own IMU.
	// The master sends its roll via MAVLink.
	// Relative hinge angle = slave_roll - master_roll (rotation around X-axis).
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

	// Attitude-based correction: if we have valid attitude, use roll deviation
	// from reference as a coarse hinge angle estimate (complementary filter)
	if (attitude_valid) {
		const matrix::Quatf q(attitude.q);
		const matrix::Eulerf euler(q);
		const float roll_error = euler.phi() - _roll_ref;

		// Blend IMU-integrated angle with attitude-based estimate
		// This corrects long-term drift while keeping high-frequency response
		const float cf_alpha = 0.02f; // complementary filter weight (low = trust integration more)
		_hinge_angle_left = (1.0f - cf_alpha) * _hinge_angle_left + cf_alpha * roll_error;
		_hinge_angle_right = (1.0f - cf_alpha) * _hinge_angle_right + cf_alpha * (-roll_error);
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

void ChainwingSlave::publishDebugArray()
{
	// Pack hinge status into DEBUG_FLOAT_ARRAY for MAVLink transmission
	// This gets automatically bridged to MAVLink by the mavlink module's
	// DEBUG_FLOAT_ARRAY stream when running in onboard mode.
	debug_array_s dbg{};
	dbg.timestamp = hrt_absolute_time();
	dbg.id = CW_HINGE_STATUS_ID;
	strncpy(dbg.name, "CW_HINGE", sizeof(dbg.name));
	dbg.name[sizeof(dbg.name) - 1] = '\0';

	dbg.data[0] = _hinge_angle_left;
	dbg.data[1] = _hinge_angle_right;
	dbg.data[2] = _hinge_rate_left;
	dbg.data[3] = _hinge_rate_right;
	dbg.data[4] = computeTrim(_hinge_angle_left, _hinge_rate_left);
	dbg.data[5] = computeTrim(_hinge_angle_right, _hinge_rate_right);
	dbg.data[6] = _ref_initialized ? 1.0f : 0.0f;

	_debug_array_pub.publish(dbg);
}

void ChainwingSlave::processMasterCommands()
{
	// Read master commands from debug_array (bridged from MAVLink receiver)
	// The MAVLink receiver automatically converts incoming DEBUG_FLOAT_ARRAY
	// messages to the debug_array uORB topic.
	debug_array_s cmd{};

	while (_debug_array_sub.update(&cmd)) {
		// Filter: only process messages with our command ID
		if (cmd.id == CW_MASTER_CMD_ID && strncmp(cmd.name, "CW_CMD", 6) == 0) {
			_master_pitch_cmd = math::constrain(cmd.data[0], -1.0f, 1.0f);
			_master_throttle = math::constrain(cmd.data[1], 0.0f, 1.0f);
			_master_roll_cmd = math::constrain(cmd.data[2], -1.0f, 1.0f);
			_last_master_cmd = hrt_absolute_time();
			_master_cmd_valid = true;
		}
	}

	// Timeout: invalidate master command if not received for 500ms
	if (_master_cmd_valid && hrt_elapsed_time(&_last_master_cmd) > MASTER_CMD_TIMEOUT_US) {
		_master_cmd_valid = false;
		PX4_WARN("Master command timeout");
	}
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
	PX4_INFO("  PWM trim overlay: %s", (_param_pwm_enable.get() != 0) ? "ENABLED (hardware)" : "DISABLED (sim)");
	PX4_INFO("  Communication: %s", (_param_comm_enable.get() != 0) ? "ENABLED" : "DISABLED");
	PX4_INFO("  Reference initialized: %s", _ref_initialized ? "YES" : "NO");
	PX4_INFO("  Hinge angles: left=%.3f deg, right=%.3f deg",
		 (double)math::degrees(_hinge_angle_left),
		 (double)math::degrees(_hinge_angle_right));
	PX4_INFO("  Hinge rates:  left=%.3f, right=%.3f rad/s",
		 (double)_hinge_rate_left, (double)_hinge_rate_right);
	PX4_INFO("  PD gains: Kp=%.2f, Kd=%.2f, max_trim=%.2f",
		 (double)_param_kp.get(), (double)_param_kd.get(), (double)_param_trim_max.get());

	if (_param_comm_enable.get() != 0) {
		PX4_INFO("  Master cmd valid: %s", _master_cmd_valid ? "YES" : "NO");
		PX4_INFO("  Master pitch=%.2f, throttle=%.2f, roll=%.2f",
			 (double)_master_pitch_cmd, (double)_master_throttle, (double)_master_roll_cmd);
	}

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
When CW_SLV_COMM_EN=1, enables inter-controller communication via
MAVLink DEBUG_FLOAT_ARRAY messages:

Slave → Master (id=42, name="CW_HINGE"):
  data[0-1]: hinge angles (rad)
  data[2-3]: hinge rates (rad/s)
  data[4-5]: trim values (normalized)
  data[6]:   data_valid flag

Master → Slave (id=43, name="CW_CMD"):
  data[0]: pitch command (normalized [-1, 1])
  data[1]: throttle (normalized [0, 1])
  data[2]: roll command (normalized [-1, 1])

Requires: mavlink start -d /dev/ttyS2 -b 921600 -m onboard

### Control Law
  δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
  δ_total = clamp(δ_master + δ_trim, -1.0, 1.0)

### Implementation
The module runs at 50 Hz and publishes ChainwingHingeStatus containing
the computed trim values.

In simulation (SITL): GZMixingInterfaceServo reads the trim values and
adds them to the Gazebo servo outputs (servo_0 and servo_2).

On hardware (CW_SLV_PWM_EN=1): ChainwingSlave directly modifies the
actuator_servos topic by reading control_allocator output, adding trim
to control[0] (left elevon) and control[2] (right elevon), and
re-publishing for PWMOut to consume.

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
