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
 * @file ChainwingMaster.cpp
 *
 * Chain-wing master communication module.
 *
 * Runs on the master (center body) Pixhawk in 3-Pixhawk distributed
 * architecture.  At 10 Hz, publishes CW_CMD (id=43) via debug_array:
 *
 *   data[0] = pitch torque command   (vehicle_torque_setpoint.xyz[1])
 *   data[1] = forward thrust command (vehicle_thrust_setpoint.xyz[0])
 *   data[2] = roll torque command    (vehicle_torque_setpoint.xyz[0])
 *   data[3] = current roll attitude  (euler phi from vehicle_attitude.q)
 *
 * The MAVLink bridge (mavlink start -d /dev/ttyS2 -b 921600 -m custom)
 * automatically forwards debug_array messages as DEBUG_FLOAT_ARRAY
 * MAVLink packets over UART to the slave controllers.
 *
 * Also receives CW_HINGE (id=42) from slaves for monitoring/logging.
 */

#include "ChainwingMaster.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

ChainwingMaster::ChainwingMaster() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	updateParams();
}

bool ChainwingMaster::init()
{
	ScheduleOnInterval(100000_us); // 10 Hz — matches MAVLink stream rate
	PX4_INFO("chainwing_master started (10Hz), CW_MST_EN=%d", _param_enable.get());
	return true;
}

void ChainwingMaster::Run()
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

	// Check if master module is enabled
	if (_param_enable.get() == 0) {
		return;
	}

	// Publish CW_CMD to slaves
	publishCommand();

	// Process CW_HINGE feedback from slaves
	processSlaveStatus();
}

void ChainwingMaster::publishCommand()
{
	debug_array_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.id = CW_MASTER_CMD_ID;
	strncpy(cmd.name, "CW_CMD", sizeof(cmd.name));
	cmd.name[sizeof(cmd.name) - 1] = '\0';

	// Read pitch and roll torque commands from rate controller output
	vehicle_torque_setpoint_s torque{};

	if (_vehicle_torque_setpoint_sub.copy(&torque)) {
		cmd.data[0] = math::constrain(torque.xyz[1], -1.0f, 1.0f); // pitch torque
		cmd.data[2] = math::constrain(torque.xyz[0], -1.0f, 1.0f); // roll torque
	}

	// Read throttle from thrust setpoint
	vehicle_thrust_setpoint_s thrust{};

	if (_vehicle_thrust_setpoint_sub.copy(&thrust)) {
		cmd.data[1] = math::constrain(thrust.xyz[0], 0.0f, 1.0f);  // forward thrust
	}

	// Read current roll attitude for relative hinge angle calculation
	vehicle_attitude_s att{};

	if (_vehicle_attitude_sub.copy(&att)) {
		const matrix::Quatf q(att.q);
		const matrix::Eulerf euler(q);
		cmd.data[3] = math::constrain(euler.phi(), -M_PI_F, M_PI_F); // roll (rad)
	}

	_debug_array_pub.publish(cmd);
}

void ChainwingMaster::processSlaveStatus()
{
	debug_array_s status{};

	while (_debug_array_sub.update(&status)) {
		if (status.id == CW_HINGE_STATUS_ID && strncmp(status.name, "CW_HINGE", 8) == 0) {
			_slave_hinge_left  = status.data[0];
			_slave_hinge_right = status.data[1];
			_slave_trim_left   = status.data[4];
			_slave_trim_right  = status.data[5];
			_last_hinge_update = hrt_absolute_time();
			_hinge_valid = true;
		}
	}

	// Timeout check
	if (_hinge_valid && hrt_elapsed_time(&_last_hinge_update) > HINGE_TIMEOUT_US) {
		_hinge_valid = false;
	}
}

int ChainwingMaster::print_status()
{
	PX4_INFO("chainwing_master status:");
	PX4_INFO("  enabled: %s", _param_enable.get() ? "YES" : "NO");
	PX4_INFO("  slave hinge valid: %s", _hinge_valid ? "YES" : "NO (timeout or no data)");

	if (_hinge_valid) {
		PX4_INFO("  slave hinge L: %.3f rad  R: %.3f rad",
			 (double)_slave_hinge_left, (double)_slave_hinge_right);
		PX4_INFO("  slave trim  L: %.3f      R: %.3f",
			 (double)_slave_trim_left, (double)_slave_trim_right);
	}

	// Show current command values
	vehicle_attitude_s att{};

	if (_vehicle_attitude_sub.copy(&att)) {
		const matrix::Quatf q(att.q);
		const matrix::Eulerf euler(q);
		PX4_INFO("  master roll attitude: %.3f rad (%.1f deg)",
			 (double)euler.phi(), (double)math::degrees(euler.phi()));
	}

	return 0;
}

int ChainwingMaster::task_spawn(int argc, char *argv[])
{
	ChainwingMaster *instance = new ChainwingMaster();

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

int ChainwingMaster::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ChainwingMaster::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Chain-wing master communication module for 3-Pixhawk distributed architecture.

Runs on the master (center body) Pixhawk at 10 Hz. Reads the current
flight control state and publishes it as CW_CMD (DEBUG_FLOAT_ARRAY id=43)
via the MAVLink bridge to slave controllers on left/right wing units.

#### CW_CMD Protocol (Master → Slave)
| Field   | Content                 | Unit      | Source                          |
|---------|-------------------------|-----------|---------------------------------|
| data[0] | pitch torque command    | [-1, 1]   | vehicle_torque_setpoint.xyz[1]  |
| data[1] | forward thrust command  | [0, 1]    | vehicle_thrust_setpoint.xyz[0]  |
| data[2] | roll torque command     | [-1, 1]   | vehicle_torque_setpoint.xyz[0]  |
| data[3] | master roll attitude    | rad       | vehicle_attitude.q → euler.phi  |

#### CW_HINGE Protocol (Slave → Master)
| Field   | Content                 | Unit      |
|---------|-------------------------|-----------|
| data[0] | left hinge angle        | rad       |
| data[1] | right hinge angle       | rad       |
| data[2] | left hinge rate         | rad/s     |
| data[3] | right hinge rate        | rad/s     |
| data[4] | left trim output        | [-1, 1]   |
| data[5] | right trim output       | [-1, 1]   |
| data[6] | data valid flag         | 0.0/1.0   |

#### Hardware Setup (Master Pixhawk)
```
# UART to left slave (TELEM2)
mavlink start -d /dev/ttyS2 -b 921600 -r 4000 -m custom
mavlink stream -d /dev/ttyS2 -s DEBUG_FLOAT_ARRAY -r 10

# UART to right slave (SERIAL5)
mavlink start -d /dev/ttyS6 -b 921600 -r 4000 -m custom
mavlink stream -d /dev/ttyS6 -s DEBUG_FLOAT_ARRAY -r 10

# Start master module
chainwing_master start
```
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("chainwing_master", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int chainwing_master_main(int argc, char *argv[])
{
	return ChainwingMaster::main(argc, argv);
}
