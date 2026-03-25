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
 * @file ChainwingMaster.hpp
 *
 * Chain-wing master communication module.
 *
 * Runs on the master (center body) Pixhawk in 3-Pixhawk distributed
 * architecture.  Publishes CW_CMD messages at 10 Hz containing:
 *   data[0] = pitch torque command (normalized)
 *   data[1] = throttle command (normalized)
 *   data[2] = roll torque command (normalized)
 *   data[3] = current roll attitude (rad)
 *
 * The MAVLink bridge forwards these as DEBUG_FLOAT_ARRAY packets over
 * UART to slave controllers on the left and right wing units.
 *
 * Also receives CW_HINGE status from slaves for telemetry/logging.
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.h>
#include <uORB/Subscription.h>
#include <uORB/SubscriptionInterval.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class ChainwingMaster : public ModuleBase<ChainwingMaster>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	ChainwingMaster();
	~ChainwingMaster() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	/** Pack and publish CW_CMD debug_array message */
	void publishCommand();

	/** Read and process CW_HINGE messages from slaves */
	void processSlaveStatus();

	// ── Constants ──────────────────────────────────────────────────────
	static constexpr uint16_t CW_MASTER_CMD_ID    = 43;   ///< CW_CMD message id
	static constexpr uint16_t CW_HINGE_STATUS_ID  = 42;   ///< CW_HINGE message id
	static constexpr hrt_abstime HINGE_TIMEOUT_US  = 500000; ///< 500 ms

	// ── Subscriptions ──────────────────────────────────────────────────
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};

	// ── Publications ───────────────────────────────────────────────────
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};

	// ── Slave hinge feedback (received via CW_HINGE) ──────────────────
	float _slave_hinge_left{0.0f};
	float _slave_hinge_right{0.0f};
	float _slave_trim_left{0.0f};
	float _slave_trim_right{0.0f};
	hrt_abstime _last_hinge_update{0};
	bool  _hinge_valid{false};

	// ── Parameters ─────────────────────────────────────────────────────
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CW_MST_EN>) _param_enable
	)
};
