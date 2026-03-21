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
 * @file ChainwingSlave.hpp
 *
 * Chain-wing slave controller module.
 *
 * Estimates relative hinge angle between master and slave units using
 * IMU integration, and computes elevator trim corrections to maintain
 * coplanarity. In simulation, reads hinge joint angles directly from
 * the Gazebo joint state topic.
 *
 * Communication with master controller via UART + MAVLink protocol:
 *   - Master sends overall pitch/throttle commands
 *   - Slave applies: δ_total = δ_master + δ_trim
 *   - δ_trim = Kp * θ_hinge + Kd * θ̇_hinge
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/chainwing_hinge_status.h>
#include <uORB/topics/debug_array.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

class ChainwingSlave : public ModuleBase<ChainwingSlave>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	// Debug array IDs for inter-controller communication protocol
	static constexpr uint16_t CW_HINGE_STATUS_ID = 42;  ///< Slave → Master: hinge status
	static constexpr uint16_t CW_MASTER_CMD_ID = 43;    ///< Master → Slave: commands
	static constexpr hrt_abstime MASTER_CMD_TIMEOUT_US = 500000; ///< 500ms master command timeout

	ChainwingSlave();
	~ChainwingSlave() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	/**
	 * Update hinge angle estimates from IMU integration.
	 * In hardware: integrates angular velocity difference between master and slave IMU.
	 * In simulation: this provides a complementary filter on top of direct joint readings.
	 */
	void updateHingeEstimate(float dt);

	/**
	 * Compute PD trim correction for a given hinge angle and rate.
	 * @param angle  Hinge deflection from nominal (rad)
	 * @param rate   Hinge angular rate (rad/s)
	 * @return       Normalized trim correction [-1, 1]
	 */
	float computeTrim(float angle, float rate);

	/**
	 * Publish hinge status via debug_array for MAVLink transmission.
	 * Uses DEBUG_FLOAT_ARRAY (id=42, name="CW_HINGE") to transmit:
	 *   data[0]: hinge_angle_left   (rad)
	 *   data[1]: hinge_angle_right  (rad)
	 *   data[2]: hinge_rate_left    (rad/s)
	 *   data[3]: hinge_rate_right   (rad/s)
	 *   data[4]: trim_left          (normalized)
	 *   data[5]: trim_right         (normalized)
	 *   data[6]: data_valid         (1.0 or 0.0)
	 */
	void publishDebugArray();

	/**
	 * Process master commands received via debug_array MAVLink bridge.
	 * Reads DEBUG_FLOAT_ARRAY (id=43, name="CW_CMD") containing:
	 *   data[0]: master_pitch_cmd   (normalized [-1, 1])
	 *   data[1]: master_throttle    (normalized [0, 1])
	 *   data[2]: master_roll_cmd    (normalized [-1, 1])
	 */
	void processMasterCommands();

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _actuator_servos_sub{ORB_ID(actuator_servos)};
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};   ///< Master commands via MAVLink bridge

	// Publications
	uORB::Publication<chainwing_hinge_status_s> _hinge_status_pub{ORB_ID(chainwing_hinge_status)};
	uORB::Publication<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};  ///< Hinge status via MAVLink bridge

	// State variables for IMU integration
	float _hinge_angle_left{0.0f};   ///< Estimated left hinge angle (rad)
	float _hinge_angle_right{0.0f};  ///< Estimated right hinge angle (rad)
	float _hinge_rate_left{0.0f};    ///< Left hinge angular rate (rad/s)
	float _hinge_rate_right{0.0f};   ///< Right hinge angular rate (rad/s)

	// Master commands received via MAVLink
	float _master_pitch_cmd{0.0f};   ///< Master pitch command (normalized [-1, 1])
	float _master_throttle{0.0f};    ///< Master throttle command (normalized [0, 1])
	float _master_roll_cmd{0.0f};    ///< Master roll command (normalized [-1, 1])
	hrt_abstime _last_master_cmd{0}; ///< Timestamp of last received master command
	bool _master_cmd_valid{false};   ///< True if master command received within timeout

	// Reference attitude (captured at startup for IMU integration baseline)
	float _pitch_ref{0.0f};
	bool  _ref_initialized{false};

	// Previous timestamp for dt calculation
	hrt_abstime _last_run{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CW_SLV_KP>)      _param_kp,
		(ParamFloat<px4::params::CW_SLV_KD>)      _param_kd,
		(ParamFloat<px4::params::CW_SLV_TRIM_MAX>) _param_trim_max,
		(ParamFloat<px4::params::CW_SLV_LP_FREQ>)  _param_lp_freq,
		(ParamInt<px4::params::CW_SLV_EN>)         _param_enable,
		(ParamInt<px4::params::CW_SLV_COMM_EN>)    _param_comm_enable
	)
};
