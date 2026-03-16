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
 * @file chainwing_slave_params.c
 *
 * Parameters for the chain-wing slave controller.
 *
 * The slave controller uses PD control on the hinge angle to compute
 * elevator trim corrections that maintain coplanarity between units.
 */

/**
 * Slave controller enable
 *
 * Enable the chain-wing slave position trim controller.
 * 0 = disabled, 1 = enabled.
 *
 * @boolean
 * @group Chain-Wing Slave
 */
PARAM_DEFINE_INT32(CW_SLV_EN, 0);

/**
 * Hinge PD proportional gain
 *
 * Proportional gain for the hinge angle PD controller.
 * Controls how aggressively the slave corrects hinge deflection.
 * Higher values give faster response but may cause oscillation.
 *
 * δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
 *
 * @unit 1/rad
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @increment 0.05
 * @group Chain-Wing Slave
 */
PARAM_DEFINE_FLOAT(CW_SLV_KP, 0.3f);

/**
 * Hinge PD derivative gain
 *
 * Derivative gain for the hinge angle PD controller.
 * Damps hinge oscillation. Higher values add more damping.
 *
 * δ_trim = Kp × θ_hinge + Kd × θ̇_hinge
 *
 * @unit s/rad
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.01
 * @group Chain-Wing Slave
 */
PARAM_DEFINE_FLOAT(CW_SLV_KD, 0.05f);

/**
 * Maximum trim deflection
 *
 * Maximum allowed trim correction from the slave PD controller.
 * Limits the slave elevon travel reserved for position trim.
 * The remaining travel is used for the master's overall pitch command.
 *
 * At 30% travel (0.3), the elevon generates approximately 24.6 N·m
 * of pitch moment, sufficient for correcting hinge deflections up to ±10°.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Chain-Wing Slave
 */
PARAM_DEFINE_FLOAT(CW_SLV_TRIM_MAX, 0.3f);

/**
 * Hinge rate low-pass filter frequency
 *
 * Corner frequency for the low-pass filter applied to the
 * angular velocity signal used for hinge rate estimation.
 * Set to 0 to disable filtering.
 *
 * @unit Hz
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 1.0
 * @group Chain-Wing Slave
 */
PARAM_DEFINE_FLOAT(CW_SLV_LP_FREQ, 10.0f);
