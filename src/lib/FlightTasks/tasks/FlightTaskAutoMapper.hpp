/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskAutoMapper.hpp
 *
 * Flight task for autonomous, gps driven mode. The vehicle flies
 * along a straight line in between waypoints.
 */

#pragma once

#include "FlightTaskAuto.hpp"


class FlightTaskAutoMapper : public FlightTaskAuto
{
public:
	FlightTaskAutoMapper() = default;
	virtual ~FlightTaskAutoMapper() = default;
	bool activate() override;
	bool update() override;

protected:

	float _alt_above_ground{0.0f}; /**< If home provided, then it is altitude above home, otherwise it is altitude above local position reference. */



	// DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTaskAuto,

	// 			       )

	virtual void _generateSetpoints() = 0; /**< Generate velocity and position setpoint for following line. */

	void _generateIdleSetpoints();
	void _generateLandSetpoints();
	void _generateVelocitySetpoints();
	void _generateTakeoffSetpoints();

	void _updateAltitudeAboveGround(); /**< Computes altitude above ground based on sensors available. */
	void updateParams() override; /**< See ModuleParam class */

private:
	void _reset(); /**< Resets member variables to current vehicle state */
	WaypointType _type_previous{WaypointType::idle}; /**< Previous type of current target triplet. */

};
