/****************************************************************************
 *
 *   Copyright (c) 2018-20 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/propulsion_system_info.h>
#include <uORB/topics/propulsion_id_info.h>

using namespace time_literals;

class PropulsionID : public ModuleBase<PropulsionID>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	PropulsionID();

	virtual ~PropulsionID();

	static int print_usage(const char *reason = nullptr);
	static int custom_command(int argc, char *argv[]);

	static int task_spawn(int argc, char *argv[]);

	int start();

private:

	void Run() override;

	uORB::Publication<propulsion_id_info_s> _propulsion_id_info_pub{ORB_ID(propulsion_id_info)};

	uORB::Subscription _propulsion_system_info_sub{ORB_ID(propulsion_system_info)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// TODO: are we going to use parameters for individual motor flight times?

	// DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::params::SENS_IMU_TEMP_FF>) _param_sens_imu_temp_ff,
	// 	(ParamFloat<px4::params::SENS_IMU_TEMP_I>)  _param_sens_imu_temp_i,
	// 	(ParamFloat<px4::params::SENS_IMU_TEMP_P>)  _param_sens_imu_temp_p,
	// 	(ParamInt<px4::params::SENS_TEMP_ID>)       _param_sens_temp_id,
	// 	(ParamFloat<px4::params::SENS_IMU_TEMP>)    _param_sens_imu_temp
	// )
};
