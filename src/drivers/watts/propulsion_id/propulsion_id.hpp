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
#include <uORB/SubscriptionMultiArray.hpp>
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

	propulsion_id_info_s get_propulsion_id_info();

	static constexpr uint32_t SAMPLE_RATE{1}; // samples per second
	static constexpr uint32_t TIMEOUT_US{1000000}; // Time before driver reports an eeprom as missing

	static constexpr uint32_t PRISM_AIRFRAME_ID_QUAD{9500};
	static constexpr uint32_t PRISM_AIRFRAME_ID_X8{9501};


	uORB::Publication<propulsion_id_info_s> _propulsion_id_info_pub{ORB_ID::propulsion_id_info};

	// uORB::Subscription _propulsion_system_info_sub{ORB_ID(propulsion_system_info)};
	uORB::SubscriptionMultiArray<propulsion_system_info_s> _propulsion_system_info_subs{ORB_ID::propulsion_system_info};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// TODO: are we going to use parameters for individual motor flight times?

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,
		(ParamInt<px4::params::PROPULSION_GROUP>) _param_prop_group
	)
};
