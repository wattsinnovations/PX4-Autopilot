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

#include "propulsion_id.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

PropulsionID::PropulsionID() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{

}

PropulsionID::~PropulsionID()
{
}

void PropulsionID::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		ModuleParams::updateParams();
	}


	auto info = get_propulsion_id_info();

	info.timestamp = hrt_absolute_time();


	_propulsion_id_info_pub.publish(info);
}

propulsion_id_info_s PropulsionID::get_propulsion_id_info()
{
	propulsion_id_info_s info = {};
	// bool deny_arm;
	// bool mismatch;
	// bool connected[4];

	// Check if any are missing
	int advertised_count = _propulsion_system_info_subs.advertised_count();
	if (advertised_count != 4) {

		info.deny_arm = true;

		// We're missing one or more, we need to iterate over the subscription list and mark which motors are connected
		for (auto &sub : _propulsion_system_info_subs) {
			// Grab the motor number from the last report for each subscription
			propulsion_system_info_s data = {};
			sub.copy(&data);
			uint8_t index = data.motor_number - 1; // Motor Numbers are 1 indexed.
			info.connected[index] = true;
		}

		return info;
	}

	// Grab the time only once for efficiency
	uint64_t time_now = hrt_absolute_time();

	// All EEPROMS are connected. Now iterate and check other things.
	for (auto &sub : _propulsion_system_info_subs) {

		propulsion_system_info_s data = {};
		if (sub.update(&data)) {

		} else {
			// Check if we need to mark it as timed out
			sub.copy(&data); // Grab the last report
			uint8_t index = data.motor_number - 1; // Motor Numbers are 1 indexed.

			if ((time_now - data.timestamp) > TIMEOUT_US) {
				PX4_WARN("Motor %d timed out!", data.motor_number);
				info.connected[index] = false;
				info.deny_arm = true;
				continue;
			}
		}

		// Check if Propulsion IDs match

		// Check if Locations are mutually exclusive

		// Check if Locations match the Motor Number
		if (data.location != data.motor_number) {
			info.deny_arm = true;
			info.mismatch = true;
		}
	} // for

	return info;
}

int PropulsionID::start()
{
	ScheduleOnInterval(1000000 / SAMPLE_RATE);
	return PX4_OK;
}

int PropulsionID::task_spawn(int argc, char *argv[])
{
	PropulsionID *instance = new PropulsionID();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int PropulsionID::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the LP work queue to monitor Propulsion System Info and perform the Propulsion ID procedure.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("propulsion_id", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int PropulsionID::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int propulsion_id_main(int argc, char *argv[])
{
	return PropulsionID::main(argc, argv);
}
