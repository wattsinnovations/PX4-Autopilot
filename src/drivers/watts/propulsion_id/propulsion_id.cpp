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

#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

extern orb_advert_t mavlink_log_pub;

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

	_propulsion_id_info_pub.publish(info);
}

propulsion_id_info_s PropulsionID::get_propulsion_id_info()
{
	propulsion_id_info_s info = {};
	info.deny_arm = true;
	bool correct_motor_order = true;

	// Grab the time only once for efficiency
	uint64_t time_now = hrt_absolute_time();
	info.timestamp = time_now;

	// All EEPROMS are connected. Now iterate and check other things.
	for (auto &sub : _propulsion_system_info_subs) {

		// Ignore instances that have never been advertised
		if (!sub.advertised()) {
			continue;
		}

		// Check if any have timed out
		propulsion_system_info_s data = {};
		if (sub.update(&data)) {

			int index = data.motor_number - 1; // Motor Numbers are 1 indexed.

			// Sanity check
			if (index < 0) {
				PX4_WARN("Invalid index %d", index);
				return info;
			}

			info.connected[index] = true;

		} else {
			// Check if we need to mark it as timed out
			sub.copy(&data); // Grab the last report
			int index = data.motor_number - 1; // Motor Numbers are 1 indexed.

			// Sanity check
			if (index < 0) {
				PX4_WARN("Invalid index %d", index);
				return info;
			}

			bool timed_out = (time_now - data.timestamp) > TIMEOUT_US;

			if (timed_out) {
				PX4_WARN("Motor %d timed out!", data.motor_number);
				info.connected[index] = false;
				continue;
			}
		}

		info.locations[data.motor_number - 1] = data.location;
		info.propulsion_ids[data.motor_number - 1] = data.propulsion_id;
		info.flight_times[data.motor_number - 1] = data.flight_time_ms;

		// Check if Locations match the Motor Number -- this would indicate an issue with firmware chip select
		// ordering and is therefore only a sanity check.
		if (data.location != data.motor_number) {
			correct_motor_order = false;
		}
	}

	// Check if they're all connected
	bool all_connected = (true == info.connected[0] &&
				info.connected[0] == info.connected[1] &&
				info.connected[1] == info.connected[2] &&
				info.connected[2] == info.connected[3]);

	// Check if Propulsion IDs match
	bool propulsion_ids_match = info.propulsion_ids[0] == info.propulsion_ids[1] &&
				info.propulsion_ids[1] == info.propulsion_ids[2] &&
				info.propulsion_ids[2] == info.propulsion_ids[3];

	// Check if Locations are mutually exclusive
	bool mutually_exclusive = info.locations[0] != info.locations[1] &&
				info.locations[1] != info.locations[2] &&
				info.locations[2] != info.locations[3];

	char error_message[128] = "Unknown";

	if (!all_connected) {
		sprintf(error_message, "Missing Propulsion system(s)\n- Connected: [%d, %d, %d, %d]",
				info.connected[0], info.connected[1], info.connected[2], info.connected[3]);

	} else if (!correct_motor_order) {
		sprintf(error_message, "Motor order is incorrect!\n- Locations: [%d, %d, %d, %d]",
			info.locations[0], info.locations[1], info.locations[2], info.locations[3]);

	} else if (!propulsion_ids_match) {
		sprintf(error_message, "Propulsion IDs do not match!\n- IDs: [%d, %d, %d, %d]",
			info.propulsion_ids[0], info.propulsion_ids[1], info.propulsion_ids[2], info.propulsion_ids[3]);

	} else if (!mutually_exclusive) {
		sprintf(error_message, "Propulsion locations are not mutually exclusive!\n- Locations: [%d, %d, %d, %d]",
			info.locations[0], info.locations[1], info.locations[2], info.locations[3]);
	}

	bool all_checks_passed = all_connected && correct_motor_order &&
							propulsion_ids_match && mutually_exclusive;

	// If all the checks have passed, check if the Propulsion Group is already configured
	if (all_checks_passed) {

		int detected_propulsion_group = info.propulsion_ids[0];
		int configured_propulsion_group = _param_prop_group.get();

		bool configured = detected_propulsion_group == configured_propulsion_group;
		if (!configured) {

			bool reboot = true;
			mavlink_log_critical(&mavlink_log_pub, "Propulsion Configuration has changed. Reconfiguring.");

			_param_sys_autoconfig.set(2); // Reload airframe parameters -- PROPULSION_GROUP is
										  // set in the airframe file

			switch (detected_propulsion_group) {
			case 1:
				_param_sys_autostart.set(PRISM_AIRFRAME_ID_QUAD);
				break;

			case 2:
				_param_sys_autostart.set(PRISM_AIRFRAME_ID_X8);
				break;

			default:
				mavlink_log_critical(&mavlink_log_pub, "Unknown Propulsion Group: %d", detected_propulsion_group);
				reboot = false; // Edge case but we don't want the drone to constantly reboot
				break;
			}

			if (reboot) {
				px4_reboot_request(false, 500000); // Give it half a second to ensure parameters are written
			}

		} else {
			// Finally, you may arm :)
			info.deny_arm = false;
		}

	} else {
		mavlink_log_critical(&mavlink_log_pub, "error: %s", error_message);
	}

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
