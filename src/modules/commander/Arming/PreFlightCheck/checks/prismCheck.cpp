/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "../PreFlightCheck.hpp"

#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/propulsion_id_info.h>

using namespace time_literals;

bool PreFlightCheck::prismCheck(orb_advert_t *mavlink_log_pub, const bool report_fail)
{
	PX4_INFO("Performing PRISM preflight check");

	uORB::SubscriptionData<propulsion_id_info_s> sub{ORB_ID(propulsion_id_info)};
	sub.update();

	auto info = sub.get();

	char error_message[128] = {};

	// Check if they're all connected
	bool all_connected = true && (info.connected[0] == info.connected[1] &&
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

	if (!all_connected) {
		sprintf(error_message, "Missing Propulsion system(s)\n- Connected: [%d, %d, %d, %d]",
				info.connected[0], info.connected[1], info.connected[2], info.connected[3]);

	} else if (!propulsion_ids_match) {
		sprintf(error_message, "Propulsion IDs do not match!\n- IDs: [%d, %d, %d, %d",
			info.propulsion_ids[0], info.propulsion_ids[1], info.propulsion_ids[2], info.propulsion_ids[3]);

	} else if (!mutually_exclusive) {
		sprintf(error_message, "Propulsion locations are not mutually exclusive!\n- Locations: [%d, %d, %d, %d",
			info.locations[0], info.locations[1], info.locations[2], info.locations[3]);
	}

	if (info.deny_arm) {
		mavlink_log_critical(mavlink_log_pub, "Fail: %s", error_message);
	}

	return info.deny_arm;
}
