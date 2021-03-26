/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "M9504DF.hpp"

using namespace M9504DF;

namespace m9504df
{


M9504DF::M9504DF(I2CSPIBusOption bus_option, int bus, device::Device *interface) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface->get_device_id()), bus_option, bus,
		     interface->get_device_address()),
	_interface(interface),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{
}

M9504DF::~M9504DF()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);

	delete _interface;
}

int
M9504DF::init()
{
	// if (RegisterRead(Register::ID) != Infineon_DPS310::REV_AND_PROD_ID) {
	// 	PX4_ERR("Product_ID mismatch");
	// 	return PX4_ERROR;
	// }

	if (reset() != PX4_OK) {
		PX4_DEBUG("reset failed");
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

int
M9504DF::reset()
{
	// Soft Reset
	// RegisterSetBits(Register::RESET, RESET_BIT::SOFT_RST);
	// usleep(40000);	// 40 milliseconds

	return PX4_OK;
}

void
M9504DF::start()
{
	ScheduleOnInterval(1000000 / SAMPLE_RATE);
}

void
M9504DF::RunImpl()
{
	perf_begin(_sample_perf);

	PX4_INFO("Driver %d is running", interface);

	// Publish to uORB
	// _px4_barometer.update(timestamp_sample, Pcomp / 100.0f); // Pascals -> Millibar

	perf_end(_sample_perf);
}

uint8_t
M9504DF::RegisterRead(Register reg)
{
	uint8_t buf{};
	_interface->read((uint8_t)reg, &buf, 1);
	return buf;
}

void
M9504DF::RegisterWrite(Register reg, uint8_t value)
{
	_interface->write((uint8_t)reg, &value, 1);
}

void
M9504DF::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void
M9504DF::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

void
M9504DF::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

} // namespace m9504df
