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

#include "M95040DF.hpp"

// using namespace M95040DF;

namespace m95040df
{


M95040DF::M95040DF(I2CSPIBusOption bus_option, int bus, int devid, int bus_frequency,
		 spi_mode_e spi_mode) :
	SPI(DRV_DEVTYPE_M95040DF, MODULE_NAME, bus, devid, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors"))
{
}

M95040DF::~M95040DF()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
M95040DF::init()
{
	SPI::set_lockmode(LOCK_THREADS);

	// Do SPI init (and probe) first
	if (SPI::init() != PX4_OK) {
		return PX4_ERROR;
	}

	start();

	return PX4_OK;
}

void
M95040DF::start()
{
	ScheduleOnInterval(1000000 / SAMPLE_RATE);
}

void
M95040DF::stop()
{
	ScheduleClear();
}

void
M95040DF::RunImpl()
{
	perf_begin(_sample_perf);

	uint8_t val = RegisterRead(CMD_READ_SR);

	PX4_INFO("Status reg: %d", val);


	// Read location page
	// uint8_t buf[PAGE_SIZE_BYTES] = {};
	// int ret = ReadPage(LOCATION_PAGE_NUM, buf);

	// if (ret != PX4_OK) {
	// 	PX4_INFO("ReadPage failed --> %d", ret);

	// } else {
	// 	// Publish to uORB
	// 	// _px4_baromster.update(timestamp_sample, Pcomp / 100.0f); // Pascals -> Millibars
	// }

	// PX4_INFO("ReadPage data 1: %d", buf[0]);
	// PX4_INFO("ReadPage data 2: %d", buf[1]);
	// PX4_INFO("ReadPage data 3: %d", buf[2]);
	// PX4_INFO("ReadPage data 4: %d\n", buf[3]);

	// // Make it a string
	// char str_buf[PAGE_SIZE_BYTES + 1] = {};
	// memcpy(str_buf, buf, PAGE_SIZE_BYTES);
	// str_buf[PAGE_SIZE_BYTES] = '\0';
	// PX4_INFO("ReadPage data: %s", str_buf);

	perf_end(_sample_perf);
}

// virtual int	read(unsigned address, void *data, unsigned count)

// Page numbers are zero indexed
int
M95040DF::ReadPage(unsigned page_number, uint8_t* data)
{
	if (page_number > MEM_SIZE_PAGES - 1) {
		return -1;
	}

	unsigned address = page_number * PAGE_SIZE_BYTES;
	uint8_t buffer[PAGE_SIZE_BYTES + 2] = {};
	uint8_t command = CMD_READ;
	uint8_t byteAddress = 0xFF & address;

	// Bit 3 of the CMD byte is used to address memory block 1 or 2
	if (address > 0xFF) {
		// bit 3 of COMMAND should be set to 1
		command |= (1 << 3);
	} else {
		// bit 3 of COMMAND should be set to 0
		command &= ~(1 << 3);
	}

	buffer[0] = command;
	buffer[1] = byteAddress;

	uint8_t recv_buf[PAGE_SIZE_BYTES] = {};

	int ret = transfer(buffer, recv_buf, PAGE_SIZE_BYTES + 2);

	memcpy(data, recv_buf, PAGE_SIZE_BYTES);

	return ret;
}

uint8_t
M95040DF::RegisterRead(uint8_t reg)
{
	uint8_t buf[32] = {};

	buf[0] = reg;

	int ret = transfer(&buf[0], &buf[0], 2);

	if (ret != PX4_OK) {
		PX4_INFO("RegisterRead transfer() failed");
	}

	uint8_t wtf = buf[1];

	return wtf;
}

// void
// M95040DF::RegisterWrite(uint8_t reg, uint8_t value)
// {
// 	_interface->write((uint8_t)reg, &value, 1);
// }

void
M95040DF::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

} // namespace m95040df
