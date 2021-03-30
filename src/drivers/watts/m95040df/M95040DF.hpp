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

/**
 * @file M95040DF.cpp
 *
 * Driver for the M95040DF EEPROM connected vi SPI.
 *
 * 4kB
 *
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <lib/drivers/device/spi.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/i2c_spi_buses.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/propulsion_system_info.h>

namespace m95040df
{

// Definitions
static constexpr int PAGE_SIZE_BYTES  = 16;
static constexpr int MEM_SIZE_PAGES   = 32;
// Commands
static constexpr int CMD_WRITE_SR = 	1;
static constexpr int CMD_WRITE = 		2;
static constexpr int CMD_READ = 		3;
static constexpr int CMD_WRITE_DIS = 	4;
static constexpr int CMD_READ_SR = 		5;
static constexpr int CMD_WRITE_EN = 	6;
static constexpr int LOCATION_PAGE_NUM = 1;

class M95040DF : public device::SPI, public I2CSPIDriver<M95040DF>
{
public:
	M95040DF(I2CSPIBusOption bus_option, int bus, int devid, int bus_frequency,
		spi_mode_e spi_mode);
	virtual ~M95040DF();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	virtual int		init();

	void			print_status();

	void			RunImpl();

private:

	void			start();
	void 			stop();


	int ReadPage(unsigned page_number, uint8_t* data);

	uint8_t			RegisterRead(uint8_t reg);
	void			RegisterWrite(uint8_t reg, uint8_t val);

	static constexpr uint32_t SAMPLE_RATE{1}; // samples per second

	uORB::Publication<propulsion_system_info_s> _propulsion_system_info_pub{ORB_ID(propulsion_system_info)};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
};

} // namespace m95040df
