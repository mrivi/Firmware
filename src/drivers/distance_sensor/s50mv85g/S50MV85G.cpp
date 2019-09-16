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

#include "S50MV85G.hpp"

static constexpr uint32_t TIME_us_TSWW = 11; //  - actually 10.5us


S50MV85G::S50MV85G(int bus, enum Rotation rotation) :
	SPI("S50MV85G", S50MV85G_DEVICE_PATH, bus, PMW3901_SPIDEV, SPIDEV_MODE3, S50MV85G_SPI_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_sample_perf(perf_alloc(PC_ELAPSED, "pmw3901: read")),
	_comms_errors(perf_alloc(PC_COUNT, "pmw3901: com err")),
	_rotation(rotation)
{
}

S50MV85G::~S50MV85G()
{
	// make sure we are truly inactive
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
S50MV85G::sensorInit()
{
	sendCommand(0x11); // start auto
	return PX4_OK;
}

int
S50MV85G::init()
{

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		return PX4_ERROR;
	}

	sensorInit();

	_previous_collect_timestamp = hrt_absolute_time();

	start();

	return PX4_OK;
}

int
S50MV85G::probe()
{
	// uint8_t data[2] {};
	//
	// readRegister(0x00, &data[0], 1); // chip id
	//
	// // Test the SPI communication, checking chipId and inverse chipId
	// if (data[0] == 0x49) {
	// 	return OK;
	// }
	uint8_t response[4];
	int ret = readMeas(0x0C, response, sizeof(response));
	printf("response %x %x %x %x\n", response[0], response[1], response[2], response[3]);
	printf("%d \n", ret);
	readMeas(0x0D, response, sizeof(response));
	printf("response %x %x %x %x\n", response[0], response[1], response[2], response[3]);
	return OK;

	// not found on any address
	// return -EIO;
}

void
S50MV85G::Run()
{

}

void
S50MV85G::start()
{
	// schedule a cycle to start things
	ScheduleOnInterval(PMW3901_SAMPLE_INTERVAL, PMW3901_US);
}

void
S50MV85G::stop()
{
	ScheduleClear();
}

void
S50MV85G::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

int
S50MV85G::sendCommand(unsigned reg)
{
	int ret;
	uint8_t cmd[1]; 						// write 1 byte
	cmd[0] = DIR_WRITE(reg);

	ret = transfer(&cmd[0], nullptr, 1);
	printf("spi::transfer returned %d \n", ret);


	if (OK != ret) {
		perf_count(_comms_errors);
		printf("spi::transfer returned %d \n", ret);
		return ret;
	}

	px4_usleep(TIME_us_TSWW);

	return ret;
}

int
S50MV85G::sendCommandMulti(unsigned cmd, uint8_t *data, uint8_t size)
{
	int ret;

	ret = sendCommand(cmd);

	ret |= transfer(&data[0], nullptr, size);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	px4_usleep(TIME_us_TSWW);

	return ret;
}

int S50MV85G::readMeas(unsigned cmd, uint8_t *read, uint8_t size)
{
	int ret = sendCommand(cmd);
	ret |= transfer(nullptr, &read[0], size);


	if (OK != ret) {
		perf_count(_comms_errors);
		printf("spi::transfer returned %d", ret);
		return ret;
	}

	px4_usleep(TIME_us_TSWW);

	return ret;
}
