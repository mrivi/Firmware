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
 * @file S50MV85G.hpp
 * @author
 *
 * Driver for the Broadcom S50MV85G ToF sensor
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/device/spi.h>
#include <conversion/rotation.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/distance_sensor.h>

/* Configuration Constants */

#if defined PX4_SPI_BUS_EXPANSION   // crazyflie
# define PMW3901_BUS PX4_SPI_BUS_EXPANSION
#elif defined PX4_SPI_BUS_EXTERNAL1   // fmu-v5
# define PMW3901_BUS PX4_SPI_BUS_EXTERNAL1
#elif defined PX4_SPI_BUS_EXTERNAL    // fmu-v4 extspi
# define PMW3901_BUS PX4_SPI_BUS_EXTERNAL
#else
# error "add the required spi bus from board_config.h here"
#endif

#if defined PX4_SPIDEV_EXPANSION_2    // crazyflie flow deck
# define PMW3901_SPIDEV PX4_SPIDEV_EXPANSION_2
#elif defined PX4_SPIDEV_EXTERNAL1_1    // fmu-v5 ext CS1
# define PMW3901_SPIDEV PX4_SPIDEV_EXTERNAL1_1
#elif defined PX4_SPIDEV_EXTERNAL   // fmu-v4 extspi
# define PMW3901_SPIDEV PX4_SPIDEV_EXTERNAL
#else
# error "add the required spi dev from board_config.h here"
#endif

#define S50MV85G_DEVICE_PATH "/dev/s50mv85g"
#define S50MV85G_SPI_BUS_SPEED (200000L) // 2MHz
#define PMW3901_SAMPLE_INTERVAL 10000 /*  10 ms */
#define PMW3901_US 1000 /*   1 ms */

#define DIR_WRITE(a) ((a) | (1 << 7))
#define DIR_READ(a) ((a) & 0x7f)

class S50MV85G : public device::SPI, public px4::ScheduledWorkItem
{
public:
	S50MV85G(int bus = PMW3901_BUS, enum Rotation yaw_rotation = (enum Rotation)0);

	virtual ~S50MV85G();

	virtual int init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

protected:
	virtual int probe();

private:

	const uint64_t _collect_time{15000}; // usecs, optical flow data publish rate

	uORB::PublicationMulti<distance_sensor_s> _distance_sensor_pub{ORB_ID(distance_sensor)};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	uint64_t _previous_collect_timestamp{0};

	enum Rotation _rotation;

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void Run() override;

	// int readRegister(unsigned reg, uint8_t *data, unsigned count);
	// int writeRegister(unsigned reg, uint8_t data);
	int sendCommand(unsigned reg);
	int sendCommandMulti(unsigned cmd, uint8_t *data, uint8_t size);
	int readMeas(unsigned cmd, uint8_t *read, uint8_t size);

	int sensorInit();
};
