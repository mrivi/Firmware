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
 * @file s50mv85G.cpp
 * @author
 *
 * Driver for the Broadcom S50MV85G ToF sensor
 */

#include <poll.h>
#include <px4_cli.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>


/* Device limits */
#define S50MV85G_MIN_DISTANCE                               (0.01f) // meters
#define S50MV85G_MAX_DISTANCE                               (10.f) // meters

#define S50MV85G_MEASURE_INTERVAL                            0.003 //seconds
#define S50MV85G_DEFAULT_PORT       "/dev/ttyS2" // Default serial port on Pixhawk (TELEM2), baudrate 115200


class S50MV85G : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	S50MV85G(const char *port = S50MV85G_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~S50MV85G() override;
	virtual int  init() override;
	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

private:

	void				start();
	void				stop();
	int         open_serial_port(const speed_t speed = B115200);
	int         collect();


	int _file_descriptor{-1};

	char _port[20] {};
	uint8_t _cycle_counter{0};
	uint8_t _rotation{0};
	// orb_advert_t _distance_sensor_topic{nullptr};


};

S50MV85G::S50MV85G(const char *port, uint8_t rotation) :
	CDev(RANGE_FINDER_BASE_DEVICE_PATH),
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_rotation(rotation)
{
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';
}

S50MV85G::~S50MV85G()
{
	// Ensure we are truly inactive.
	stop();
	//
	// perf_free(_sample_perf);
	// perf_free(_comms_errors);
}

int
S50MV85G::init()
{
	// Intitialize the character device.
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	// start();
	return PX4_OK;
}

void S50MV85G::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	// Perform collection.
	if (collect() == -EAGAIN) {
		_cycle_counter++;
	}

	_cycle_counter = 0;
}

void
S50MV85G::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(S50MV85G_MEASURE_INTERVAL, 0);
	PX4_INFO("driver started");
}

void
S50MV85G::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);

	// Clear the work queue schedule.
	ScheduleClear();
}

int
S50MV85G::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

int
S50MV85G::collect()
{
	return PX4_OK;
}

/**
 * Local functions in support of the shell command.
 */
namespace s50mv85g
{
S50MV85G *g_dev;
int start(const char *port = S50MV85G_DEFAULT_PORT,
	  const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int usage();

/**
 * Start the driver.
 */
int
start(const char *port, const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new S50MV85G(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	// g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

int
usage()
{
	PX4_INFO("usage: s50mv85g command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace s50mv85g
/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int s50mv85g_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = S50MV85G_DEFAULT_PORT;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R': {
				int rot = -1;

				if (px4_get_parameter_value(myoptarg, rot) != 0) {
					PX4_ERR("rotation parsing failed");
					return -1;
				}

				rotation = (uint8_t)rot;
				break;
			}

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return s50mv85g::usage();
		}
	}

	if (myoptind >= argc) {
		return s50mv85g::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		return s50mv85g::start(device_path, rotation);
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return s50mv85g::stop();
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return s50mv85g::status();
	}

	return s50mv85g::usage();
}
