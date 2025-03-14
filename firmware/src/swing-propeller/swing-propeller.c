#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "bsp/board.h"
#include "tusb.h"

const uint LedYellow = 26;
const uint LedRed = 27;
const uint DataDebugUart = 1; // 0 is reserved for the pico stdio
const uint8_t mpu6050_addr = 0x68;

const uint8_t SwingAxis = 1;
const int32_t SwingPhaseHysteresis = 200;

const int32_t SwingFilterKf = 20;
const int32_t SwingFilterKr = 1;
const int32_t SwingFilterKn = SwingFilterKf + SwingFilterKr;

enum Phase {
	PHASE_UNKNOWN,
	PHASE_RISING,
	PHASE_FALLING,
};

uint64_t lastEcho;
uint64_t lastData;

int16_t accelDataRaw[3];
int16_t gyroDataRaw[3];

int32_t accelDataFiltered[3];
int32_t gyroDataFiltered[3];

enum Phase swingPhase = PHASE_UNKNOWN;
int32_t swingAxisValue;
uint64_t swingLastPeakUs;
uint64_t swingPeriodUs;

void filterData(const int16_t rawData[3], int32_t filteredData[3]) {
	const int32_t kf = SwingFilterKf;
	const int32_t kr = SwingFilterKr;
	const int32_t kn = SwingFilterKn;
	for (int i = 0; i < 3; i++) {
		filteredData[i] = (kf * filteredData[i] + kr * rawData[i]) / kn;
	}
}

void promoteSwingPhase(const enum Phase newPhase) {
	if (newPhase == swingPhase) {
		return;
	}

	swingPhase = newPhase;
	if (newPhase == PHASE_FALLING) {
		const uint64_t now = time_us_64();
		swingPeriodUs = now - swingLastPeakUs;
		swingLastPeakUs = now;
	}
}

void swing_task() {
	const int32_t newAxisValue = accelDataFiltered[SwingAxis];
	const int32_t oldAxisValue = swingAxisValue;
	if (newAxisValue > oldAxisValue + SwingPhaseHysteresis) {
		if (newAxisValue < SwingPhaseHysteresis) {
			promoteSwingPhase(PHASE_RISING);
		}
		swingAxisValue = newAxisValue;
	} else if (newAxisValue < oldAxisValue - SwingPhaseHysteresis) {
		if (newAxisValue > SwingPhaseHysteresis) {
			promoteSwingPhase(PHASE_FALLING);
		}
		swingAxisValue = newAxisValue;
	}
}

void mpu6050_reset() {
	// the simplest reset sequence
	uint8_t buf[] = {0x6B, 0x00};
	i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
	// For this particular device, we send the device the register we want to read
	// first, then subsequently read from the device. The register is auto incrementing
	// so we don't need to keep sending the register we want, just the first.

	uint8_t buffer[6];

	// Start reading acceleration registers from register 0x3B for 6 bytes
	uint8_t val = 0x3B;
	i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true); // true to keep master control of bus
	i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 6, false);

	for (int i = 0; i < 3; i++) {
		accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
	}

	// Now gyro data from reg 0x43 for 6 bytes
	// The register is auto incrementing on each read
	val = 0x43;
	i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true);
	i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 6, false);  // False - finished with bus

	for (int i = 0; i < 3; i++) {
		gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
	}
}

void mpu6050_task() {
	mpu6050_read_raw(accelDataRaw, gyroDataRaw);
	filterData(accelDataRaw, accelDataFiltered);
	filterData(gyroDataRaw, gyroDataFiltered);

	char buffer[255];
	sprintf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
		accelDataFiltered[0], accelDataFiltered[1], accelDataFiltered[2],
		gyroDataFiltered[0], gyroDataFiltered[1], gyroDataFiltered[2],
		swingAxisValue, swingPhase * 10000 - 15000, 0
	);
	tud_cdc_n_write_str(DataDebugUart, buffer);
}

void placeholder_task() {
	const uint64_t now = time_us_64();

	if (lastEcho + 1000000 < now) {
		printf("Swing period = %d ms\r\n", (int32_t)(swingPeriodUs / 1000));
		lastEcho = now;
	}

	if (lastData + 10000 < now) {
		mpu6050_task();
		swing_task();
		lastData = now;
	}
}

int main(void) {
	board_init();
	// initialize tiny usb before stdio as we use the usb backend
	tud_init(BOARD_TUD_RHPORT);
	stdio_init_all();

	gpio_init(LedYellow);
	gpio_set_dir(LedYellow, GPIO_OUT);

	gpio_init(LedRed);
	gpio_set_dir(LedRed, GPIO_OUT);

	// This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	mpu6050_reset();

	lastData = lastEcho = time_us_64();

	while (true) {
		gpio_put(LedYellow, 1);

		tud_task();
		placeholder_task();

		gpio_put(LedYellow, 0);
		sleep_ms(10); // __wfi();
	}
}
