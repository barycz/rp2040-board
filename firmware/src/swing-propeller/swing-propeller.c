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
#include "hardware/irq.h"
#include "hardware/dma.h"

#include "bsp/board.h"
#include "tusb.h"

const uint LedYellow = 26;
const uint LedRed = 27;
const uint EscPwmPin = 8;
const uint DataDebugUart = 1; // 0 is reserved for the pico stdio
const uint8_t mpu6050_addr = 0x68;

const uint GyroCalibrationTargetSamples = 200;

const uint EscPwmPeriodUs = 20000;
const uint EscPwmDutyOffUs = 1100;
const uint EscPwmDutyMinUs = 1200;
const uint EscPwmDutySlowUs = 1280;
const uint EscPwmDutyMaxUs = 1360;
const uint EscPwmClkDivider = SYS_CLK_KHZ / 1000;

const uint8_t SwingAxis = 0;
const int32_t SwingPhaseHysteresis = 200;
const int32_t SwingPhaseMinAmplitude = 500;
const uint SwingPropellerImpulseLengthUs = 1000000;

const int32_t SwingFilterKf = 20;
const int32_t SwingFilterKr = 1;
const int32_t SwingFilterKn = SwingFilterKf + SwingFilterKr;

enum SwingPhase {
	PHASE_IDLE,
	PHASE_RISING,
	PHASE_FALLING,
};

uint EscPwmSlice;
uint EscPwmChannel;
int EscPwmDmaChannel;

enum PwmDataConstants {
	PwmWaveformSampleCount = 50,
	PwmWaveformBufferCount = 2,
};
uint16_t PwmWaveformBuffers[PwmWaveformBufferCount][PwmWaveformSampleCount];
uint currentPwmBuffer = 0;
uint nextPwmBuffer = 1;

uint64_t lastEcho;
uint64_t lastData;

int16_t accelDataRaw[3];
int16_t gyroDataRaw[3];
uint gyroNumTakenCalibSamples;
int32_t gyroCalibration[3];

int32_t accelDataFiltered[3];
int32_t gyroDataFiltered[3];

enum SwingPhase swingPhase = PHASE_IDLE;
int32_t swingAxisValue;
uint64_t swingLastPeakUs;
uint64_t swingPeriodUs;

void pwm_waveforms_generate();

void spinUpPropeller() {
	// the previous dma transfer should be already finished
	if (!dma_channel_is_busy(EscPwmDmaChannel)) {
		// Switch to the next buffer
		uint temp = currentPwmBuffer;
		currentPwmBuffer = nextPwmBuffer;
		nextPwmBuffer = temp;

		// Update DMA with the new buffer
		dma_channel_set_read_addr(EscPwmDmaChannel, PwmWaveformBuffers[currentPwmBuffer], false);
		// Start the DMA transfer
		dma_channel_start(EscPwmDmaChannel);

		// Generate new waveform for the next buffer
		pwm_waveforms_generate();
	}
}

bool swingPeriodIsAcceptable() {
	static const uint64_t SwingPeriodAcceptableMinUs = 2000000; // roughly 1m pendulum
	static const uint64_t SwingPeriodAcceptableMaxUs = 4500000; // roughly 5m pendulum
	assert(SwingPeriodAcceptableMinUs >= 2 * PwmWaveformSampleCount * EscPwmPeriodUs);
	return swingPeriodUs >= SwingPeriodAcceptableMinUs
		&& swingPeriodUs <= SwingPeriodAcceptableMaxUs;
}

bool isGyroCalibrated() {
	return gyroNumTakenCalibSamples >= GyroCalibrationTargetSamples;
}

void filterData(const int16_t rawData[3], int32_t filteredData[3]) {
	const int32_t kf = SwingFilterKf;
	const int32_t kr = SwingFilterKr;
	const int32_t kn = SwingFilterKn;
	for (int i = 0; i < 3; i++) {
		filteredData[i] = (kf * filteredData[i] + kr * rawData[i]) / kn;
	}
}

void promoteSwingPhase(const enum SwingPhase newPhase) {
	if (newPhase == swingPhase) {
		return;
	}

	swingPhase = newPhase;
	if (newPhase == PHASE_FALLING) {
		const uint64_t now = time_us_64();
		swingPeriodUs = now - swingLastPeakUs;
		swingLastPeakUs = now;
		printf("falling now %s acceptable period\n",
			swingPeriodIsAcceptable() ? "with" : "without");
		if (swingPeriodIsAcceptable()) {
			spinUpPropeller();
		}
	}
}

void swing_calibration_task() {
	assert(isGyroCalibrated() == false);
	for (int i = 0; i < 3; i++) {
		gyroCalibration[i] += gyroDataRaw[i];
	}
	
	gyroNumTakenCalibSamples += 1;
	if (isGyroCalibrated()) {
		for (int i = 0; i < 3; i++) {
			gyroCalibration[i] /= gyroNumTakenCalibSamples;
		}
		srand(gyroCalibration[0] + gyroCalibration[1] + gyroCalibration[2]);
	}
}

void swing_task() {
	const int32_t newAxisValue = gyroDataFiltered[SwingAxis] - gyroCalibration[SwingAxis];
	const int32_t oldAxisValue = swingAxisValue;
	if (newAxisValue > oldAxisValue + SwingPhaseHysteresis) {
		if (newAxisValue < -SwingPhaseMinAmplitude) {
			promoteSwingPhase(PHASE_RISING);
		}
		swingAxisValue = newAxisValue;
	} else if (newAxisValue < oldAxisValue - SwingPhaseHysteresis) {
		if (newAxisValue > SwingPhaseMinAmplitude) {
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

#if 0
	char buffer[255];
	sprintf(buffer, "%d,%d,%d,%d,%d,%d\r",
		accelDataFiltered[0], accelDataFiltered[1], accelDataFiltered[2],
		gyroDataFiltered[0] - gyroCalibration[0],
		gyroDataFiltered[1] - gyroCalibration[1],
		gyroDataFiltered[2] - gyroCalibration[2]
	);
	tud_cdc_n_write_str(DataDebugUart, buffer);
	tud_cdc_n_write_flush(DataDebugUart);
#endif
}

void placeholder_task() {
	const uint64_t now = time_us_64();

	if (lastEcho + 2000000 < now) {
		printf("Swing period = %d ms\r\n", (int32_t)(swingPeriodUs / 1000));
		lastEcho = now;

#if 0
		for (size_t wf = 0; wf < PwmWaveformCount; ++wf) {
			for (size_t s = 0; s < PwmWaveformSampleCount; ++s) {
				char buffer[64];
				sprintf(buffer, "%u,%u\n", (uint)PwmWaveformBuffers[wf][s], (uint)PwmWaveformBuffers[wf][s]);
				tud_cdc_n_write_str(DataDebugUart, buffer);
				tud_cdc_n_write_flush(DataDebugUart);
			}
		}
#endif
	}

	if (lastData + 10000 < now) {
		mpu6050_task();
		if (isGyroCalibrated()) {
			swing_task();
		} else {
			swing_calibration_task();
		}
		lastData = now;
	}
}

void pwm_waveform_lineseg(size_t waveform, size_t beginIdx, size_t endIdx, int beginValue, int endValue) {
	assert(waveform < PwmWaveformBufferCount);
	assert(endIdx <= PwmWaveformSampleCount);
	assert(beginIdx + 1 < endIdx);
	const int diff = endValue - beginValue;
	const int interval = endIdx - beginIdx;
	for (int i = 0; i < interval; ++i) {
		PwmWaveformBuffers[waveform][beginIdx + i] = beginValue + diff * i / (interval - 1);
	}
}

void pwm_waveforms_generate() {
	// Predefined duty cycle values
	const int predefined_duty_cycles[] = {
		EscPwmDutyOffUs,
		EscPwmDutyMinUs,
		EscPwmDutySlowUs,
		EscPwmDutyMaxUs
	};
	const size_t num_predefined = sizeof(predefined_duty_cycles) / sizeof(predefined_duty_cycles[0]);
	const size_t num_segments = 4; // Number of random segments
	const uint total_duration = PwmWaveformSampleCount;
	const uint off_duration = 5;
	const uint segment_span = (total_duration - 2 * off_duration) / num_segments; // Calculate span from total duration and segments

	// Transition probability matrix
	// Format: [from_duty][to_duty] = probability (0-100)
	// Rows: Off, Min, Slow, Max
	// Columns: Off, Min, Slow, Max
	const uint8_t transition_probabilities[4][4] = {
		// Off -> Off, Min, Slow, Max
		{ 0, 40, 40, 20 },
		// Min -> Off, Min, Slow, Max
		{ 0, 10, 50, 40 },
		// Slow -> Off, Min, Slow, Max
		{ 0, 45, 10, 45 },
		// Max -> Off, Min, Slow, Max
		{ 0, 50, 40, 10 }
	};

	// Generate random waveform for the next buffer
	size_t current_idx = 0;

	// Generate segments (including the first one after off state)
	for (uint seg = 0; seg <= num_segments + 1; seg++) {
		// Calculate start and end times
		uint start_time = seg == 0 ? 0 : off_duration + (seg - 1) * segment_span;
		uint end_time = seg == 0 ? off_duration : start_time + segment_span;

		// Select next duty based on probabilities
		uint8_t rand_val = rand() % 100; // Get random value 0-99
		uint8_t cumulative_prob = 0;
		size_t next_idx = 0;

		// Find which probability range the random value falls into
		for (size_t i = 0; i < num_predefined; i++) {
			cumulative_prob += transition_probabilities[current_idx][i];
			if (rand_val < cumulative_prob) {
				next_idx = i;
				break;
			}
		}

		// For the last segment, go back to off state
		if (seg == num_segments + 1) {
			next_idx = 0; // Force off state
			end_time = total_duration;
		}

		pwm_waveform_lineseg(nextPwmBuffer, start_time, end_time, predefined_duty_cycles[current_idx], predefined_duty_cycles[next_idx]);
		current_idx = next_idx; // Update current index for next segment
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

	gpio_set_function(EscPwmPin, GPIO_FUNC_PWM);
	EscPwmSlice = pwm_gpio_to_slice_num(EscPwmPin);
	EscPwmChannel = pwm_gpio_to_channel(EscPwmPin);

	pwm_set_clkdiv_int_frac(EscPwmSlice, EscPwmClkDivider, 0);
	pwm_set_wrap(EscPwmSlice, EscPwmPeriodUs);
	pwm_set_chan_level(EscPwmSlice, EscPwmChannel, EscPwmDutyOffUs);
	pwm_set_enabled(EscPwmSlice, true);

	pwm_waveforms_generate();

	EscPwmDmaChannel = dma_claim_unused_channel(true);
	dma_channel_config dma_config = dma_channel_get_default_config(EscPwmDmaChannel);

	channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
	channel_config_set_read_increment(&dma_config, true);
	channel_config_set_write_increment(&dma_config, false);
	channel_config_set_dreq(&dma_config, pwm_get_dreq(EscPwmSlice));

	assert(EscPwmChannel == PWM_CHAN_A);
	dma_channel_configure(
		EscPwmDmaChannel,
		&dma_config,
		&pwm_hw->slice[EscPwmSlice].cc, // Write address (PWM counter compare)
		PwmWaveformBuffers[nextPwmBuffer], // Read address (data array)
		PwmWaveformSampleCount, // Transfer count
		false // do not start yet
	);

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
