#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "ssd1306.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "quadrature_encoder.pio.h"

const uint LedYellow = 26;
const uint LedRed = 27;

const uint AudioPwm = 22;

const uint RotaryA = 17;
const uint RotaryB = 18;
const uint RotarySW = 19;

const uint DmxTx = 24;
const uint DmxRx = 25;
const uint DmxRts = 23;
uart_inst_t* const DmxUart = uart1;
const uint DmxBaudRate = 250000;
const uint DmxDataBits = 8;
const uint DmxStopBits = 2;
const uart_parity_t DmxParity = UART_PARITY_NONE;
#define DMX_NUM_CHANNELS 512
const uint DmxMinBreakUs = 92;
const uint DmxMinMarkAfterBreak = 12;

const uint DispSda = 28;
const uint DispSck = 29;
#define DISP_I2C (&i2c0_inst)

// start code + channels
uint8_t DmxBuffer[DMX_NUM_CHANNELS + 1];

ssd1306_t disp;
PIO encoderPio = pio0;
const uint encoderSm = 0;

struct {
	bool channelSelect;
	int32_t currentChannel;
	int32_t lastEncoderValue;
	bool lastSwitchState;
} DmxUi;

uint8_t getDmxChannelData(uint channel) {
	const uint offset = channel + 1; // buffer contains 1 byte start code
	if (offset >= sizeof(DmxBuffer)) {
		return 0;
	}

	return DmxBuffer[offset];
}

void updateDmxChannelData(uint channel, int32_t diff) {
	const uint offset = channel + 1; // buffer contains 1 byte start code
	if (offset >= sizeof(DmxBuffer)) {
		return;
	}

	DmxBuffer[offset] += diff;
}

// https://en.wikipedia.org/wiki/DMX512
// takes 21.2ms for 512 slots/channels
// do not send less that 24 slots as there is a minimum break to break delay of 1204us
void sendDmxFrameBlocking() {
	// switch to GPIO to transmit the break condition
	gpio_init(DmxTx);
	gpio_set_dir(DmxTx, GPIO_OUT);

	// break
	gpio_put(DmxTx, false);
	sleep_us(DmxMinBreakUs);

	// mark after break
	gpio_put(DmxTx, true);
	sleep_us(DmxMinMarkAfterBreak);

	// switch to uart
	gpio_set_function(DmxTx, GPIO_FUNC_UART);

	// start code + channels
	uart_write_blocking(DmxUart, DmxBuffer, sizeof(DmxBuffer));
}

// takes 10-20us
void processUserInput() {
	// there are 4 steps between each encoder notch
	const int32_t encoderValue = quadrature_encoder_get_count(encoderPio, encoderSm) / 4;
	const bool switchState = gpio_get(RotarySW);
	if (switchState != DmxUi.lastSwitchState) {
		// only switch modes when pressed, do nothing when released
		if (switchState) {
			DmxUi.channelSelect = !DmxUi.channelSelect;
		}

		DmxUi.lastSwitchState = switchState;
	}

	// we want CW rotation to be the positive direction, but the encoder value increases CCW
	const int32_t encoderDiff = DmxUi.lastEncoderValue - encoderValue;
	if (encoderDiff) {
		if (DmxUi.channelSelect) {
			DmxUi.currentChannel = MAX(MIN(DmxUi.currentChannel + encoderDiff, DMX_NUM_CHANNELS - 1), 0);
		} else { // value select
			updateDmxChannelData(DmxUi.currentChannel, encoderDiff * 256 / 32); // speedup
		}

		DmxUi.lastEncoderValue = encoderValue;
	}
}

// takes 26.6ms
void displayUserInterface() {
	ssd1306_clear(&disp);

	char buffer[64];
	snprintf(buffer, sizeof(buffer), "%c Channel %d", DmxUi.channelSelect ? '>' : ' ', DmxUi.currentChannel);
	ssd1306_draw_string(&disp, 0, 0, 1, buffer);

	snprintf(buffer, sizeof(buffer), "%c Value %d", DmxUi.channelSelect ? ' ' : '>', getDmxChannelData(DmxUi.currentChannel));
	ssd1306_draw_string(&disp, 0, 16, 1, buffer);
	ssd1306_show(&disp);
}

int main() {
	stdio_init_all();

	gpio_init(LedYellow);
	gpio_set_dir(LedYellow, GPIO_OUT);

	gpio_init(LedRed);
	gpio_set_dir(LedRed, GPIO_OUT);

	uart_init(DmxUart, DmxBaudRate);
	uart_set_format(DmxUart, DmxDataBits, DmxStopBits, DmxParity);
	gpio_set_function(DmxTx, GPIO_FUNC_UART);
	gpio_set_function(DmxRx, GPIO_FUNC_UART);
	gpio_init(DmxRts);
	gpio_set_dir(DmxRts, GPIO_OUT);
	gpio_put(DmxRts, true);

	gpio_init(RotaryA);
	gpio_pull_up(RotaryA);

	gpio_init(RotaryB);
	gpio_pull_up(RotaryB);

	gpio_init(RotarySW);
	gpio_pull_up(RotarySW);

	pio_add_program(encoderPio, &quadrature_encoder_program);
	quadrature_encoder_program_init(encoderPio, encoderSm, RotaryA, 10000);

	gpio_set_function(AudioPwm, GPIO_FUNC_PWM);
	uint pwm_slice = pwm_gpio_to_slice_num(AudioPwm);
	pwm_set_gpio_level(AudioPwm, 100);
	pwm_set_wrap(pwm_slice, 200);
	pwm_set_clkdiv_int_frac(pwm_slice, 100, 0);
	pwm_set_enabled(pwm_slice, true);

	i2c_init(DISP_I2C, 400000);
	gpio_set_function(DispSda, GPIO_FUNC_I2C);
	gpio_set_function(DispSck, GPIO_FUNC_I2C);
	gpio_pull_up(DispSda);
	gpio_pull_up(DispSck);

	disp.external_vcc=false;
	ssd1306_init(&disp, 128, 64, 0x3C, DISP_I2C);

	while (true) {
		gpio_put(LedYellow, 1);

		const uint64_t timeBefore = time_us_64();

		processUserInput();

		displayUserInterface();

		sendDmxFrameBlocking();

		const uint64_t timeNow = time_us_64();
		printf("main loop took %uus\n", (uint32_t)(timeNow - timeBefore));

		gpio_put(LedYellow, 0);
		sleep_ms(20);
	}
}