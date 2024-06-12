#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#include "bsp/board.h"
#include "tusb.h"

const uint LedYellow = 26;
const uint LedRed = 27;

const uint AudioPwm = 22;

const uint RotaryA = 17;
const uint RotaryB = 18;
const uint RotarySW = 19;

const uint JoystickPins[] = {28, 29};
const uint JoystickInputs[] = {2, 3};

const uint MidiCable = 0;
const uint MidiChannel = 0;

typedef struct axis_calibration {
	uint min;
	uint max;
} axis_calibration_t;

// hardcored values for now
axis_calibration_t adc_a_calibration = { 492, 3397 };
axis_calibration_t adc_b_calibration = { 561, 3292 };

uint axis_remap_adc(axis_calibration_t* calibration, uint raw_adc) {
	if (raw_adc < calibration->min) {
		raw_adc = calibration->min;
	} else if (raw_adc > calibration->max) {
		raw_adc = calibration->max;
	}
	const uint input_range = calibration->max - calibration->min;
	const uint output_range = 127; // 7bits of midi cc data
	return (raw_adc - calibration->min) * output_range / input_range;
}

void midi_task(uint cc_a, uint cc_b) {
	static uint32_t start_ms = 0;

	// discard incoming trafic
	uint8_t packet[4];
	while (tud_midi_available()) {
		tud_midi_packet_read(packet);
	}

	// https://www.songstuff.com/recording/article/midi-message-format/
	// https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/

	// mod wheel
	uint8_t midi_cca[3] = { 0xb0 | MidiChannel, 1, 0x7f & cc_a };
	tud_midi_stream_write(MidiCable, midi_cca, 3);
	// foot pedal
	uint8_t midi_ccb[3] = { 0xb0 | MidiChannel, 4, 0x7f & cc_b };
	tud_midi_stream_write(MidiCable, midi_ccb, 3);
}

int main(void) {
	board_init();
	// initialize tiny usb before stdio as we use the usb backend
	tud_init(BOARD_TUD_RHPORT);
	stdio_init_all();

	adc_init();
	adc_gpio_init(JoystickPins[0]);
	adc_gpio_init(JoystickPins[1]);

	gpio_init(LedYellow);
	gpio_set_dir(LedYellow, GPIO_OUT);

	gpio_init(LedRed);
	gpio_set_dir(LedRed, GPIO_OUT);

	gpio_init(RotaryA);
	gpio_pull_up(RotaryA);

	gpio_init(RotaryB);
	gpio_pull_up(RotaryB);

	gpio_init(RotarySW);
	gpio_pull_up(RotarySW);

	while (true) {
		gpio_put(LedYellow, 1);
		tud_task();

		adc_select_input(JoystickInputs[0]);
		uint adc_x_raw = adc_read();
		adc_select_input(JoystickInputs[1]);
		uint adc_y_raw = adc_read();

		// dead zone 1960 1973 1892 1906
		// min-max   0492 3397 0561 3292

		//printf("\r %04u %04u", adc_x_raw, adc_y_raw);
		const uint cc_a = axis_remap_adc(&adc_a_calibration, adc_x_raw);
		const uint cc_b = axis_remap_adc(&adc_b_calibration, adc_y_raw);
		midi_task(cc_a, cc_b);

		gpio_put(LedYellow, 0);
		sleep_ms(10);
	}
}
