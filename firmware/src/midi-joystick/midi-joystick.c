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

void midi_task(uint adc_a, uint adc_b) {
	static uint32_t start_ms = 0;

	// discard incoming trafic
	uint8_t packet[4];
	while (tud_midi_available()) {
		tud_midi_packet_read(packet);
	}

	// https://www.songstuff.com/recording/article/midi-message-format/
	// https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/

	// mod wheel
	uint8_t midi_cca[3] = { 0xb0 | MidiChannel, 1, 0x7f & (adc_a * 128 / 4096) };
	tud_midi_stream_write(MidiCable, midi_cca, 3);
	// foot pedal
	uint8_t midi_ccb[3] = { 0xb0 | MidiChannel, 4, 0x7f & (adc_b * 128 / 4096) };
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

		printf("\r %04u %04u", adc_x_raw, adc_y_raw);
		midi_task(adc_x_raw, adc_y_raw);

		gpio_put(LedYellow, 0);
		sleep_ms(10);
	}
}
