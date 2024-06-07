#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

const uint LedYellow = 26;
const uint LedRed = 27;

const uint AudioPwm = 22;

const uint RotaryA = 17;
const uint RotaryB = 18;
const uint RotarySW = 19;

const uint JoystickPins[] = {28, 29};
const uint JoystickInputs[] = {2, 3};

int main() {
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

		const uint64_t timeBefore = time_us_64();

		adc_select_input(JoystickInputs[0]);
		uint adc_x_raw = adc_read();
		adc_select_input(JoystickInputs[1]);
		uint adc_y_raw = adc_read();

		printf("\r %04u %04u", adc_x_raw, adc_y_raw);
		sleep_ms(50);

		const uint64_t timeNow = time_us_64();
		//printf("main loop took %uus\n", (uint32_t)(timeNow - timeBefore));

		gpio_put(LedYellow, 0);
		sleep_ms(20);
	}
}