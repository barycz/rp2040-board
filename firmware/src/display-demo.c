#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "ssd1306.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

const uint LedYellow = 26;
const uint LedRed = 27;

const uint AudioPwm = 22;

const uint RotaryA = 17;
const uint RotaryB = 18;
const uint RotarySW = 19;

const uint DmxTx = 24;
const uint DmxRx = 25;
const uint DmxRts = 23;

const uint DispSda = 28;
const uint DispSck = 29;
#define DISP_I2C (&i2c0_inst)

int main() {
	stdio_init_all();

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

	ssd1306_t disp;
	disp.external_vcc=false;
	ssd1306_init(&disp, 128, 64, 0x3C, DISP_I2C);

	while (true) {
		gpio_put(LedYellow, 1);

		ssd1306_clear(&disp);
		ssd1306_draw_string(&disp, 0, 0, 1, "Hello World!");
		if (gpio_get(RotaryA)) {
			ssd1306_draw_square(&disp, 0, 16, 16, 16);
		} else {
			ssd1306_draw_empty_square(&disp, 0, 16, 16, 16);
		}
		if (gpio_get(RotaryB)) {
			ssd1306_draw_square(&disp, 32, 16, 16, 16);
		} else {
			ssd1306_draw_empty_square(&disp, 32, 16, 16, 16);
		}
		if (gpio_get(RotarySW)) {
			ssd1306_draw_square(&disp, 64, 16, 16, 16);
		} else {
			ssd1306_draw_empty_square(&disp, 64, 16, 16, 16);
		}
		ssd1306_show(&disp);

		gpio_put(LedYellow, 0);
		sleep_ms(20);
	}
}