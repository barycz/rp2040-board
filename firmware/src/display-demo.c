#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "ssd1306.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

const uint LedYellow = 26;
const uint LedRed = 27;

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

	i2c_init(DISP_I2C, 400000);
	gpio_set_function(DispSda, GPIO_FUNC_I2C);
	gpio_set_function(DispSck, GPIO_FUNC_I2C);
	gpio_pull_up(DispSda);
	gpio_pull_up(DispSck);

	ssd1306_t disp;
	disp.external_vcc=false;
	ssd1306_init(&disp, 128, 64, 0x3C, DISP_I2C);
	ssd1306_clear(&disp);

	while (true) {
		gpio_put(LedYellow, 1);
		ssd1306_draw_string(&disp, 0, 0, 1, "Hello World!");
		ssd1306_show(&disp);
		sleep_ms(250);
		
		gpio_put(LedYellow, 0);
		sleep_ms(250);
	}
}