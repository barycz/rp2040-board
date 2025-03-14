#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "bsp/board.h"
#include "tusb.h"

const uint LedYellow = 26;
const uint LedRed = 27;
const uint DataDebugUart = 1; // 0 is reserved for the pico stdio

uint64_t lastEcho;
uint64_t lastData;

void placeholder_task() {
	const uint64_t now = time_us_64();

	if (lastEcho + 1000000 < now) {
		printf("uart spam\r\n");
		lastEcho = now;
	}

	if (lastData + 10000 < now) {
		char buffer[255];
		sprintf(buffer, "%u,10\r\n", now);
		tud_cdc_n_write_str(DataDebugUart, buffer);
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

	lastData = lastEcho = time_us_64();

	while (true) {
		gpio_put(LedYellow, 1);

		tud_task();
		placeholder_task();

		gpio_put(LedYellow, 0);
		sleep_ms(10); // __wfi();
	}
}
