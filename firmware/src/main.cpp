#include <Arduino.h>

const pin_size_t LedYellow = 26;
const pin_size_t LedRed = 27;

const pin_size_t DmxTx = 24;
const pin_size_t DmxRx = 25;
const pin_size_t DmxRts = 23;

// not using uart flow control as it does not work in arduino framework
UART dmx(DmxTx, DmxRx, NC, NC);

void setup() {
	pinMode(LedYellow, OUTPUT);
	pinMode(LedRed, OUTPUT);

	pinMode(DmxRts, OUTPUT);
	digitalWrite(DmxRts, HIGH);

	dmx.begin(250000);
}

void updateLeds() {
	const PinStatus ledStates[] = {LOW, HIGH};
	static uint8_t iter = 0;
	digitalWrite(LedRed, ledStates[iter++ % 2]);
	digitalWrite(LedYellow, ledStates[iter % 2]);
}

void loop() {
	dmx.println("test");
	updateLeds();
	sleep_ms(100);
}
