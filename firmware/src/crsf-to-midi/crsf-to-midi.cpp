#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <memory>

#include "CrsfSerial.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/timer.h"

#include "bsp/board.h"
#include "tusb.h"

const uint LedYellow = 26;
const uint LedRed = 27;

const uint AudioPwm = 22;

const uint RotaryA = 17;
const uint RotaryB = 18;
const uint RotarySW = 19;

const uint CrsfTx = 20;
const uint CrsfRx = 21;

const uint JoystickPins[] = {28, 29};
const uint JoystickInputs[] = {2, 3};

const uint MidiCable = 0;
const uint MidiChannel = 0;
const uint MidiDataMask = 0x7f; // 7bits

static CrsfSerial crsf(uart1);
static bool crsfNewPacket = false;

struct MidiControllerData {
	uint8_t Controller;
	uint8_t Data;
};

static const uint ControllerDataMaxCount = 2;
using ControllerDataArray = std::array<MidiControllerData, ControllerDataMaxCount>;

void midi_task(const ControllerDataArray& dataArray) {
	// discard incoming trafic
	uint8_t packet[4];
	while (tud_midi_available()) {
		tud_midi_packet_read(packet);
	}

	// https://www.songstuff.com/recording/article/midi-message-format/
	// https://anotherproducer.com/online-tools-for-musicians/midi-cc-list/

	for (const auto& controllerData: dataArray) {
		uint8_t midiCC[3] = {
			0xb0 | MidiChannel,
			controllerData.Controller,
			static_cast<uint8_t>(MidiDataMask & controllerData.Data)
		};
		tud_midi_stream_write(MidiCable, midiCC, 3);
	}
}

static uint8_t mapCrsfChannelToMidi(int channelData) {
	static const int crsfMin = 989;
	static const int crsfMax = 2011;
	static const int midiMax = MidiDataMask;
	if (channelData < crsfMin) {
		return 0;
	}
	if (channelData > crsfMax) {
		return midiMax;
	}
	return (channelData - crsfMin) * midiMax / (crsfMax - crsfMin);
}

static void onCrsfPacketChannels() {
	crsfNewPacket = true;
}

static void onCrsfLinkUp() {
	printf("CRSF link up\n");
}

static void onCrsfLinkDown() {
	printf("CRSF link down\n");
}

static void onCrsfOobData(uint8_t b) {
	//printf("CRSF oob data %x", static_cast<uint>(b));
}

void crsf_to_midi_update() {
	if (crsfNewPacket == false) {
		return;
	}

	int sticksEATR[4] = {
		mapCrsfChannelToMidi(crsf.getChannel(1)),
		mapCrsfChannelToMidi(crsf.getChannel(2)),
		mapCrsfChannelToMidi(crsf.getChannel(3)),
		mapCrsfChannelToMidi(crsf.getChannel(4)),
	};

	//printf("CRSF packet channels: %d %d %d %d\n",
	//	sticksEATR[0], sticksEATR[1], sticksEATR[2], sticksEATR[3]);

	ControllerDataArray dataArray;
	dataArray[0].Controller = 1; // mod wheel
	dataArray[0].Data = sticksEATR[2];
	dataArray[1].Controller = 4; // foot pedal
	dataArray[1].Data = sticksEATR[3];
	midi_task(dataArray);

	crsfNewPacket = false;
}

void init() {
	board_init();
	// initialize tiny usb before stdio as we use the usb backend
	tud_init(BOARD_TUD_RHPORT);
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

	gpio_set_function(CrsfTx, GPIO_FUNC_UART);
	gpio_set_function(CrsfRx, GPIO_FUNC_UART);

	crsf.onPacketChannels = &onCrsfPacketChannels;
	crsf.onLinkUp = &onCrsfLinkUp;
	crsf.onLinkDown = &onCrsfLinkDown;
	crsf.onOobData = &onCrsfOobData;
	crsf.begin();
}

void update() {
	gpio_put(LedYellow, 1);

	crsf.loop();
	tud_task();
	crsf_to_midi_update();

	gpio_put(LedYellow, 0);
	sleep_ms(10);
}

int main() {
	init();

	while(true) {
		update();
	}
}
