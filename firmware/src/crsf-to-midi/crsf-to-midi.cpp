#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

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

static CrsfSerial crsf(uart1);
static bool crsfNewPacket = false;

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
	uint8_t midi_cca[3] = { 0xb0 | MidiChannel, 1, static_cast<uint8_t>(0x7f & cc_a) };
	tud_midi_stream_write(MidiCable, midi_cca, 3);
	// foot pedal
	uint8_t midi_ccb[3] = { 0xb0 | MidiChannel, 4, static_cast<uint8_t>(0x7f & cc_b) };
	tud_midi_stream_write(MidiCable, midi_ccb, 3);
}

static uint8_t mapCrsfChannelToMidi(int channelData) {
	static const int crsfMin = 989;
	static const int crsfMax = 2011;
	static const int midiMax = 0x7f;
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

int main(void) {
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

	while (true) {
		gpio_put(LedYellow, 1);
		crsf.loop();
		tud_task();

		if (crsfNewPacket) {
			int sticksEATR[4] = {
				mapCrsfChannelToMidi(crsf.getChannel(1)),
				mapCrsfChannelToMidi(crsf.getChannel(2)),
				mapCrsfChannelToMidi(crsf.getChannel(3)),
				mapCrsfChannelToMidi(crsf.getChannel(4)),
			};

			//printf("CRSF packet channels: %d %d %d %d\n",
			//	sticksEATR[0], sticksEATR[1], sticksEATR[2], sticksEATR[3]);

			midi_task(sticksEATR[2], sticksEATR[3]);
			crsfNewPacket = false;
		}

		gpio_put(LedYellow, 0);
		sleep_ms(10);
	}
}
