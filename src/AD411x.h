/*
AD411x.h
Definitions, types and constants that are shared across the AD411x family.
Part of Adopticum_AD411x Analog to digital converter Arduino library.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#pragma once
#include "Adafruit_SPIDevice.h"

// Definitions, types and constants that are shared across the AD411x family.
namespace AD411x
{
	// The AD411x communicates with clock polarity = 1 and clock phase = 1 aka
	// SPI mode 3 data shifted out on falling clk and data sampled on rising clk.
	static const uint8_t DATA_MODE = SPI_MODE3;
	static const BusIOBitOrder BIT_ORDER = SPI_BITORDER_MSBFIRST;


	// Enumeration of all registers in the AD411x family.
	// All access to the on-chip registers must start with a write to the communications register. 
	// This write determines which register is accessed next and whether that operation is a write or a read.
	namespace Register
	{
		enum
		{
			COMMS = 0x00,
			STATUS = 0x00,
			ADCMOD = 0x01,
			IFMODE = 0x02,
			REGCHECK = 0x03,
			DATA = 0x04,
			GPIOCON = 0x06,
			ID = 0x07,
			// ADC channel registers.
			CH0 = 0x10,
			CH1 = 0x11,
			CH2 = 0x12,
			CH3 = 0x13,
			CH4 = 0x14,
			CH5 = 0x15,
			CH6 = 0x16,
			CH7 = 0x17,
			CH8 = 0x18,
			CH9 = 0x19,
			CH10 = 0x1A,
			CH11 = 0x1B,
			CH12 = 0x1C,
			CH13 = 0x1D,
			CH14 = 0x1E,
			CH15 = 0x1F,
			// ADC setup config registers.
			SETUP0 = 0x20,
			SETUP1 = 0x21,
			SETUP2 = 0x22,
			SETUP3 = 0x23,
			SETUP4 = 0x24,
			SETUP5 = 0x25,
			SETUP6 = 0x26,
			SETUP7 = 0x27,
			// ADC filter config registers.
			FILTER0 = 0x28,
			FILTER1 = 0x29,
			FILTER2 = 0x2A,
			FILTER3 = 0x2B,
			FILTER4 = 0x2C,
			FILTER5 = 0x2D,
			FILTER6 = 0x2E,
			FILTER7 = 0x2F,
			// ADC offset registers.
			OFFSET0 = 0x30,
			OFFSET1 = 0x31,
			OFFSET2 = 0x32,
			OFFSET3 = 0x33,
			OFFSET4 = 0x34,
			OFFSET5 = 0x35,
			OFFSET6 = 0x36,
			OFFSET7 = 0x37,
			// ADC gain registers.
			GAIN0 = 0x38,
			GAIN1 = 0x39,
			GAIN2 = 0x3A,
			GAIN3 = 0x3B,
			GAIN4 = 0x3C,
			GAIN5 = 0x3D,
			GAIN6 = 0x3E,
			GAIN7 = 0x3F
		};
	}


	// The setup configuration registers are 16-bit registers
	// that configure the reference selection, input buffers
	// and output coding of the ADC.
	// The layout for SETUPCON0 to SETUPCON7 is identical.
	// Setup register is identical for all AD411x devices. Correct?
	namespace Setup
	{
		// bits 15..13 are reserved. Set to 0!

		// bit 12 Bipolar and Unipolar output coding.
		// 0 = unipolar coded output, 1 = bipolar coded output.
		const uint16_t UNIPOLAR = 0x0000;
		const uint16_t BIPOLAR = 0x1000;

		// bit 11 REFBUF+ This bit enables or disables the REF+ input buffer.
		const uint16_t REFBUF_P = 0x0800;

		// bit 10 REFBUF- This bit enables or disables the REF- input buffer.
		const uint16_t REFBUF_N = 0x0400;

		// bits 9..8 INBUF enables or disables input buffers.
		// 00 = Disable input buffers.
		// 01 = Reserved.
		// 10 = Reserved.
		// 11 = Enable input buffers.
		const uint16_t INPUT_BUFFERS = 0x0300;

		// bits 7..6 are reserved. Set to 0!

		// bits	5..4 REF_SEL selects the reference source for ADC conversion.
		// 00 = External reference (REF+-).
		// 10 = Internal reference (2.5V). Must also be enabled in via ADCMODE.
		// 11 = AVDD - AVSS. Low voltage reference.
		const uint16_t EXTERNAL_REF = 0x0000;
		const uint16_t INTERNAL_REF = 0x0020;
		const uint16_t AVDD_AVSS_REF = 0x0030;

		// bits 3..0 are reserved. Set to 0!
		const uint16_t RESERVED = 0x0000;

		// Default setup setting is (0x1000) bipolar, external reference, no buffers.
		//TODO: DEFAULT unusable because of namespace pollution from macro "#define DEFAULT" in Arduino.h.
		const uint16_t DEFAULT_SETUP = Setup::BIPOLAR;
	};

	// The filter configuration registers are 16-bit registers that configure
	// the ADC data rate and filter options. Writing to any of these registers
	// resets any active ADC conversion and restarts converting at the first
	// channel in the sequence. The layout for FILTCON0 to FILTCON7 is identical.
	namespace Filter
	{
		// bit 15 SINC3_MAP
		// Mapping of the filter register changes to directly program the decimation rate
		// of the sinc3 filter. All other options are eliminated.
		// This bit allows fine tuning of the output data rate and
		// filter notch for rejection of specific frequencies.
		// The data rate when on a single channel equals fMOD/(32 × FILTCON0[14:0]).
		const uint16_t SINC3_MAP = 0x8000;

		// bits 14..12 are reserved. Set to 0!

		// bit 11 ENHFILTEN enables various post filters for enhanced 50 Hz/60 Hz rejection.
		// The ORDER bits must be set to 00 to select the sinc5 + sinc1 filter for this function to work.
		const uint16_t ENHFILT_EN = 0x8000;

		// bits 10..8 ENHFILT selects between various post filters for enhanced 50 Hz/60 Hz rejection.
		const uint16_t ENHFILT_27SPS = 0x0200;
		const uint16_t ENHFILT_25SPS = 0x0300;
		const uint16_t ENHFILT_20SPS = 0x0500;
		const uint16_t ENHFILT_16SPS = 0x0600;

		// bit 7 is reserved. Set to 0!

		// bits 6..5 ORDER control the order of the digital filter that processes the modulator data.
		const uint16_t SINC5_SINC1 = 0x0000;
		const uint16_t SINC3 = 0x0060;

		// bits 4..0 Output datarate (ODR)
		// These bits control the  output data rate of the ADC and thus the settling time and noise.
		// These are specific to each member of the AD411x family.
		// e.g. AD4116::Filter.

		// Default setup setting is (0x0500).
		// 20 SPS, 86 dB rejection, and 50 ms settling time for enhanced 50 Hz/60 Hz rejection.
		//TODO: DEFAULT unusable because of namespace pollution from macro "#define DEFAULT" in Arduino.h.
		const uint16_t DEFAULT_FILTER = Filter::ENHFILT_20SPS;
	}

	// ADC MODE REGISTER
	// Address: 0x01, Reset: 0x2000, Name: ADCMODE
	// Controls the operating mode of the ADC and the master clock selection.
	// A write to the ADC mode register resets the filter and
	// the RDY bits and starts a new conversion or calibration.
	namespace ADCMode
	{
		// bit 15 REF_EN
		// Enables internal reference and outputs a buffered 2.5 V to the REFOUT pin.
		const uint16_t REF_EN = 0x8000;

		// bit 14 is reserved. Set to 0!

		// bit 13 SING_CYC
		// This bit can be used when only a single channel is active to
		// set the ADC to only output at the settled filter data rate.
		const uint16_t SING_CYC = 0x2000;

		// bits 12..11 reserved. Set to 0!

		// bits 10..8 DELAY
		// These bits allow a programmable delay to be added after a channel switch to
		// allow the settling of external circuitry before the ADC starts processing its input.
		// TODO: The actual delays in microseconds are device specific.
		enum class Delay : uint16_t
		{
			DELAY_0 = 0x0000, // 000  0 us on all AD411x devices.
			DELAY_1 = 0x0100, // 001
			DELAY_2 = 0x0200, // 010
			DELAY_3 = 0x0300, // 011
			DELAY_4 = 0x0400, // 100
			DELAY_5 = 0x0500, // 101
			DELAY_6 = 0x0600, // 110
			DELAY_7 = 0x0700  // 111  4 ms on AD4116, 8 ms on AD4111.
		};

		// bit 7 is reserved. Set to 0!

		// bits 6..4 MODE
		// These bits control the operating mode of the ADC.
		// See the Operating Modes section in datasheet for more information.
		enum class Mode : uint16_t
		{
			CONTINUOUS = 0x0000,				  // 000 Continuous conversion mode. (default)
			SINGLE_CONV = 0x0010,				  // 001 Single conversion mode.
			STANDBY = 0x0020,					  // 010 Standby mode.
			POWER_DOWN = 0x0030,				  // 011 Power-down mode.
			INTERNAL_OFFSET_CALIBRATION = 0x0040, // 100 Internal offset calibration.
			INTERNAL_GAIN_CALIBRATION = 0x0050,	  // 101 Internal gain calibration
			SYSTEM_OFFSET_CALIBRATION = 0x0060,	  // 110 System offset calibration.
			SYSTEM_GAIN_CALIBRATION = 0x0070	  // 111 System gain calibration.
		};

		// bits 3..2 CLOCKSEL
		// Select the ADC clock source. Selecting the internal oscillator
		// also enables the internal oscillator.
		enum class Clock : uint16_t
		{
			INTERNAL_OSCILLATOR = 0x0000,			   // 00 Internal oscillator.
			INTERNAL_WITH_OUTPUT = 0x0004, // 01 Internal oscillator output on the XTAL2/CLKIO pin.
			EXTERNAL_INPUT = 0x0008,	   // 10 External clock input on the XTAL2/CLKIO pin.
			EXTERNAL_CRYSTAL = 0x000C,	   // 11 External crystal on the XTAL1 pin and the XTAL2/CLKIO pin.
		};

		// bits 1..0 reserved. Set to 0!

		// Default is 0x2000
		//TODO: DEFAULT unusable because of namespace pollution from macro "#define DEFAULT" in Arduino.h.
		const uint16_t DEFAULT_ADCMODE = SING_CYC;
	}


	//TODO: WIP strongly typed parameters to functions to simplify usage by completion.
}