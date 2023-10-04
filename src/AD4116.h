/*
AD4116.h
Include this file to use one AD4116 (singleton instance) connected to your MCU.
Part of Adopticum_AD411x Analog to digital converter Arduino library.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#pragma once
#include "AD411x.h"
#include "AD411x_Device.h"


// Definitions, types and constants specific to the AD4116.
namespace AD4116 
{
	//Definitions for input channel selection bits for AD4116.
	// These bits select which input pair is connected to the input of the ADC for a channel.
	// These are bits [9..0] in the CHx registers, named INPUTx.
	enum class Input : uint16_t
	{
		VIN0_VIN1 = 0b0000000001,	 // VIN0 and VIN1.
		VIN0_VINCOM = 0b0000010000,	 // VIN0 and VINCOM.
		VIN1_VIN0 = 0b0000100000,	 // VIN1 and VIN0.
		VIN1_VINCOM = 0b0000110000,	 // VIN1 and VINCOM.
		VIN2_VIN3 = 0b0001000011,	 // VIN2 and VIN3.
		VIN2_VINCOM = 0b0001010000,	 // VIN2 and VINCOM.
		VIN3_VIN2 = 0b0001100010,	 // VIN3 and VIN2.
		VIN3_VINCOM = 0b0001110000,	 // VIN3 and VINCOM.
		VIN4_VIN5 = 0b0010000101,	 // VIN4 and VIN5.
		VIN4_VINCOM = 0b0010010000,	 // VIN4 and VINCOM.
		VIN5_VIN4 = 0b0010100100,	 // VIN5 and VIN4.
		VIN5_VINCOM = 0b0010110000,	 // VIN5 and VINCOM.
		VIN6_VIN7 = 0b0011000111,	 // VIN6 and VIN7.
		VIN6_VINCOM = 0b0011010000,	 // VIN6 and VINCOM.
		VIN7_VIN6 = 0b0011100110,	 // VIN7 and VIN6.
		VIN7_VINCOM = 0b0011110000,	 // VIN7 and VINCOM.
		VIN8_VIN9 = 0b0100001001,	 // VIN8 and VIN9.
		VIN8_VINCOM = 0b0100010000,	 // VIN8 and VINCOM.
		VIN9_VIN8 = 0b0100101000,	 // VIN9 and VIN8.
		VIN9_VINCOM = 0b0100110000,	 // VIN9 and VINCOM.
		VIN10_VINCOM = 0b0101010000, // VIN10, VINCOM (single-ended or differential pair)

		ADCIN11_ADCIN12 = 0b0101101100, // ADCIN11, ADCIN12.
		ADCIN12_ADCIN11 = 0b0110001011, // ADCIN12, ADCIN11.
		ADCIN13_ADCIN14 = 0b0110101110, // ADCIN13, ADCIN14.
		ADCIN14_ADCIN13 = 0b0111001101, // ADCIN14, ADCIN13.
		ADCIN11_ADCIN15 = 0b0101101111, // ADCIN11, ADCIN15. (pseudo differential or differential pair)
		ADCIN12_ADCIN15 = 0b0110001111, // ADCIN12, ADCIN15. (pseudo differential or differential pair)
		ADCIN13_ADCIN15 = 0b0110101111, // ADCIN13, ADCIN15. (pseudo differential or differential pair)
		ADCIN14_ADCIN15 = 0b0111001111, // ADCIN14, ADCIN15. (pseudo differential or differential pair)
		Temperature = 0b1000110010,		// Temperature sensor.
		Reference = 0b1010110110		// Reference.
	};

	// These bits control the output data rate of the ADC4116 specifically 
	// and thus also the settling time and noise for Setup x (0 to 7). 
	// Rates shown are for single channel enabled sinc5 + sinc 1 filter.
	// See Table 7 for multiple channels enabled.
	enum class OutputDataRate : uint16_t
	{
		SPS_62500_0 = 0b00000,  // 62500 SPS.
		SPS_62500_1 = 0b00001,  // 62500 SPS.
		SPS_62500_2 = 0b00010,  // 62500 SPS.
		SPS_62500_3 = 0b00011,  // 62500 SPS.
		SPS_31250_0 = 0b00100,  // 31250 SPS.
		SPS_31250_1 = 0b00101,  // 31250 SPS.
		SPS_15625   = 0b00110,  // 15625 SPS.
		SPS_10416   = 0b00111,  // 10416.7 SPS.
		SPS_5194    = 0b01000,  // 5194.8 SPS (5208.3 SPS for sinc3).
		SPS_2496    = 0b01001,  // 2496.9 SPS (2500 SPS for sinc3).
		SPS_1007    = 0b01010,  // 1007.6 SPS (1008.1 SPS for sinc3).
		SPS_500     = 0b01011,  // 499.9 SPS (500 SPS for sinc3).
		SPS_390     = 0b01100,  // 390.6 SPS (400.64 SPS for sinc3).
		SPS_200     = 0b01101,  // 200.3 SPS (200.32 SPS for sinc3).
		SPS_100     = 0b01110,  // 100.0 SPS.
		SPS_60      = 0b01111,  // 59.75 SPS (59.98 SPS for sinc3).
		SPS_50      = 0b10000,  // 49.84 SPS (50 SPS for sinc3).
		SPS_20      = 0b10001,  // 20.00 SPS.
		SPS_16      = 0b10010,  // 16.65 SPS (16.67 SPS for sinc3).
		SPS_10      = 0b10011,  // 10.00 SPS.
		SPS_5       = 0b10100,  // 5.00 SPS.
		SPS_2       = 0b10101,  // 2.50 SPS.
		SPS_1       = 0b10110   // 1.25 SPS.
	};
}


// Note: 
// This implementation is limited to EXACTLY ONE AD4116 CONNECTED TO THE MCU.
// This class implements the singleton pattern to prevent accidental misuse.
// To enable one MCU to use multiple AD4116 peripherals at once some more
// work is required to map hadrware interrupts to the correct instance.

// AD4116_Device represents an instance of the AD4116 device.
// Inherits from the abstract baseclass AD411x_Device, which 
// implements functionality that is common across the AD411x family.
class AD4116_Device : public AD411x_Device
{
#pragma region singleton pattern
// Region with singleton pattern implementation -------------------------------
	private:
		// Singleton pattern. First make the c-tors private.
		AD4116_Device() : AD411x_Device() {}

	public:
		// Aquire reference to the one and only instance, which is statically initialized.
		static AD4116_Device &getInstance()
		{
			static AD4116_Device instance; // Guaranteed to be destroyed. Instantiated on first use.
			return instance;
		}

		// Ensure no implementations for copy and assignment.
		// Make sure they are inaccessible (especially from outside), 
		// to prevent accidental copies of your singleton.
		// Dirrerent solution for C++ 11 onwards compares to legacy C++ 03.

		// C++ 11 and onwards
		// We can use this technique for deleting the methods we don't want.
		AD4116_Device(AD4116_Device const&)   = delete;
		void operator=(AD4116_Device const&)  = delete;
	// ------------------------------------------------------------------------
#pragma endregion

	public:
		void configure_channel(byte channel, AD4116::Input input, byte setup, bool enable = true);

		bool check_id();
};

// Singleton instance declaration.
extern AD4116_Device &ad4116;
