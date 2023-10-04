/*
AD4116.cpp
Implementation specific to AD4116.
Part of Adopticum_AD411x Analog to digital converter Arduino library.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#include "AD4116.h"
#include "AD411x_Helpers.h"

using namespace AD411x::Helpers;

// Singleton pattern implementation. Initialize the static instance.
// Note: 
// This implementation is limited to EXACTLY ONE AD4116 CONNECTED TO THE MCU.
// This class implements the singleton pattern to prevent accidental misuse.
// To enable one MCU to use multiple AD4116 peripherals at once some more
// work is required to map hadrware interrupts to the correct instance.
AD4116_Device &ad4116 = AD4116_Device::getInstance();


void AD4116_Device::configure_channel(byte channel_number, AD4116::Input input, byte setup, bool enable)
{
	AD411x_Device::configure_channel(channel_number, (uint16_t)input, setup, enable);
}


bool AD4116_Device::check_id()
{ // The ID register returns a 16-bit ID. For the AD4116, this value is 0x34Dx. (x=undefined)
	byte buf[4];
	read_register(0x07, buf, 2);
	bool is_ad4116 = (0x34 == buf[0]) && (0xd0 == (0xf0 & buf[1]));
	if (this->DEBUG) {
		if (is_ad4116) {
			Serial.println("ADC product id matches AD4116.");
		} else {
			Serial.print("ADC product id is unknown (0x");
			print_bytes(buf, 2);
			Serial.println(").");
		}
	}
	return is_ad4116;
}
