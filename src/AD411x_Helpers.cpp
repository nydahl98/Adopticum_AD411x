/*
AD411x_Helpers.cpp
Some generic static helper methods.
Part of Adopticum_AD411x Analog to digital converter Arduino library.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#include <Arduino.h>
#include "AD411x_Helpers.h"

namespace AD411x
{
	namespace Helpers
	{
		// Some generic helper functions. ---------------------------------------------

		byte set_bit(byte data, byte bit, bool value)
		{
			byte mask = 0x01 << bit;
			bit = value ? 0xff : 0x00;
			data = (data & ~mask) | (bit & mask);
			return data;
		}

		void print_bytes(byte *data, byte data_len)
		{
			char sz[4];
			for (auto i = 0; i < data_len; i++)
			{
				snprintf(sz, 4, " %02X", data[i]);
				Serial.print(sz);
			}
		}
	}
}