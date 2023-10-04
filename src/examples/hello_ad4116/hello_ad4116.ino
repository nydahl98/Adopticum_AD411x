/*
hello_ad4116.ino
Example code for Adopticum_AD411x Analog to Digital converter Arduino library.

This example connects to an AD4116 device connected to the Arduino's native SPI bus and
prints a message to indicate if communication was successful.
Every 10 seconds it reads some registers on the AD4116 and prints them to the serial port.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#include <Adopticum_AD411x.h>

namespace config {
  const char *Program = "hello_ad4116";
  const char *Version = "2023-09-26";

	// Only REQUIRE serial connection to computer in debugging scenario. 
	const bool SerialDebug = false;
  const long SerialSpeed = 115200;

  // Pin on Arduino connected to chip select on AD411x.
  const int CS_PIN = 10;
}


void setup() 
{
	// Generic initialisation.
	if (config::SerialDebug) { while (!Serial) { delay(10); } }
	Serial.begin(config::SerialSpeed);
  Serial.println(config::Program);
	Serial.println(config::Version);
	pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup communication with AD411x.
  ad4116.setup(config::CS_PIN);
  if (!ad4116.begin() || !ad4116.check_id()) {
    Serial.println("Could not initialize AD411x device.");
    while (1);
  }
  ad4116.reset();
  Serial.println("AD411x initialized.");
}


void loop() 
{
  Serial.println("---");
  ad4116.read_many_things();
  delay(10000);
}
