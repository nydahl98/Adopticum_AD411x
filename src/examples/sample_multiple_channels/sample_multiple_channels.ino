/*
sample_multiple_channels.ino
Example code for Adopticum_AD411x Analog to Digital converter Arduino library.

This example connects to an AD4116 device connected to the Arduino's native SPI bus.
We configure the AD4116 to sample on multiple channels continuously and output data
at the configured data rate.

Note:
For interrupt handling to work, connect DRDY pin to CIPO (a.k.a. MISO) pin.
With a default Arduino setup that would be pin 3 for DRDY and pin 12 CIPO.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#include <Adopticum_AD411x.h>

namespace config
{
  const char *Program = "sample_multiple_channels";
  const char *Version = "2023-10-03";

  // Only REQUIRE serial connection to computer in debugging scenario.
  const bool SerialDebug = false;
  const long SerialSpeed = 115200;

  // Pin on Arduino connected to chip select on AD411x.
  const int CS_PIN = 10;
  // Pin on Arduino to trigger interrupt when data is ready.
  // Connect DRDY pin to CIPO (a.k.a. MISO) pin.
  const int DRDY_PIN = 3;
  // Select an output data rate for the AD4116.
  const uint16_t ODR = (uint16_t)AD4116::OutputDataRate::SPS_500;
}

void setup_ad4116()
{
  // Reset and configure AD411x.
  ad4116.reset();

  // Configure channel 0 to use setup 0 and measure voltage between VIN0 and VIN1.
  ad4116.configure_channel(0, AD4116::Input::VIN0_VIN1, 0);

  // Configure channel 1 to use setup 0 and measure voltage between VIN2 and VIN3.
  ad4116.configure_channel(1, AD4116::Input::VIN2_VIN3, 0);

  // Configure channel 2 to use setup 0 and measure voltage between VIN4 and VINCOM.
  ad4116.configure_channel(2, AD4116::Input::VIN4_VINCOM, 0);

  // Configure setup 0 to measure bipolar voltage with external reference voltage,
  // enable input buffer, REF+ and REF- buffers.
  uint16_t setup_cfg = AD411x::Setup::BIPOLAR | AD411x::Setup::EXTERNAL_REF 
    | AD411x::Setup::INPUT_BUFFERS | AD411x::Setup::REFBUF_P | AD411x::Setup::REFBUF_N;
  ad4116.configure_setup(0, setup_cfg);

  // Configure setup 0 to use default filtering (sinc5+sinc1 + 20 SPS 50/60Hz rejection)
  ad4116.write_filter_register(0, AD411x::Filter::DEFAULT_FILTER | config::ODR);

  // Configure ADC mode to continuous measurement.
  ad4116.write_adc_mode((uint16_t)AD411x::ADCMode::Mode::CONTINUOUS);

  // Use one pin for data ready interrupt.
  ad4116.enable_interrupt(config::DRDY_PIN);
}

void setup()
{
  // Generic initialisation.
  if (config::SerialDebug)
  { while (!Serial) { delay(10); } }
  Serial.begin(config::SerialSpeed);
  Serial.println(config::Program);
  Serial.println(config::Version);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Setup analog outputs as sample signals.
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A0, LOW); // keep A0 as "GND" always.

  // Setup communication with AD411x.
  ad4116.setup(config::CS_PIN);
  if (!ad4116.begin() || !ad4116.check_id())
  {
    Serial.println("Could not initialize AD411x device.");
    while (1) {}
  }
  Serial.println("AD411x initialized.");

  // Configure AD4116.
  setup_ad4116();

  Serial.println("setup done.");
}

byte channel;
double volt;
double samples[3] = {0.0, 0.0, 0.0};

uint32_t output_counter = 0;
uint32_t toggle_counter = 0;

void loop()
{
  // AD411x is set to continuous mode.
  // We do nothing until there is data ready to read.
  if (!ad4116.is_data_ready())
  {
    return;
  }

  // Read the latest sample from the AD-converter.
  ad4116.read_volt(&channel, &volt);
  samples[channel] = volt;

  auto half_second = millis() / 500;

  // Limit printing outout to  only once per halv second.
  if (half_second <= output_counter)
  {
    return;
  }
  output_counter++;

  // Print the latest samples.
  Serial.print("Latest samples:");
  for (auto i = 0; i < 3; i++)
  {
    Serial.print(" ch ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(samples[i]);
    Serial.print(" V ");
  }
  Serial.println("");

  // Limit toggling outputs to only once every 2 seconds (4 halv seconds).
  if (output_counter / 4 <= toggle_counter)
  {
    return;
  }
  toggle_counter++;

  // Toggle some outputs to use as example analog signals.
  byte toggle = toggle_counter % 2;
  Serial.print("Toggle output: ");
  Serial.println(toggle);
  // Use digitalWrite to get 0V or 5V, instead of PWM.
  digitalWrite(A1, toggle);     // analogWrite(A1, 255 * toggle);
  digitalWrite(A2, toggle);     // analogWrite(A2, 255 * toggle);
  digitalWrite(A3, 1 - toggle); // analogWrite(A3, 255 * (1-toggle));
}
