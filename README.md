# Adopticum AD411x
AD411x Analog to Digital converter Arduino library.


## Installation

- Clone the Adopticum_AD411x repository from GitHub to your computer.
- Put the "Adopticum_AD411x" folder inside the "libraries" folder.
In your Arduino sketchbook folder is a subfolder called "libraries".
- Install the dependency: Adafruit BusIO library from the library manager.
- Restart the Arduino IDE.
- Open an example from the main menu by selecting e.g.
*File > Examples > Adopticum_AD411x > hello_ad4116*


## License

This work is open source and licensed under the Boost Software License.

We also reuse source code from the following libraries. Please observe their respective copyright and license notices.

- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)


## About the AD4116 Analog to Digital converter

The AD4116 is a low power, low noise, 24-bit, Σ-Δ analog-to-digital converter (ADC) that integrates an analog front end (AFE) for six fully differential or eleven single-ended, high impedance (≥10 MΩ) bipolar, ±10 V voltage inputs. The additional two differential or four single-ended/pseudo differential direct ADC inputs provides excellent performance at lower input ranges.

The AD4116 also integrates key analog and digital signal conditioning blocks to configure eight individual setups for each analog input channel in use. As many as 16 channels can be enabled at any time. A channel is defined as any of the standard analog voltage inputs or a low level direct ADC input. The AD4116 features a maximum channel scan rate of 12,422 SPS (80 μs) using a sinc5 + sinc1 filter and 20,618 SPS per channel (48 μs) using a sinc3 filter.

The embedded 2.5 V, low drift (5 ppm/°C), band gap internal reference (with output reference buffer) reduces the external component count.

The digital filter allows flexible settings, including simultaneous 50 Hz and 60 Hz rejection at a 27.27 SPS output data rate. The user can select between the different filter settings depending on the demands of each channel in the application. The automatic channel sequencer enables the ADC to switch through each enabled channel.

The precision performance of the AD4116 is achieved by integrating the proprietary iPassives® technology from Analog Devices, Inc.

The AD4116 operates with a single power supply, making it easy to use in galvanically isolated applications. The specified operating temperature range is −40°C to +105°C. The AD4116 is housed in a 40-lead, 6 mm × 6 mm LFCSP.

- 24-bit ADC with integrated analog front end.
- Output data rate from 1.25 SPS to 62.5 kSPS.
- Channel scan data rates:
  + 12,422 SPS per channel (80 μs settling, sinc5 + sinc1 filter).
  + 20,618 SPS per channel (48 μs settling, sinc3 filter).
  + 85 dB rejection of 50 Hz and 60 Hz at 20 SPS per channel.
- 6 differential or 11 single-ended inputs +-10 V.
  + Pin absolute maximum rating: ±65 V
  + Absolute input pin voltage up to ±20 V
  + ≥10 MΩ impedance
- 2 differential or 4 single-ended low level inputs +-VREF.
  + Absolute input pin voltage, AVDD to AVSS
  + True rail-to-rail analog input buffers
- On-chip 2.5 V reference with ±0.12% accuracy at 25°C, ±5 ppm/°C (typical) drift
- Internal or external clock.
- Power specs:
  + AVDD to AVSS = 4.5 V to 5.5 V
  + IOVDD to DGND = 2 V to 5.5 V
  + Total IDD (AVDD + IOVDD) = 6.15 mA
- Temperature range: −40°C to +105°C
- 3-wire or 4-wire serial digital interface (Schmitt trigger on SCLK)
  + SPI, QSPI, MICROWIRE, and DSP compatible

[AD4116](https://www.analog.com/en/products/ad4116.html)


## Evaluation boards

EVAL-AD4116 is an evaluation board for the AD4116. It has standard Arduino headers. It can be powered with 5V from an Arduino board or via a separate power connector.

EVAL-SDP-CB1Z is a mezzanine board to be used together with the EVAL-AD4116 (among others). It connects to a PC via USB cable and demo software from Analog Devices (created with LabView) is available for Windows.
