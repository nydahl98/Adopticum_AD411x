/*
AD411x_Device.h
Abstract baseclass for AD411x devices (e.g. AD4111/4112/4113/4114/4115/4116).
Implements functionality that is shared across the AD411x family.
Part of Adopticum_AD411x Analog to digital converter Arduino library.

Created by Greger Burman, Adopticum, 2023.

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
*/

#pragma once
#include "AD411x.h"
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SPIDevice.h>

// Abstract baseclass for AD411x devices e.g. AD4111/4112/4113/4114/4115/4116.
// Implements functionality that is shared across the AD411x family.
class AD411x_Device
{
public:
	// Static configuration:
	static const bool DEBUG = false; // Extra verbosity for debugging.

	// Pure virtual destructor function. This is an abstract base class that cannot be instantiated.
	virtual ~AD411x_Device() = default;

	/*!
	@brief  Constructor/Initialization for an AD411x connected via native hardware SPI bus.
	@param  cs_pin
					Chip-select pin (using Arduino pin numbering) to enable communication with peripheral.
	@param  spi
					Pointer to an existing SPIClass instance (e.g. &SPI, the microcontroller's primary SPI bus).
	@param  bitrate
					SPI clock rate for transfers to this peripheral. Default value is 4000000UL (4 MHz).
	@note   Call the object's begin() function before use.
	*/
	void setup(int8_t cs_pin, SPIClass *spi = &SPI, uint32_t bitrate = 4000000UL);
	// void setup(int8_t cs_pin);

	// dep. default parameters. AD411x(int8_t cs_pin, SPIClass *spi = &SPI, uint32_t bitrate = 4000000UL);
	// AD411x(int8_t cs_pin, SPIClass *spi, uint32_t bitrate);
	// AD411x(int8_t cs_pin, int8_t sclk_pin, int8_t miso_pin, int8_t mosi_pin);  //TODO: Not tested!
	// private AD411x(int8_t cs_pin);

	// Call the object's begin() function before use.
	bool begin();

	// Resets the AD411x device and enables data_stat setting.
	void reset();

	// --- Generic register read and write operations. --------------------------
	// Read data of arbitrary length from a register.
	bool read_register(byte reg, byte *data, byte data_len) const;
	// Generic read operation for 16-bit registers.
	uint16_t read_register_16bit(byte reg) const;

	// Generic write operation for 16-bit registers.
	bool write_register(byte reg, uint16_t data) const;
	// Generic write operation for 8-bit registers.
	bool write_register(byte reg, byte data) const;
	// --------------------------------------------------------------------------

	// --- Specific register read and write operations. -------------------------
	void read_volt(byte *out_channel, double *out_volt);
	void read_data();
	void read_status();
	void read_many_things();

	uint16_t read_adc_mode();
	void write_adc_mode(uint16_t value);

	// --------------------------------------------------------------------------

	bool is_data_ready() const;

	bool get_data_stat() const;

	// V_REF must be configurable.
	double get_v_ref() const;
	void set_v_ref(double value);

#pragma region Configuration
	// --- Channel configuration. Channels 0 to 15. -----------------------------
public:
	bool is_channel_enabled(byte channel);
	void enable_channel(byte channel);
	void disable_channel(byte channel);

protected:
	// Generic impl to configure channels.
	// Derived classes implement specific strongly typed versions.
	void configure_channel(byte channel, uint16_t input, byte setup, bool enable = true);
	// --------------------------------------------------------------------------

	// --- Setup configuration. Setup 0 to 7. -----------------------------------
public:
	// Helper functions to create a setup value from boolean parameters.
	uint16_t make_setup(bool bipolar, bool refbuf_p, bool refbuf_n,
											bool input_buffer, bool internal_vref, bool low_level_ref);

	// Configure one of the 8 setups.
	// To set the proper bits in the register, either use the make_setup function
	// or combine the constants defined in AD4116::Setup.
	void configure_setup(byte setup_number, uint16_t setup_bits);

	// Read the setup register for one of the 8 setups.
	uint16_t read_setup_register(byte setup_number);

	// Write directly to the filter register with a 16-bit value,
	// bitwise combination of settings gives some (but low) abstraction.
	void write_filter_register(byte setup_number, uint16_t filter_bits);

	//TODO: Implement filter configuration methods with higher level of abstraction to make them easier to use.
	
	// Read the filter settings for one of the 8 setups.
	uint16_t read_filter_register(byte setup_number);

	// --------------------------------------------------------------------------
#pragma endregion

#pragma region Counters
	// Counters just to help observe and debug interrupt behaviour. -------------
public:
	uint32_t get_interrupt_count();
	uint32_t get_skip_count();
	uint32_t get_drdy_count();
	uint32_t get_read_count();

	uint32_t get_interrupt_lap();
	uint32_t get_skip_lap();
	uint32_t get_drdy_lap();
	uint32_t get_read_lap();
// ----------------------------------------------------------------------------
#pragma endregion

#pragma region Interrupt handler
	// Region with static code to handle hardware interrupts. -------------------
public:
	void enable_interrupt(int8_t interrupt_pin);
	void disable_interrupt();

protected:
	// Called by interrupt handler when data is available on this instance.
	void on_data_ready();
	void clear_data_ready();

	// Static members. Mainly to handle hardware interrupts.
	static int8_t interrupt_pin;
	static void on_interrupt();

// ----------------------------------------------------------------------------
#pragma endregion

protected:
	double v_ref;
	double bipolar_factor;
	double unipolar_factor;

	Adafruit_SPIDevice *spi_dev = NULL;
	int8_t cs_pin;
	bool data_stat;

	// Constructor is never called from outside.
	// Initializes member variables with default values.
	AD411x_Device();

	// Write data of arbitrary length to a register.
	bool write_register(byte reg, byte *data, byte data_len) const;

	// We always want data_stat to be enabled. Don't let user disable it.
	void set_data_stat(bool value);
};
