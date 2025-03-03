/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2014 by Ray Wang (ray@opensprinkler.com)
 *
 * GPIO functions
 * Feb 2015 @ OpenSprinkler.com
 *
 * This file is part of the OpenSprinkler library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "gpio.h"

#if defined(ARDUINO)

#if defined(ESP8266)

#include <Wire.h>
#include "defines.h"

byte IOEXP::detectType(uint8_t address) {
	Wire.beginTransmission(address);
	if(Wire.endTransmission()!=0) return IOEXP_TYPE_NONEXIST; // this I2C address does not exist

	Wire.beginTransmission(address);
	Wire.write(NXP_INVERT_REG); // ask for polarity register
	Wire.endTransmission();

	if(Wire.requestFrom(address, (uint8_t)2) != 2) return IOEXP_TYPE_UNKNOWN;
	uint8_t low = Wire.read();
	uint8_t high = Wire.read();
	if(low==0x00 && high==0x00) {
		return IOEXP_TYPE_9555; // PCA9555 has polarity register which inits to 0
	}
	return IOEXP_TYPE_8575;
}

void PCA9555::pinMode(uint8_t pin, uint8_t IOMode) {
	uint16_t config = i2c_read(NXP_CONFIG_REG);
	if(IOMode == OUTPUT) {
		config &= ~(1 << pin); // config bit set to 0 for output pin
	} else {
		config |= (1 << pin);  // config bit set to 1 for input pin
	}
	i2c_write(NXP_CONFIG_REG, config);
}

uint16_t PCA9555::i2c_read(uint8_t reg) {
	if(address==255)	return 0xFFFF;
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.endTransmission();
	if(Wire.requestFrom(address, (uint8_t)2) != 2) {return 0xFFFF; DEBUG_PRINTLN("GPIO error");}
	uint16_t data0 = Wire.read();
	uint16_t data1 = Wire.read();
	return data0+(data1<<8);
}

void PCA9555::i2c_write(uint8_t reg, uint16_t v){
	if(address==255)	return;
	Wire.beginTransmission(address);
	Wire.write(reg);
	Wire.write(v&0xff);
	Wire.write(v>>8);
	Wire.endTransmission();
}

void PCA9555::shift_out(uint8_t plat, uint8_t pclk, uint8_t pdat, uint8_t v) {
	if(plat<IOEXP_PIN || pclk<IOEXP_PIN || pdat<IOEXP_PIN)
		return; // the definition of each pin must be offset by IOEXP_PIN to begin with

	plat-=IOEXP_PIN;
	pclk-=IOEXP_PIN;
	pdat-=IOEXP_PIN;

	uint16_t output = i2c_read(NXP_OUTPUT_REG); // keep a copy of the current output registers

	output &= ~(1<<plat); i2c_write(NXP_OUTPUT_REG, output); // set latch low

	for(uint8_t s=0;s<8;s++) {
		output &= ~(1<<pclk); i2c_write(NXP_OUTPUT_REG, output); // set clock low

		if(v & ((byte)1<<(7-s))) {
			output |= (1<<pdat);
		} else {
			output &= ~(1<<pdat);
		}
		i2c_write(NXP_OUTPUT_REG, output); // set data pin according to bits in v

		output |= (1<<pclk); i2c_write(NXP_OUTPUT_REG, output); // set clock high
	}

	output |= (1<<plat); i2c_write(NXP_OUTPUT_REG, output); // set latch high
}

uint16_t PCF8575::i2c_read(uint8_t reg) {
	if(address==255)	return 0xFFFF;
	Wire.beginTransmission(address);
	if(Wire.requestFrom(address, (uint8_t)2) != 2) return 0xFFFF;
	uint16_t data0 = Wire.read();
	uint16_t data1 = Wire.read();
	Wire.endTransmission();
	return data0+(data1<<8);
}

void PCF8575::i2c_write(uint8_t reg, uint16_t v) {
	if(address==255)	return;
	Wire.beginTransmission(address);
	// todo: handle inputmask (not necessary unless if using any pin as input)
	Wire.write(v&0xff);
	Wire.write(v>>8);
	Wire.endTransmission();
}

uint16_t PCF8574::i2c_read(uint8_t reg) {
	if(address==255)	return 0xFFFF;
	Wire.beginTransmission(address);
	if(Wire.requestFrom(address, (uint8_t)1) != 1) return 0xFFFF;
	uint16_t data = Wire.read();
	Wire.endTransmission();
	return data;
}

void PCF8574::i2c_write(uint8_t reg, uint16_t v) {
	if(address==255)	return;
	Wire.beginTransmission(address);
	Wire.write((uint8_t)(v&0xFF) | inputmask);
	Wire.endTransmission();
}

#include "OpenSprinkler.h"

extern OpenSprinkler os;

void pinModeExt(byte pin, byte mode) {
	if(pin==255) return;
	if(pin>=IOEXP_PIN) {
		os.mainio->pinMode(pin-IOEXP_PIN, mode);
	} else {
		pinMode(pin, mode);
	}
}

void digitalWriteExt(byte pin, byte value) {
	if(pin==255) return;
	if(pin>=IOEXP_PIN) {

		os.mainio->digitalWrite(pin-IOEXP_PIN, value);
	} else {
		digitalWrite(pin, value);
	}
}

byte digitalReadExt(byte pin) {
	if(pin==255) return HIGH;
	if(pin>=IOEXP_PIN) {
		return os.mainio->digitalRead(pin-IOEXP_PIN);
		// a pin on IO expander
		//return pcf_read(MAIN_I2CADDR)&(1<<(pin-IOEXP_PIN));
	} else {
		return digitalRead(pin);
	}
}
#endif

#elif defined(OSPI) || defined(OSBO)

#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>
#include <gpiod.h>

#include "utils.h"

#define BUFFER_MAX 64
#define GPIO_MAX	 64

// GPIO interfaces
const char *gpio_chipname = "gpiochip0";
const char *gpio_consumer = "opensprinkler";

struct gpiod_chip *chip = NULL;
struct gpiod_line* gpio_lines[] = {
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
	NULL, NULL, NULL, NULL, NULL,
};

int assert_gpiod_chip() {
	if( !chip ) {
		chip = gpiod_chip_open_by_name(gpio_chipname);
		if( !chip ) {
			DEBUG_PRINTLN("failed to open gpio chip");
			return -1;
		} else {
			DEBUG_PRINTLN("gpio chip opened");
			return 0;
		}
	}
	return 0;
}

int assert_gpiod_line(int pin) {
	if( !gpio_lines[pin] ) {
		if( assert_gpiod_chip() ) { return -1; }
		gpio_lines[pin] = gpiod_chip_get_line(chip, pin);
		if( !gpio_lines[pin] ) {
			DEBUG_PRINT("failed to open gpio line ");
			DEBUG_PRINTLN(pin);
			return -1;
		} else {
			DEBUG_PRINT("opened gpio line ");
			DEBUG_PRINT(pin);
			return 0;
		}
	}
	return 0;
}

/** Set pin mode, in or out */
void pinMode(int pin, byte mode) {
	if( assert_gpiod_line(pin) ) { return; }
	switch(mode) {
		case INPUT:
			gpiod_line_request_input(gpio_lines[pin], gpio_consumer);
			break;
		case INPUT_PULLUP:
			gpiod_line_request_input_flags(gpio_lines[pin], gpio_consumer, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
			break;
		case OUTPUT:
			gpiod_line_request_output(gpio_lines[pin], gpio_consumer, LOW);
			break;
		default:
			DEBUG_PRINTLN("invalid pin direction");
			break;
	}

	return;
}

/** Read digital value */
byte digitalRead(int pin) {
	if( !gpio_lines[pin] ) {
		DEBUG_PRINT("tried to read uninitialized pin ");
		DEBUG_PRINTLN(pin);
		return 0;
	}
	int val = gpiod_line_get_value(gpio_lines[pin]);
	if( val < 0 ) {
		DEBUG_PRINT("failed to read value on pin ");
		DEBUG_PRINTLN(pin);
		return 0;
	}
	return val;
}

/** Write digital value */
void digitalWrite(int pin, byte value) {
	if( !gpio_lines[pin] ) {
		DEBUG_PRINT("tried to write uninitialized pin ");
		DEBUG_PRINTLN(pin);
		return;
	}

	int res;
	res = gpiod_line_set_value(gpio_lines[pin], value);
	if( res ) {
		DEBUG_PRINT("failed to write value on pin ");
		DEBUG_PRINTLN(pin);
	}
}

void attachInterrupt(int pin, const char* mode, void (*isr)(void)) {}
void gpio_write(int fd, byte value) {}
int gpio_fd_open(int pin, int mode) {return 0;}
void gpio_fd_close(int fd) {}
#else

void pinMode(int pin, byte mode) {}
void digitalWrite(int pin, byte value) {}
byte digitalRead(int pin) {return 0;}
void attachInterrupt(int pin, const char* mode, void (*isr)(void)) {}
int gpio_fd_open(int pin, int mode) {return 0;}
void gpio_fd_close(int fd) {}
void gpio_write(int fd, byte value) {}

#endif
