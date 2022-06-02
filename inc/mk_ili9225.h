/**
 * Copyright (C) 2019-2022 by Mahyar Koshkouei <mk@deltabeard.com>
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 * OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#pragma once

//#define NDEBUG
#ifndef MK_ILI9225_READ_AVAILABLE
# define MK_ILI9225_READ_AVAILABLE 0
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define SCREEN_SIZE_X 176u
#define SCREEN_SIZE_Y 220u

typedef enum {
	ILI9225_COLOR_MODE_FULL = 0,
	ILI9225_COLOR_MODE_8COLOR = 1
} ili9225_color_mode_e;

/**
 * Controls the reset pin of the ILI9225.
 * \param state	Set to 0 on low output, else high.
 */
extern void mk_ili9225_set_rst(bool state);

/**
 * Controls state of RS pin.
 * \param state	Set to 0 on low output, else high.
 */
extern void mk_ili9225_set_rs(bool state);

/**
 * Controls state of CS pin.
 * \param state	Set to 0 on low output, else high.
 */
extern void mk_ili9225_set_cs(bool state);

/**
 * Controls state of LED pin.
 * \param state	Set to 0 on low output, else high.
 */
extern void mk_ili9225_set_led(bool state);

/**
 * Sends data to the ILI9225 using SPI. Return only after sending data.
 * \param halfword Data to send.
 */
void mk_ili9225_spi_write16(const uint16_t *halfwords, size_t len);

#if MK_ILI9225_READ_AVAILABLE
/**
 * Reads 16-bit data from the ILI9225 using SPI.
 * \return 16-bit data.
 */
extern uint16_t mk_ili9225_spi_read16(void);
#endif

/**
 * Delays execution in milliseconds.
 * \param ms Duration to sleep for.
 */
extern void mk_ili9225_delay_ms(unsigned ms);

/**
 * Initialise ILI9225 LCD with default settings.
 * \return 0 on success, else error due to invalid LCD identification.
 */
unsigned mk_ili9225_init(void);

#if MK_ILI9225_READ_AVAILABLE
/**
 * Read the current line being driven by the LCD. Can help with tearing
 * mitigation.
 * \return Line driven by LCD.
 */
unsigned mk_ili9225_read_driving_line(void);
#endif

/**
 * Set the window that pixel will be written to. Address will loop within the
 * window.
 *
 * \param hor_start
 * \param hor_end
 * \param vert_start
 * \param vert_end
 */
void mk_ili9225_set_window(uint16_t hor_start, uint16_t hor_end,
	uint16_t vert_start, uint16_t vert_end);

/**
 * Set address pointer in GRAM. Must be within window.
 * \param x
 * \param y
 */
void mk_ili9225_set_address(uint8_t x, uint8_t y);

/**
 * Write pixels to ILI9225 GRAM. These pixels will be displayed on screen at
 * next vsync.
 * \param pixels Pixel data in RGB565 format to write to LCD.
 * \param nmemb Number of pixels.
 */
void mk_ili9225_write_pixels(const uint16_t *pixels, uint_fast16_t nmemb);

/**
 * Inverts the display.
 * @param invert
 */
void mk_ili9225_invert_display(bool invert);

void mk_ili9225_write_pixels_start(void);
void mk_ili9225_write_pixels_end(void);

void mk_ili9225_set_gate_scan(uint16_t hor_start, uint16_t hor_end);

void mk_ili9225_display_control(bool invert, ili9225_color_mode_e colour_mode);

void mk_ili9225_power_control(uint8_t drive_power, bool sleep);

void mk_ili9225_set_drive_freq(uint16_t f);

void mk_ili9225_set_x(uint8_t x);

/**
 * Exit and stop using LCD. Currently does nothing.
 */
void mk_ili9225_exit(void);
