/**
 * Copyright (C) 2022 by Mahyar Koshkouei <mk@deltabeard.com>
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

#define ENABLE_LCD	1

#ifndef ENABLE_SOUND
# define ENABLE_SOUND	0
#endif

/* Use DMA for all drawing to LCD. Benefits aren't fully realised at the moment
 * due to busy loops waiting for DMA completion. */
#define USE_DMA		0

/**
 * Reducing VSYNC calculation to lower multiple.
 * When setting a clock IRQ to DMG_CLOCK_FREQ_REDUCED, count to
 * SCREEN_REFRESH_CYCLES_REDUCED to obtain the time required each VSYNC.
 * DMG_CLOCK_FREQ_REDUCED = 2^18, and SCREEN_REFRESH_CYCLES_REDUCED = 4389.
 * Currently unused.
 */
#define VSYNC_REDUCTION_FACTOR 16u
#define SCREEN_REFRESH_CYCLES_REDUCED (SCREEN_REFRESH_CYCLES/VSYNC_REDUCTION_FACTOR)
#define DMG_CLOCK_FREQ_REDUCED (DMG_CLOCK_FREQ/VSYNC_REDUCTION_FACTOR)

/* C Headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* RP2040 Headers */
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/spi.h>
#include <hardware/sync.h>
#include <hardware/timer.h>
#include <hardware/vreg.h>
#include <pico/bootrom.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <sys/unistd.h>
#include <hardware/irq.h>

/* Project headers */
#include "hedley.h"
#include "peanut_gb.h"
#include "mk_ili9225.h"

/* LCD Connections. */
#define GPIO_CS		1
#define GPIO_CLK	2
#define GPIO_SDA	3
#define GPIO_RS		4
#define GPIO_RST	5

/* DMA channel for LCD communication. */
static uint dma_lcd;
/* Definition of ROM data variable. Must be declared like:
 * #include <pico/platform.h>
 * const unsigned char __in_flash("rom") rom[] = {
 * 	...
 * };
 */
extern const unsigned char rom[];
unsigned char rom_bank0[16384];
static uint8_t ram[32768];
static int lcd_line_busy = 0;

/* Multicore command structure. */
union core_cmd {
    struct {
	/* Does nothing. */
#define CORE_CMD_NOP		0
	/* Set line "data" on the LCD. Pixel data is in pixels_buffer. */
#define CORE_CMD_LCD_LINE	1
	/* Control idle mode on the LCD. Limits colours to 2 bits. */
#define CORE_CMD_IDLE_SET	2
	/* Set a specific pixel. For debugging. */
#define CORE_CMD_SET_PIXEL	3
	uint8_t cmd;
	uint8_t unused1;
	uint8_t unused2;
	uint8_t data;
    };
    uint32_t full;
};

/* Pixel data is stored in here. */
static uint8_t pixels_buffer[LCD_WIDTH];

#define putstdio(x) write(1, x, strlen(x))

/* Functions required for communication with the ILI9225. */
void mk_ili9225_set_rst(bool state)
{
	gpio_put(GPIO_RST, state);
}

void mk_ili9225_set_rs(bool state)
{
	gpio_put(GPIO_RS, state);
}

void mk_ili9225_set_cs(bool state)
{
	gpio_put(GPIO_CS, state);
}

void mk_ili9225_spi_write16(const uint16_t *halfwords, size_t len)
{
	spi_write16_blocking(spi0, halfwords, len);
}

void mk_ili9225_delay_ms(unsigned ms)
{
	sleep_ms(ms);
}

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
	if(addr < sizeof(rom_bank0))
		return rom_bank0[addr];

	return rom[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
	return ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	ram[addr] = val;
}

/**
 * Ignore all errors.
 */
void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t val)
{
#if 1
	const char* gb_err_str[4] = {
			"UNKNOWN",
			"INVALID OPCODE",
			"INVALID READ",
			"INVALID WRITE"
		};
	printf("Error %d occurred: %s\n. Abort.\n",
		gb_err,
		gb_err >= GB_INVALID_MAX ?
		gb_err_str[0] : gb_err_str[gb_err]);
	abort();
#endif
}

void core1_irq_dma_lcd_end_transfer(void)
{
	mk_ili9225_write_pixels_end();
	__atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
	dma_channel_acknowledge_irq0(dma_lcd);
}

void core1_lcd_draw_line(const uint_fast8_t line)
{
	const uint16_t palette[3][4] = {
		{ 0xFFFF, 0x651F, 0x001F, 0x0000 },
		{ 0xFFFF, 0xFC10, 0x89C7, 0x0000 },
		{ 0xFFFF, 0x651F, 0x001F, 0x0000 }
	};
	static uint16_t fb[LCD_WIDTH];

	//dma_channel_wait_for_finish_blocking(dma_lcd);
	for(unsigned int x = 0; x < LCD_WIDTH; x++)
	{
		fb[x] = palette[(pixels_buffer[x] & LCD_PALETTE_ALL) >> 4]
				[pixels_buffer[x] & 3];
	}

	//mk_ili9225_set_address(line + 16, LCD_WIDTH + 30);
	mk_ili9225_set_x(line + 16);

#if USE_DMA
	mk_ili9225_write_pixels_start();
	dma_channel_transfer_from_buffer_now(dma_lcd, &fb[0], LCD_WIDTH);
	do {
		__wfi();
	} while (dma_channel_is_busy(dma_lcd));
	__compiler_memory_barrier();
#else
	mk_ili9225_write_pixels(fb, LCD_WIDTH);
	__atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
#endif
}

_Noreturn
void main_core1(void)
{
	static dma_channel_config c2;
	static const uint16_t red = 0xF800;
	union core_cmd cmd;

	/* Initialise and control LCD on core 1. */
	mk_ili9225_init();

	/* Initilise DMA transfer for clearing the LCD screen. */
	dma_lcd = dma_claim_unused_channel(true);
	c2 = dma_channel_get_default_config(dma_lcd);
	channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);
	channel_config_set_dreq(&c2,DREQ_SPI0_TX);
	channel_config_set_read_increment(&c2, false);
	channel_config_set_write_increment(&c2, false);
	channel_config_set_ring(&c2, false, 0);

	/* Enable IRQ for wake on interrupt functionality. */
	irq_set_exclusive_handler(DMA_IRQ_0, core1_irq_dma_lcd_end_transfer);
	dma_channel_set_irq0_enabled(dma_lcd, true);
	irq_set_enabled(DMA_IRQ_0, true);

	/* Clear LCD screen. */
	mk_ili9225_write_pixels_start();
	dma_channel_configure(dma_lcd, &c2, &spi_get_hw(spi0)->dr, &red,
			      SCREEN_SIZE_X*SCREEN_SIZE_Y+16, true);
	do {
		__wfi();
	} while (dma_channel_is_busy(dma_lcd));
	__compiler_memory_barrier();

	/* Set DMA transfer to be the length of a DMG line. */
	dma_channel_set_trans_count(dma_lcd, LCD_WIDTH, false);
	channel_config_set_read_increment(&c2, true);
	//dma_sniffer_enable(dma_lcd, 0, true);

	/* Set LCD window to DMG size. */
	mk_ili9225_set_window(16, LCD_HEIGHT + 15,
			      31, LCD_WIDTH + 30);
	mk_ili9225_set_address(16, LCD_WIDTH + 30);
	//mk_ili9225_set_x(15);

#if 0
	/* Clear GB Screen window. */
	mk_ili9225_write_pixels_start();
	dma_channel_set_trans_count(dma_lcd, LCD_HEIGHT*LCD_WIDTH+16, false);
	dma_channel_set_read_addr(dma_lcd, &green, true);
	/* TODO: Add sleeping wait. */
	dma_channel_wait_for_finish_blocking(dma_lcd);
	mk_ili9225_write_pixels_end();
#endif

	// Sleep used for debugging LCD window.
	//sleep_ms(1000);

	while(1)
	{
		static uint8_t dot_x, dot_y;
		static bool use_dot = false;

		cmd.full = multicore_fifo_pop_blocking();
		switch(cmd.cmd)
		{
		const uint16_t dot = 0x001F;
		case CORE_CMD_LCD_LINE:
			core1_lcd_draw_line(cmd.data);
			if(use_dot)
				goto set_pixel;
			break;

		case CORE_CMD_IDLE_SET:
			mk_ili9225_display_control(true, cmd.data);
			break;

		case CORE_CMD_SET_PIXEL:
			use_dot = true;
			dot_x = cmd.unused2;
			dot_y = cmd.data;
set_pixel:
			mk_ili9225_set_address(dot_x, dot_y);
			mk_ili9225_write_pixels(&dot, 1);
			break;

		case CORE_CMD_NOP:
		default:
			break;
		}
	}

	HEDLEY_UNREACHABLE();
}

void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
		   const uint_fast8_t line)
{
	union core_cmd cmd;

	/* Wait until previous line is sent. */
	while(__atomic_load_n(&lcd_line_busy, __ATOMIC_SEQ_CST))
		tight_loop_contents();

	/* Populate command. */
	cmd.cmd = CORE_CMD_LCD_LINE;
	cmd.data = line;

	__atomic_store_n(&lcd_line_busy, 1, __ATOMIC_SEQ_CST);
	memcpy(pixels_buffer, pixels, LCD_WIDTH);
	multicore_fifo_push_blocking(cmd.full);
}

int main(void)
{
	static struct gb_s gb;
	enum gb_init_error_e ret;

	/* Overclock. */
	{
		/* The value for VCO set here is meant for least power
		 * consumption. */
		const unsigned vco = 532000000; /* 266MHz/133MHz */
		const unsigned div1 = 2, div2 = 1;

		vreg_set_voltage(VREG_VOLTAGE_1_15);
		sleep_ms(2);
		set_sys_clock_pll(vco, div1, div2);
		sleep_ms(2);
	}

	/* Initialise USB serial connection for debugging. */
	stdio_init_all();
	//(void) getchar();
	putstdio("INIT: ");

	/* Initialise GPIO pins. */
	gpio_set_function(GPIO_CS, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_CLK, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_SDA, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_RS, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_RST, GPIO_FUNC_SIO);

	gpio_set_dir(GPIO_CS, true);
	gpio_set_dir(GPIO_RS, true);
	gpio_set_dir(GPIO_RST, true);
	gpio_set_slew_rate(GPIO_CLK, GPIO_SLEW_RATE_FAST);
	gpio_set_slew_rate(GPIO_SDA, GPIO_SLEW_RATE_FAST);

	/* Set SPI clock to use high frequency. */
	clock_configure(clk_peri, 0,
			CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
			125 * 1000 * 1000, 125 * 1000 * 1000);
	spi_init(spi0, 32*1000*1000);
	spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

	/* Start Core1, which processes requests to the LCD. */
	//puts_raw("Launching Core 1");
	putstdio("CORE1 ");
	multicore_launch_core1(main_core1);

	/* Initialise GB context. */
	memcpy(rom_bank0, rom, sizeof(rom_bank0));
	ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, NULL);
	putstdio("GB ");

	if(ret != GB_INIT_NO_ERROR)
	{
		printf("Error: %d\n", ret);
		goto sleep;
	}

#if ENABLE_LCD
	gb_init_lcd(&gb, &lcd_draw_line);
	putstdio("LCD ");
	//gb.direct.interlace = 1;
#endif
#if ENABLE_SOUND
	audio_init();
	putstdio("AUDIO ");
#endif

	putstdio("\n> ");
	uint_fast32_t frames = 0;
	uint64_t start_time = time_us_64();
	while(1)
	{
		int input;
#if ENABLE_SOUND
		static uint16_t stream[1098];
#endif

		gb.gb_frame = 0;

		do {
			__gb_step_cpu(&gb);
			tight_loop_contents();
		} while(HEDLEY_LIKELY(gb.gb_frame == 0));

		frames++;
#if ENABLE_SOUND
		audio_callback(NULL, stream, 1098);
#endif

		/* Required since we do not know whether a button remains
		 * pressed over a serial connection. */
		gb.direct.joypad = 0xFF;
		input = getchar_timeout_us(0);
		if(input == PICO_ERROR_TIMEOUT)
			continue;

		switch(input)
		{
		static uint8_t dot_x = 50, dot_y = 50;
#if 0
		static bool invert = false;
		static bool sleep = false;
		static uint8_t freq = 1;
		static ili9225_color_mode_e colour = ILI9225_COLOR_MODE_FULL;

		case 'i':
			invert = !invert;
			mk_ili9225_display_control(invert, colour);
			break;

		case 'f':
			freq++;
			freq &= 0x0F;
			mk_ili9225_set_drive_freq(freq);
			printf("Freq %u\n", freq);
			break;
#endif
		case 'c':
		{
			static ili9225_color_mode_e mode = ILI9225_COLOR_MODE_FULL;
			union core_cmd cmd;

			mode = !mode;
			cmd.cmd = CORE_CMD_IDLE_SET;
			cmd.data = mode;
			multicore_fifo_push_blocking(cmd.full);
			break;
		}

		case 'd':
		{
			printf("(x,y): (%d,%d)\n", dot_x, dot_y);
			break;
		}

		case 'J':
		{
			union core_cmd cmd;
			dot_x--;
			cmd.cmd = CORE_CMD_SET_PIXEL;
			cmd.unused2 = dot_x;
			cmd.data = dot_y;
			multicore_fifo_push_blocking(cmd.full);
			break;
		}

		case 'L':
		{
			union core_cmd cmd;
			dot_x++;
			cmd.cmd = CORE_CMD_SET_PIXEL;
			cmd.unused2 = dot_x;
			cmd.data = dot_y;
			multicore_fifo_push_blocking(cmd.full);
			break;
		}

		case 'I':
		{
			union core_cmd cmd;
			dot_y++;
			cmd.cmd = CORE_CMD_SET_PIXEL;
			cmd.unused2 = dot_x;
			cmd.data = dot_y;
			multicore_fifo_push_blocking(cmd.full);
			break;
		}

		case 'K':
		{
			union core_cmd cmd;
			dot_y--;
			cmd.cmd = CORE_CMD_SET_PIXEL;
			cmd.unused2 = dot_x;
			cmd.data = dot_y;
			multicore_fifo_push_blocking(cmd.full);
			break;
		}

		case 'i':
			gb.direct.interlace = !gb.direct.interlace;
			break;

		case 'f':
			gb.direct.frame_skip = !gb.direct.frame_skip;
			break;

		case 'b':
		{
			uint64_t end_time;
			uint32_t diff;
			uint32_t fps;

			end_time = time_us_64();
			diff = end_time-start_time;
			fps = ((uint64_t)frames*1000*1000)/diff;
			printf("Frames: %u\n"
				"Time: %lu us\n"
				"FPS: %lu\n",
				frames, diff, fps);
			stdio_flush();
			frames = 0;
			start_time = time_us_64();
			break;
		}

		case '\n':
		case '\r':
		{
			gb.direct.joypad_bits.start = 0;
			break;
		}

		case '\b':
		{
			gb.direct.joypad_bits.select = 0;
			break;
		}

		case '8':
		{
			gb.direct.joypad_bits.up = 0;
			break;
		}

		case '2':
		{
			gb.direct.joypad_bits.down = 0;
			break;
		}

		case '4':
		{
			gb.direct.joypad_bits.left= 0;
			break;
		}

		case '6':
		{
			gb.direct.joypad_bits.right = 0;
			break;
		}

		case 'z':
		{
			gb.direct.joypad_bits.a = 0;
			break;
		}

		case 'x':
		{
			gb.direct.joypad_bits.b = 0;
			break;
		}

		case 'q':
			goto out;

		default:
			break;
		}
	}
out:

	puts("\nEmulation Ended");

	mk_ili9225_set_rst(true);
	reset_usb_boot(0, 0);

	/* Sleep forever. */
sleep:
	stdio_flush();
	while(1)
		__wfi();

	HEDLEY_UNREACHABLE();
}
