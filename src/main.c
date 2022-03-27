#define ENABLE_LCD 1
#define ENABLE_SOUND 0
#define USE_DMA 0

/**
 * Reducing VSYNC calculation to lower multiple.
 * When setting a clock IRQ to DMG_CLOCK_FREQ_REDUCED, count to
 * SCREEN_REFRESH_CYCLES_REDUCED to obtain the time required each VSYNC.
 * DMG_CLOCK_FREQ_REDUCED = 2^18, and SCREEN_REFRESH_CYCLES_REDUCED = 4389.
 */
#define VSYNC_REDUCTION_FACTOR 16u
#define SCREEN_REFRESH_CYCLES_REDUCED (SCREEN_REFRESH_CYCLES/VSYNC_REDUCTION_FACTOR)
#define DMG_CLOCK_FREQ_REDUCED (DMG_CLOCK_FREQ/VSYNC_REDUCTION_FACTOR)

/* C Headers */
#include <stdio.h>

/* RP2040 Headers */
#include <hardware/sync.h>
#include <pico/stdio.h>
#include <hardware/timer.h>
#include <hardware/vreg.h>
#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <hardware/spi.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>

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

static uint dma_lcd;
extern const unsigned char rom[];
static uint8_t ram[32768];
static int shift = 0;

struct priv_t
{
    /* Pointer to allocated memory holding GB file. */
    const uint8_t *rom;
    /* Pointer to allocated memory holding save file. */
    uint8_t *cart_ram;
};

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
	//const struct priv_t * const p = gb->direct.priv;
	return rom[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	return ram[addr];
	//const struct priv_t * const p = gb->direct.priv;
	//return p->cart_ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	ram[addr] = val;
	//const struct priv_t * const p = gb->direct.priv;
	//p->cart_ram[addr] = val;
}

/**
 * Ignore all errors.
 */
void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t val)
{
#if 0
	const char* gb_err_str[4] =
		{
			"UNKNOWN",
			"INVALID OPCODE",
			"INVALID READ",
			"INVALID WRITE"
		};
	fprintf(stderr, "Error %d occurred: %s\n. Abort.\n",
		gb_err,
		gb_err >= GB_INVALID_MAX ?
		gb_err_str[0] : gb_err_str[gb_err]);
	abort();
#endif
}

void gb_serial_tx(struct gb_s *gb, const uint8_t tx)
{
	(void) gb;
	//putchar_raw(tx);
}

enum gb_serial_rx_ret_e gb_serial_rx(struct gb_s *gb, uint8_t *rx)
{
	(void) gb;
	(void) rx;
	return GB_SERIAL_RX_NO_CONNECTION;
}

void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[160],
		   const uint_fast8_t line)
{
	struct priv_t *priv = gb->direct.priv;
	const uint16_t palette[3][4] = {
		{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
		{ 0x7FFF, 0x5294, 0x294A, 0x0000 },
		{ 0x7FFF, 0x5294, 0x294A, 0x0000 }
	};
	static uint16_t fb[LCD_WIDTH] = { 0 };

	//dma_channel_wait_for_finish_blocking(dma_lcd);
	for(unsigned int x = 0; x < LCD_WIDTH; x++)
	{
		fb[x] = palette[(pixels[x] & LCD_PALETTE_ALL) >> 4][pixels[x] & 3];
	}

	mk_ili9225_set_address(0, 15 + line);
	mk_ili9225_write_pixels(fb, LCD_WIDTH + shift);
	//dma_channel_transfer_from_buffer_now(dma_lcd, &fb[0], LCD_WIDTH);
}

int main(void)
{
	static struct gb_s gb;
	enum gb_init_error_e ret;
	static const uint16_t clear = 0x0000;

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

	stdio_init_all();
	(void)getchar();
	puts("Starting");

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

	puts("Initialising SPI");
	clock_configure(clk_peri, 0,
			CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
			125 * 1000 * 1000, 125 * 1000 * 1000);
	spi_init(spi0, 32*1000*1000);
	printf("SPI baudrate = %u\n", spi_get_baudrate(spi0));
	spi_set_format(spi0, 16,
		       SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

	puts("Initialising LCD");
	mk_ili9225_init();

	dma_lcd = dma_claim_unused_channel(true);
	dma_channel_config c2 = dma_channel_get_default_config(dma_lcd);
	channel_config_set_transfer_data_size(&c2, DMA_SIZE_16);
	channel_config_set_dreq(&c2,DREQ_SPI0_TX); // select SPI TX as Dreq for pacing data transfer
	channel_config_set_read_increment(&c2, false);
	channel_config_set_write_increment(&c2, false);
	channel_config_set_ring(&c2, false, 0);

	/* Clear LCD screen. */
	mk_ili9225_write_pixels_start();
	dma_channel_configure(dma_lcd,
			      &c2,
			      &spi_get_hw(spi0)->dr,
			      &clear,
			      SCREEN_SIZE_X*(SCREEN_SIZE_Y+1),
			      true);
	dma_channel_wait_for_finish_blocking(dma_lcd);
	mk_ili9225_write_pixels_end();

	/* Initialise context. */
	ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, NULL);

	if(ret != GB_INIT_NO_ERROR)
	{
		printf("Error: %d\n", ret);
		goto sleep;
	}

	dma_channel_set_trans_count(dma_lcd, LCD_WIDTH, false);
	channel_config_set_read_increment(&c2, true);

	mk_ili9225_set_window(15, 144+15, 29, 160+29);

#if ENABLE_LCD
	gb_init_lcd(&gb, &lcd_draw_line);
	//gb.direct.interlace = 1;
#endif

	//gb_init_serial(&gb, gb_serial_tx, gb_serial_rx);

#if USE_DMA
	mk_ili9225_write_pixels_start();
#endif

	uint_fast32_t frames = 0;
	uint64_t start_time = time_us_64();
	uint64_t end_time;
	while(1)
	{
		int cmd;

		__gb_step_cpu(&gb);
		__gb_step_cpu(&gb);
		__gb_step_cpu(&gb);
		__gb_step_cpu(&gb);
		__gb_step_cpu(&gb);
		__gb_step_cpu(&gb);
		__gb_step_cpu(&gb);
		__gb_step_cpu(&gb);

		if(gb.gb_frame == 0)
			continue;

		frames++;
		gb.gb_frame = 0;

		cmd = getchar_timeout_us(0);
		if(cmd == PICO_ERROR_TIMEOUT)
			continue;

		switch(cmd)
		{
#if USE_DMA == 0
			static bool invert = false;
			static bool sleep = false;
			static uint8_t freq = 1;
			static ili9225_color_mode_e colour = ILI9225_COLOR_MODE_FULL;

			case 'i':
				invert = !invert;
				mk_ili9225_display_control(invert, colour);
				break;

			case 'c':
				colour = !colour;
				mk_ili9225_display_control(invert, colour);
				break;

			case 'f':
				freq++;
				freq &= 0x0F;
				mk_ili9225_set_drive_freq(freq);
				printf("Freq %u\n", freq);
				break;

			case 'u':
				shift++;
				break;

			case 'd':
				shift--;
				break;
#endif
			case 'q':
				goto out;

			default:
				break;
		}

		//mk_ili9225_write_pixels_start();
	}
out:

	end_time = time_us_64();

	puts("\nEmulation Ended");
	printf("Instructions: %u\n"
	       "Time: %llu us\n", frames, end_time-start_time);

	mk_ili9225_write_pixels_end();
	mk_ili9225_set_rst(true);
	reset_usb_boot(0, 0);

	/* Sleep forever. */
sleep:
	stdio_flush();
	while(1)
		__wfi();

	HEDLEY_UNREACHABLE();
}
