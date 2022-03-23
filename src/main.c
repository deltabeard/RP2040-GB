#define ENABLE_LCD 0
#define ENABLE_SOUND 0

/* C Headers */
#include <stdio.h>

/* RP2040 Headers */
#include <hardware/sync.h>
#include <pico/stdio.h>
#include <hardware/timer.h>
#include <hardware/vreg.h>
#include <pico/stdlib.h>

/* Project headers */
#include "hedley.h"
#include "peanut_gb.h"

extern const unsigned char hello_world_gb[];

struct priv_t
{
    /* Pointer to allocated memory holding GB file. */
    const uint8_t *rom;
    /* Pointer to allocated memory holding save file. */
    uint8_t *cart_ram;
};

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	//const struct priv_t * const p = gb->direct.priv;
	return hello_world_gb[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	return 0xFF;
	//const struct priv_t * const p = gb->direct.priv;
	//return p->cart_ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	return;
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
	putchar_raw(tx);
	//printf("0x%02X ", tx);
}

enum gb_serial_rx_ret_e gb_serial_rx(struct gb_s *gb, uint8_t *rx)
{
	(void) gb;
	(void) rx;
	return GB_SERIAL_RX_NO_CONNECTION;
}

int main(void)
{
	static struct gb_s gb;
	enum gb_init_error_e ret;

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

	/* Initialise context. */
	ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, NULL);

	if(ret != GB_INIT_NO_ERROR)
	{
		printf("Error: %d\n", ret);
		goto sleep;
	}

#if ENABLE_LCD
	//gb_init_lcd(&gb, &lcd_draw_line);
	// gb.direct.interlace = 1;
#endif

	gb_init_serial(&gb, gb_serial_tx, gb_serial_rx);

	uint_fast32_t instrs = 0;
	uint64_t start_time = time_us_64();
	uint64_t end_time;
	while(!gb.direct.ended)
	{
		__gb_step_cpu(&gb);
		instrs++;
	}

	end_time = time_us_64();

	puts("\nEmulation Ended");
	printf("Instructions: %u\n"
	       "Time: %llu us\n", instrs, end_time-start_time);

	/* Sleep forever. */
sleep:
	stdio_flush();
	while(1)
		__wfi();

	HEDLEY_UNREACHABLE();
}
