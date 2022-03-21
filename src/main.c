#define ENABLE_LCD 0
#define ENABLE_SOUND 0

/* C Headers */
#include <stdio.h>

/* RP2040 Headers */
#include <hardware/sync.h>

/* Project headers */
#include "hedley.h"
#include "peanut_gb.h"

static unsigned char rom[32768] = { 0 };

struct priv_t
{
    /* Pointer to allocated memory holding GB file. */
    uint8_t *rom;
    /* Pointer to allocated memory holding save file. */
    uint8_t *cart_ram;
};

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t gb_rom_read(struct gb_s *gb, const uint_fast32_t addr)
{
	const struct priv_t * const p = gb->direct.priv;
	return p->rom[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	const struct priv_t * const p = gb->direct.priv;
	return p->cart_ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	const struct priv_t * const p = gb->direct.priv;
	p->cart_ram[addr] = val;
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
	printf("0x%02X ", tx);
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
	static struct priv_t priv = { .rom = rom, .cart_ram = NULL };
	enum gb_init_error_e ret;

	puts("Hello World!");

	/* Initialise context. */
	ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, &priv);

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

	while(1)
		gb_run_frame(&gb);

	/* Sleep forever. */
sleep:
	while(1)
		__wfi();

	HEDLEY_UNREACHABLE();
}
