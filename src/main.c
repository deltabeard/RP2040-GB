/* C Headers */
#include <stdio.h>

/* RP2040 Headers */
#include <hardware/sync.h>

/* Project headers */
#include "hedley.h"

int main(void)
{
	puts("Hello World!");

	/* Sleep forever. */
	while(1)
		__wfi();

	HEDLEY_UNREACHABLE();
}
