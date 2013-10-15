#include "ringbuffer.h"

#define DEBUG_RX_BASE (0x02000000)
#define DEBUG_TX_BASE (0x02000800)

volatile struct ring * const rx = (void *)DEBUG_RX_BASE;
volatile struct ring * const tx = (void *)DEBUG_TX_BASE;

void init_debug_comm(void)
{
	rx->head = rx->tail = 0;
	tx->head = tx->tail = 0;
}

void poll_debug_ring(void)
{
	if (ring_have_data(rx)) {
		breakpoint();
	}
}

void putDebugChar(char c)
{
	while (!ring_have_space(tx))
		;

	tx->data[tx->head] = c;
	tx->head = ring_next_head(tx);
}

int getDebugChar(void)
{
	int v;
	while (!ring_have_data(rx))
		;

	v = rx->data[rx->tail];
	rx->tail = ring_next_tail(rx);
	return v;
}
