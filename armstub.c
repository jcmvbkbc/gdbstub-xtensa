#include <fcntl.h>
#include <getopt.h>
#include <netinet/ip.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "ringbuffer.h"

#define PAGE_SIZE (0x1000)
#define PAGE_MASK (~(PAGE_SIZE - 1))

struct serve_data {
	int rxfd;
	int txfd;
	volatile struct ring *rx;
	volatile struct ring *tx;
	volatile int done;
};

static void print_usage(void)
{
	printf("  -p PORT  --port=PORT\n"
	       "  -r RX-RING-PHYS-ADDR  --rx=RX-RING-PHYS-ADDR\n"
	       "  -t TX-RING-PHYS-ADDR  --tx=TX-RING-PHYS-ADDR\n"
	       "  -s  --stdio\n"
	       "  -h  --help\n");
}

static void *map_phys_mem(unsigned long addr)
{
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	void *map;

	if (fd < 0) {
		perror("open(/dev/mem)");
		abort();
	}
	map = mmap(NULL, sizeof(struct ring) + (addr & ~PAGE_MASK),
		   PROT_READ | PROT_WRITE,
		   MAP_SHARED, fd, addr & PAGE_MASK);
	if (map == MAP_FAILED) {
		perror("mmap");
		abort();
	} else {
		map += addr & ~PAGE_MASK;
	}
	close(fd);
	return map;
}

static void *serve_rx(void *p)
{
	struct serve_data *data = p;
	int rd;
	unsigned char b;

	while ((rd = read(data->rxfd, &b, 1)) == 1) {
		while (!data->done && !ring_have_space(data->rx)) {
			struct timespec ts = {
				.tv_nsec = 1000000,
			};
			nanosleep(&ts, NULL);
		}
		if (data->done)
			break;
		data->rx->data[data->rx->head] = b;
		data->rx->head = ring_next_head(data->rx);
	}
	data->done = 1;
	return NULL;
}

static void *serve_tx(void *p)
{
	struct serve_data *data = p;
	int wr;
	unsigned char b;

	while (!data->done) {
		if (!ring_have_data(data->tx)) {
			struct timespec ts = {
				.tv_nsec = 1000000,
			};
			nanosleep(&ts, NULL);
		}
		while (ring_have_data(data->tx)) {
			wr = write(data->txfd,
				   (void *)(data->tx->data + data->tx->tail),
				   1);
			if (wr != 1)
				break;
			data->tx->tail = ring_next_tail(data->tx);
		}
	}
	data->done = 1;
	return NULL;
}

int main(int argc, char **argv)
{
	struct option opts[] = {
		{"port", required_argument, NULL, 'p'},
		{"rx", required_argument, NULL, 'r'},
		{"tx", required_argument, NULL, 't'},
		{"stdio", no_argument, NULL, 's'},
		{"help", no_argument, NULL, 'h'},
		{0}
	};
	struct sockaddr_in sa = {
		.sin_family = AF_INET,
		.sin_port = htons(2345),
		.sin_addr = INADDR_ANY,
	};
	struct ring *rx = NULL;
	struct ring *tx = NULL;
	int s;
	int io;
	int rc;
	int opt, idx;
	int stdio = 0;

	while((opt = getopt_long(argc, argv, "p:r:t:sh", opts, &idx)) != -1) {
		switch(opt) {
		case 'p':
			sa.sin_port = htons(strtoul(optarg, NULL, 0));
			break;

		case 'r':
			rx = map_phys_mem(strtoul(optarg, NULL, 0));
			break;

		case 't':
			tx = map_phys_mem(strtoul(optarg, NULL, 0));
			break;

		case 's':
			stdio = 1;
			break;

		case 'h':
			print_usage();
			return 0;
		}
	}

	if (!rx || !tx) {
		printf("rx and tx rings location must be specified\n");
		abort();
	}
	if (!stdio) {
		s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (s < 0) {
			perror("socket");
			abort();
		}
		rc = bind(s, (struct sockaddr *)&sa, sizeof(sa));
		if (rc < 0) {
			perror("bind");
			abort();
		}
		rc = listen(s, 5);
		if (rc < 0) {
			perror("listen");
			abort();
		}
	}
	for (;;) {
		socklen_t sasz = sizeof(sa);
		struct serve_data data = {
			.rxfd = STDIN_FILENO,
			.txfd = STDOUT_FILENO,
			.rx = rx,
			.tx = tx,
		};
		pthread_t rx_thread, tx_thread;
		void *rv;

		if (!stdio) {
			io = accept(s, (struct sockaddr *)&sa, &sasz);
			if (io < 0) {
				perror("accept");
				abort();
			}
			data.rxfd = data.txfd = io;
		}
		rc = pthread_create(&rx_thread, NULL, serve_rx, &data);
		if (rc) {
			perror("pthread_create(&rx_thread,...)");
			abort();
		}
		rc = pthread_create(&tx_thread, NULL, serve_tx, &data);
		if (rc) {
			perror("pthread_create(&tx_thread,...)");
			abort();
		}
		pthread_join(rx_thread, &rv);
		pthread_join(tx_thread, &rv);
		if (!stdio) {
			close(io);
		}
	}
}
