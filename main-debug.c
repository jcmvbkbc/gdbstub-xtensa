#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/ip.h>

void handle_exception(unsigned long *registers);

void breakinst(void)
{
}

void flush_i_cache(void)
{
}

int io;
int dir = -1;

void putDebugChar(char c)
{
	if (dir != 0) {
		dir = 0;
		fputs("\n-> ", stderr);
	}
	putc(c, stderr);
	write(io, &c, 1);
}

int getDebugChar(void)
{
	char c;

	ssize_t rd = read(io, &c, 1);
	if (rd == 0)
		abort();
	if (dir != 1) {
		dir = 1;
		fputs("\n<- ", stderr);
	}
	putc(c, stderr);
	return c;
}

int main(int c, char **v)
{
	int s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	struct sockaddr_in sa = {
		.sin_family = AF_INET,
		.sin_port = htons(2345),
		.sin_addr = INADDR_ANY,
	};
	socklen_t sasz = sizeof(sa);
	unsigned long regs[1024];
	int rc;

	rc = bind(s, (struct sockaddr *)&sa, sizeof(sa));
	if (rc < 0)
		perror("bind");
	rc = listen(s, 5);
	if (rc < 0)
		perror("listen");
	io = accept(s, (struct sockaddr *)&sa, &sasz);

	for (;;)
		handle_exception(regs);
}
