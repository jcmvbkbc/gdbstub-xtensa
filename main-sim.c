#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

void init_gdbstub(void);
int in, out;

void flush_i_cache(void)
{
}

void init_debug_comm(void)
{
	in = open("in", O_RDONLY);
	out = open("out", O_WRONLY);
}

void putDebugChar(char c)
{
	if (write(out, &c, 1) != 1)
		abort();
}

int getDebugChar(void)
{
	char c;
	if (read(in, &c, 1) != 1)
		abort();
	return c;
}

int main()
{
	init_gdbstub();
	breakpoint();
	printf("test\n");
	return 0;
}
