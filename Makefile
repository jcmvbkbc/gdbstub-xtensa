LD := gcc
CFLAGS += -g
LDFLAGS += -g

XTCC := xt-xcc --xtensa-core=DC_C_233L
XTCFLAGS += -g

xs-debug: main-debug.o xtensa-stub.o
	$(LD) $(LDFLAGS) $^ -o $@

xs-xtensa: main-sim.c gdbstub-entry.S xtensa-stub.c
	$(XTCC) $(XTCFLAGS) $^ -o $@

xs-demo: main.c gdbstub-entry.S gdbstub-fault_handler.S xtensa-stub.c ringbuffer.c
	$(XTCC) $(XTCFLAGS) $^ -o $@

clean:
	rm -f xs-debug main-debug.o xtensa-stub.o

armstub: armstub.o
	$(LD) $(LDFLAGS) $^ -pthread -o $@
