CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: adapt

adapt: src/adapt.c src/adapt_core.c src/adapt_core.h
	$(CC) $(CFLAGS) src/adapt.c src/adapt_core.c -o $@ -lm

sim: src/sim.c src/adapt_core.c src/adapt_core.h
	$(CC) $(CFLAGS) src/sim.c src/adapt_core.c -o sim -lm

test: adapt
	./adapt

runsim: sim
	mkdir -p ../artifacts/adapt
	./sim ../artifacts/adapt

clean:
	rm -f adapt sim
