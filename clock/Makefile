CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I src -DMINILAB_SELFTEST=clock_selftest

all: clock

clock: src/clock.c src/clock.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/clock.c -o $@ -lm

test: clock
	./clock

clean:
	rm -f clock
