CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: selftest

selftest: src/selftest.c src/timebase.c src/timebase.h
	$(CC) $(CFLAGS) src/selftest.c src/timebase.c -o $@

test: selftest
	./selftest

clean:
	rm -f selftest
