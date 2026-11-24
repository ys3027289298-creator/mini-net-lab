CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: dualtime

dualtime: src/dualtime.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: dualtime
	./dualtime

clean:
	rm -f dualtime
