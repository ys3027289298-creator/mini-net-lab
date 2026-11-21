CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fec

fec: src/fec.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: fec
	./fec

clean:
	rm -f fec
