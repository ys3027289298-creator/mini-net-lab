CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: scenetree

scenetree: src/scenetree.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: scenetree
	./scenetree

bench: scenetree
	./scenetree bench

clean:
	rm -f scenetree
