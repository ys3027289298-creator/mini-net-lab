CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: guard

guard: src/guard.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: guard
	./guard

clean:
	rm -f guard
