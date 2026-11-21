CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -ffp-contract=off
# -ffp-contract=off: no FMA contraction, so float sums keep their source order.
# Do not add -ffast-math here; replay.c rejects it at compile time.

all: replay

replay: src/replay.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: replay
	./replay

clean:
	rm -f replay

.PHONY: all test clean
