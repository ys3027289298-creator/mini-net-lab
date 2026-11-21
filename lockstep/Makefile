CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: lockstep

lockstep: src/lockstep.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: lockstep
	./lockstep

clean:
	rm -f lockstep
