CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: clock

clock: src/clock.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: clock
	./clock

clean:
	rm -f clock
