CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: spectator

spectator: src/spectator.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: spectator
	./spectator

clean:
	rm -f spectator
