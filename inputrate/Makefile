CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: inputrate

inputrate: src/inputrate.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: inputrate
	./inputrate

clean:
	rm -f inputrate
