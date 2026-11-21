CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tickdrift

tickdrift: src/tickdrift.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tickdrift
	./tickdrift

clean:
	rm -f tickdrift
