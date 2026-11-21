CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: inqueue

inqueue: src/inqueue.c
	$(CC) $(CFLAGS) $< -o $@

test: inqueue
	./inqueue

clean:
	rm -f inqueue
