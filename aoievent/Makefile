CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aoievent

aoievent: src/aoievent.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: aoievent
	./aoievent

clean:
	rm -f aoievent
