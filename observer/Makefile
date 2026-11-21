CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: observer

observer: src/observer.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: observer
	./observer

clean:
	rm -f observer
