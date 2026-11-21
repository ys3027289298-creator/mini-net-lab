CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fade

fade: src/fade.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: fade
	./fade

clean:
	rm -f fade
