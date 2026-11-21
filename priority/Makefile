CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: priority

priority: src/priority.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: priority
	./priority

clean:
	rm -f priority
