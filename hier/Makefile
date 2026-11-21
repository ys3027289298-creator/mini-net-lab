CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: hier

hier: src/hier.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: hier
	./hier

clean:
	rm -f hier
