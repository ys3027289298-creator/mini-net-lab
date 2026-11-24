CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: resync

resync: src/resync.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: resync
	./resync

clean:
	rm -f resync
