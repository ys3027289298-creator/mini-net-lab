CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: snapshot

snapshot: src/snapshot.c src/snapshot.h
	$(CC) $(CFLAGS) -DMINILAB_SELFTEST $< -o $@ -lm

test: snapshot
	./snapshot

clean:
	rm -f snapshot
