CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: snapshot

snapshot: src/snapshot.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: snapshot
	./snapshot

clean:
	rm -f snapshot
