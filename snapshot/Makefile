CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I src -DMINILAB_SELFTEST=snapshot_selftest

all: snapshot

snapshot: src/snapshot.c src/snapshot.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/snapshot.c -o $@ -lm

test: snapshot
	./snapshot

clean:
	rm -f snapshot
