CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: snapshot adaptive

snapshot: src/snapshot.c
	$(CC) $(CFLAGS) $< -o $@ -lm

adaptive: src/adaptive.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: snapshot adaptive
	./snapshot
	./adaptive

clean:
	rm -f snapshot adaptive
