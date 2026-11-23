CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: reconcile

reconcile: src/reconcile.c src/reconcile.h
	$(CC) $(CFLAGS) -DMINILAB_SELFTEST $< -o $@ -lm

test: reconcile
	./reconcile

clean:
	rm -f reconcile
