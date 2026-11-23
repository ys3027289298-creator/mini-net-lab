CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I src -DMINILAB_SELFTEST=reconcile_selftest

all: reconcile

reconcile: src/reconcile.c src/reconcile.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/reconcile.c -o $@ -lm

test: reconcile
	./reconcile

clean:
	rm -f reconcile
