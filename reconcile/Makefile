CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: reconcile

reconcile: src/reconcile.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: reconcile
	./reconcile

clean:
	rm -f reconcile
