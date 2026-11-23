CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../mnl

all: reconcile

reconcile: src/reconcile.c ../mnl/mnl.c ../mnl/mnl.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/reconcile.c ../mnl/mnl.c -o $@ -lm

test: reconcile
	./reconcile

clean:
	rm -f reconcile
