CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
TB = ../timebase/src
TB_SRC = $(TB)/timebase.c $(TB)/accept.c

all: reconcile

reconcile: src/reconcile.c $(TB_SRC) $(TB)/timebase.h $(TB)/accept.h
	$(CC) $(CFLAGS) -I$(TB) src/reconcile.c $(TB_SRC) -o $@ -lm

test: reconcile
	./reconcile

accept: reconcile
	./reconcile --accept

clean:
	rm -f reconcile
