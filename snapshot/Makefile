CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
TB = ../timebase/src
TB_SRC = $(TB)/timebase.c $(TB)/accept.c

all: snapshot

snapshot: src/snapshot.c $(TB_SRC) $(TB)/timebase.h $(TB)/accept.h
	$(CC) $(CFLAGS) -I$(TB) src/snapshot.c $(TB_SRC) -o $@ -lm

test: snapshot
	./snapshot

accept: snapshot
	./snapshot --accept

clean:
	rm -f snapshot
