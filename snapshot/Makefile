CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../mnl

all: snapshot

snapshot: src/snapshot.c ../mnl/mnl.c ../mnl/mnl.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/snapshot.c ../mnl/mnl.c -o $@ -lm

test: snapshot
	./snapshot

clean:
	rm -f snapshot
