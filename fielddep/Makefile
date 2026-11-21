CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: fielddep

fielddep: src/fielddep.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: fielddep
	./fielddep

clean:
	rm -f fielddep
