CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: extrap

extrap: src/extrap.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: extrap
	./extrap

clean:
	rm -f extrap
