CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: coalesce

coalesce: src/coalesce.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: coalesce
	./coalesce

clean:
	rm -f coalesce
