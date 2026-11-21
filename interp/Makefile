CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: interp

interp: src/interp.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: interp
	./interp

clean:
	rm -f interp
