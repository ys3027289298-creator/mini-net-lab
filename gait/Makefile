CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: gait

gait: src/gait.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: gait
	./gait

clean:
	rm -f gait
