CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: entropy

entropy: src/entropy.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: entropy
	./entropy

clean:
	rm -f entropy
