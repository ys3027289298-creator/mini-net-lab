CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: interest

interest: src/interest.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: interest
	./interest

clean:
	rm -f interest
