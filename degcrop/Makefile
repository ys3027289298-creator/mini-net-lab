CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: degcrop

degcrop: src/degcrop.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: degcrop
	./degcrop

clean:
	rm -f degcrop
