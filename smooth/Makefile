CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: smooth

smooth: src/smooth.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: smooth
	./smooth

clean:
	rm -f smooth
