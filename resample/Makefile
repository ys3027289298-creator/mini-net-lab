CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: resample

resample: src/resample.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: resample
	./resample

clean:
	rm -f resample
