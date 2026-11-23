CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: verify

verify: src/verify.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: verify
	./verify out golden.txt

clean:
	rm -f verify
	rm -rf out out2
