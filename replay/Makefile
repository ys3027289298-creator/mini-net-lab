CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
PY ?= python3

all: replay

replay: src/replay.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: replay
	./replay

csv: replay
	./replay csv

verify:
	cd /tmp && $(PY) -I $(CURDIR)/materials/verify.py

clean:
	rm -f replay

.PHONY: all test csv verify clean
