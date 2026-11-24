CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: reorder

reorder: src/reorder.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: reorder
	./reorder

report: reorder
	./reorder --report

sweep: reorder
	./reorder --sweep

verify: reorder
	./reorder --verify

clean:
	rm -f reorder
