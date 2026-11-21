CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: hyst

hyst: src/hyst.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: hyst
	./hyst

clean:
	rm -f hyst
