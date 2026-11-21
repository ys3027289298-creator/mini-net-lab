CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: retrans

retrans: src/retrans.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: retrans
	./retrans

clean:
	rm -f retrans
