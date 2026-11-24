CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: streamload

streamload: src/streamload.c
	$(CC) $(CFLAGS) $< -o $@

test: streamload
	./streamload

clean:
	rm -f streamload
