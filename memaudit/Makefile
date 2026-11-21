CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: memaudit

memaudit: src/memaudit.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: memaudit
	./memaudit

clean:
	rm -f memaudit
