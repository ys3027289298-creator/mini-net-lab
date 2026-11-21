CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: anim

anim: src/anim.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: anim
	./anim

clean:
	rm -f anim
