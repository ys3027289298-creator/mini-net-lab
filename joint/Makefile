CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: joint

joint: src/joint.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: joint
	./joint

clean:
	rm -f joint
