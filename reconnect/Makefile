CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: reconnect

reconnect: src/reconnect.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: reconnect
	mkdir -p shots/data
	./reconnect

shots: test
	python3 tools/render_shots.py

clean:
	rm -f reconnect
	rm -rf shots
