CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: tuning

tuning: src/tuning.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: tuning
	./tuning

sweep: tuning
	./tuning sweep

clean:
	rm -f tuning tuning_sweep.csv tuning_frontier.svg
