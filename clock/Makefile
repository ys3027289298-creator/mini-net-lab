CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: clock

clock: src/clock.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# Unified timing instrumentation (default build above keeps it off).
timing: src/clock.c ../common/ftrace.h ../common/ft_accept.h
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=199309L -DFT_ENABLE=1 $< -o clock_timing -lm

test: clock
	./clock

test-timing: timing
	./clock_timing

clean:
	rm -f clock clock_timing
