CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: reconcile

reconcile: src/reconcile.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# Unified timing instrumentation (default build above keeps it off).
timing: src/reconcile.c ../common/ftrace.h ../common/ft_accept.h
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=199309L -DFT_ENABLE=1 $< -o reconcile_timing -lm

test: reconcile
	./reconcile

test-timing: timing
	./reconcile_timing

clean:
	rm -f reconcile reconcile_timing
