CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: snapshot

snapshot: src/snapshot.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# Unified timing instrumentation (default build above keeps it off).
timing: src/snapshot.c ../common/ftrace.h ../common/ft_accept.h
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=199309L -DFT_ENABLE=1 $< -o snapshot_timing -lm

test: snapshot
	./snapshot

test-timing: timing
	./snapshot_timing

clean:
	rm -f snapshot snapshot_timing
