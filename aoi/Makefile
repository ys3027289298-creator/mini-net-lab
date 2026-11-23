CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aoi

aoi: src/aoi.c
	$(CC) $(CFLAGS) $< -o $@ -lm

# Unified timing instrumentation (default build above keeps it off).
timing: src/aoi.c ../common/ftrace.h ../common/ft_accept.h
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=199309L -DFT_ENABLE=1 $< -o aoi_timing -lm

test: aoi
	./aoi

test-timing: timing
	./aoi_timing

clean:
	rm -f aoi aoi_timing
