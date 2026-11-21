CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aoi_sync

aoi_sync: src/aoi_sync.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: aoi_sync
	./aoi_sync

clean:
	rm -f aoi_sync
