CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aoi

aoi: src/aoi.c src/aoi.h
	$(CC) $(CFLAGS) -DMINILAB_SELFTEST $< -o $@ -lm

test: aoi
	./aoi

clean:
	rm -f aoi
