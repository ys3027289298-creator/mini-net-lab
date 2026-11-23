CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I src -DMINILAB_SELFTEST=aoi_selftest

all: aoi

aoi: src/aoi.c src/aoi.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/aoi.c -o $@ -lm

test: aoi
	./aoi

clean:
	rm -f aoi
