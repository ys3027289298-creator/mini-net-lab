CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aoi

aoi: src/aoi.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: aoi
	./aoi

clean:
	rm -f aoi
