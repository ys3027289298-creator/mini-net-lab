CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: aoi_tier

aoi_tier: src/aoi_tier.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: aoi_tier
	./aoi_tier

clean:
	rm -f aoi_tier
