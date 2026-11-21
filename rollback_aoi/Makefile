CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: rollback_aoi

rollback_aoi: src/rollback_aoi.c
	$(CC) $(CFLAGS) $< -o $@

test: rollback_aoi
	./rollback_aoi

clean:
	rm -f rollback_aoi
