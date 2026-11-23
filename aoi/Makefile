CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../mnl

all: aoi

aoi: src/aoi.c ../mnl/mnl.c ../mnl/mnl.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/aoi.c ../mnl/mnl.c -o $@ -lm

test: aoi
	./aoi

clean:
	rm -f aoi
