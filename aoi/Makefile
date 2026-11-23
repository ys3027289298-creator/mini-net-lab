CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
TB = ../timebase/src
TB_SRC = $(TB)/timebase.c $(TB)/accept.c

all: aoi

aoi: src/aoi.c $(TB_SRC) $(TB)/timebase.h $(TB)/accept.h
	$(CC) $(CFLAGS) -I$(TB) src/aoi.c $(TB_SRC) -o $@ -lm

test: aoi
	./aoi

accept: aoi
	./aoi --accept

clean:
	rm -f aoi
