CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
TB = ../timebase/src
TB_SRC = $(TB)/timebase.c $(TB)/accept.c

all: clock

clock: src/clock.c $(TB_SRC) $(TB)/timebase.h $(TB)/accept.h
	$(CC) $(CFLAGS) -I$(TB) src/clock.c $(TB_SRC) -o $@ -lm

test: clock
	./clock

accept: clock
	./clock --accept

clean:
	rm -f clock
