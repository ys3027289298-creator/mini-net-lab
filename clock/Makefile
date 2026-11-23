CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../mnl

all: clock

clock: src/clock.c ../mnl/mnl.c ../mnl/mnl.h
	$(CC) $(CFLAGS) $(CPPFLAGS) src/clock.c ../mnl/mnl.c -o $@ -lm

test: clock
	./clock

clean:
	rm -f clock
