CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: ftrace_test ftrace_noop_test

ftrace_test: ftrace_test.c ftrace.h
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=199309L -DFT_ENABLE=1 -I. $< -o $@ -lm

ftrace_noop_test: ftrace_noop_test.c ftrace.h
	$(CC) $(CFLAGS) -I. $< -o $@

test: all
	./ftrace_test && ./ftrace_noop_test

clean:
	rm -f ftrace_test ftrace_noop_test
