CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: predict

predict: src/predict.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: predict
	./predict

clean:
	rm -f predict
