CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra

all: budget

budget: src/budget.c
	$(CC) $(CFLAGS) $< -o $@ -lm

test: budget
	./budget

report: budget
	./budget curves > out/curves.csv
	./budget cal    > out/cal.csv
	./budget exp    > out/exp.csv
	./budget equal  > out/equal.csv
	python3 tools/build_report.py

clean:
	rm -f budget
	rm -f out/*.csv report.html
