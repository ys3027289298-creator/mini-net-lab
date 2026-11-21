PY ?= python3

test:
	$(PY) pbt.py manual
	$(PY) pbt.py run --seed 1 --cases 50

mutations:
	$(PY) pbt.py mutations

clean:
	rm -f harness
