CC ?= gcc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra
CPPFLAGS += -I../mnl -DMNL_NO_MAIN
MODS = ../snapshot/src/snapshot.c ../clock/src/clock.c \
       ../aoi/src/aoi.c ../reconcile/src/reconcile.c
COMMON = ../mnl/mnl.c

all: mnl_tests mnl_zc_driver mnl_zc_negative libmnl_zc_shim.so

mnl_tests: mnl_tests.c $(MODS) $(COMMON) ../mnl/mnl.h
	$(CC) $(CFLAGS) $(CPPFLAGS) mnl_tests.c $(MODS) $(COMMON) -o $@ -lm

mnl_zc_driver: mnl_zc_driver.c $(MODS) $(COMMON) ../mnl/mnl.h
	$(CC) $(CFLAGS) $(CPPFLAGS) mnl_zc_driver.c $(MODS) $(COMMON) -o $@ -lm -ldl

# -O0 + -fno-builtin-memcpy: the deliberate payload copy must remain a real
# libc call so the versioned interposer can see it.
mnl_zc_negative: mnl_zc_negative.c ../mnl/mnl.h
	$(CC) -O0 -fno-builtin-memcpy -std=c11 -Wall -Wextra $(CPPFLAGS) \
	  mnl_zc_negative.c -o $@ -ldl

libmnl_zc_shim.so: mnl_zc_shim.c mnl_zc_shim.map
	$(CC) $(CFLAGS) -fPIC -shared mnl_zc_shim.c -Wl,--version-script=mnl_zc_shim.map -o $@ -ldl

test: all
	./mnl_tests
	LD_PRELOAD=./libmnl_zc_shim.so ./mnl_zc_driver
	@echo "--- shim negative control (expect SIGABRT / exit 134) ---"
	@set +e; LD_PRELOAD=./libmnl_zc_shim.so ./mnl_zc_negative; \
	  rc=$$?; \
	  if [ $$rc -eq 134 ]; then echo "negative_control: correctly aborted ($$rc)"; \
	  else echo "negative_control: FAIL (rc=$$rc)"; exit 1; fi

clean:
	rm -f mnl_tests mnl_zc_driver mnl_zc_negative libmnl_zc_shim.so
