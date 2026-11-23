# mini-net-lab top level: per-module self-tests plus the unified suite.
.PHONY: all test clean

all test:
	@ok=1; for d in snapshot clock aoi reconcile; do \
	  $(MAKE) -C $$d test || ok=0; \
	done; \
	$(MAKE) -C tests test || ok=0; \
	test "$$ok" = 1

clean:
	@for d in snapshot clock aoi reconcile; do $(MAKE) -C $$d clean; done
	$(MAKE) -C tests clean
