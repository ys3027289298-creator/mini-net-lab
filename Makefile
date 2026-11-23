# run every module's self-test
SUBDIRS := snapshot clock aoi reconcile degcrop

test:
	@for d in $(SUBDIRS); do \
		echo "== $$d =="; $(MAKE) -C $$d test || exit 1; \
	done

clean:
	@for d in $(SUBDIRS); do $(MAKE) -C $$d clean; done

.PHONY: test clean
