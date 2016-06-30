


default: help


help:
	@echo
	@if test -d dist; then \
		echo "To install ARNL, use 'make install'."; \
		else echo "To see and build example programs, enter the 'examples' directory."; \
	fi
	@echo
	@echo "ARNL cannot be (re)compiled since most of its source code ('src' dir) "
	@echo "is proprietary and not distributed."

.PHONY: install install-base install-arnl install-sonarnl install-mogs all help examples default info


-include Makefile.dev


examples/%:
	$(MAKE) -C examples $*

examples/%Static:
	$(MAKE) -C examples $*Static

examples:
	$(MAKE) -C examples




