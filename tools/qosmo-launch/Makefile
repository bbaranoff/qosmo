# tools/qosmo-launch — lanceur C de qemu-system-arm pour CE fork.
#   make            -> ./qosmo-dsp
#   make install    -> /usr/local/bin/qosmo-dsp
# La source qosmo-launch.c est identique dans qosmo-grgsm/tools/qosmo-launch ;
# seules ces trois variables changent.
ALIAS   ?= qosmo-grgsm
TREE    ?= $(abspath $(CURDIR)/../..)
DSP     ?= 0
PREFIX  ?= /usr/local
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=gnu11
VERSION := $(shell git -C $(TREE) rev-parse --short HEAD 2>/dev/null || echo local)

all: $(ALIAS)

$(ALIAS): qosmo-launch.c Makefile
	$(CC) $(CFLAGS) -DQOSMO_ALIAS='"$(ALIAS)"' -DQOSMO_TREE='"$(TREE)"' \
	      -DQOSMO_DSP=$(DSP) -DQOSMO_VERSION='"$(VERSION)"' -o $@ $<

install: $(ALIAS)
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 0755 $(ALIAS) $(DESTDIR)$(PREFIX)/bin/$(ALIAS)

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/$(ALIAS)

clean:
	rm -f $(ALIAS)

.PHONY: all install uninstall clean
