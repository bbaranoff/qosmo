# Plugin TCG tbring (voir tbring.c). Se compile hors de meson, contre l'en-tete
# public include/qemu/qemu-plugin.h de cet arbre :
#     make -C tools/qosmo-tbring            -> libtbring.so a cote
#     make -C tools/qosmo-tbring install    -> /usr/local/lib/qosmo/libtbring.so
QOSMO  ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
PREFIX ?= /usr/local
CFLAGS ?= -O2 -g -Wall
all: libtbring.so
libtbring.so: tbring.c
	$(CC) $(CFLAGS) -shared -fPIC -I$(QOSMO)/include/qemu $(shell pkg-config --cflags glib-2.0) -o $@ $< $(shell pkg-config --libs glib-2.0)
install: libtbring.so tbring-dump.py
	install -d $(PREFIX)/lib/qosmo $(PREFIX)/bin
	install -m 644 libtbring.so $(PREFIX)/lib/qosmo/
	install -m 755 tbring-dump.py $(PREFIX)/bin/tbring-dump
clean:
	rm -f libtbring.so
.PHONY: all install clean
