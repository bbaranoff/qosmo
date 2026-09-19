/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Stand-in for exec/memory.h.
 * [2026-09-16] calypso_trx.h includes it for ONE type, used only in the
 * calypso_trx_init() prototype. The real header drags in the whole QEMU memory
 * subsystem (memop -> host-utils -> bswap -> rcu -> ramlist): 112 errors on
 * calypso_bsp.c. An opaque type is enough. */
#ifndef QOSMO_DOUBLURE_MEMORY_H
#define QOSMO_DOUBLURE_MEMORY_H
typedef struct MemoryRegion MemoryRegion;
typedef struct MemoryRegionOps MemoryRegionOps;
typedef struct AddressSpace AddressSpace;
#endif
