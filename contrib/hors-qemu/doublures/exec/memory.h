/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Doublure de exec/memory.h.
 * [2026-09-16] calypso_trx.h l'inclut pour UN SEUL type, et seulement dans le
 * prototype de calypso_trx_init(). Le vrai en-tete entraine tout le
 * sous-systeme memoire de QEMU (memop -> host-utils -> bswap -> rcu ->
 * ramlist) : 112 erreurs sur calypso_bsp.c. Un type opaque suffit. */
#ifndef QOSMO_DOUBLURE_MEMORY_H
#define QOSMO_DOUBLURE_MEMORY_H
typedef struct MemoryRegion MemoryRegion;
typedef struct MemoryRegionOps MemoryRegionOps;
typedef struct AddressSpace AddressSpace;
#endif
