/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Doublure de qemu/bswap.h : les seuls acces utilises par la L1 gr-gsm.
 * [2026-09-16] Le vrai en-tete entraine host-utils puis rcu et ramlist. */
#ifndef QOSMO_DOUBLURE_BSWAP_H
#define QOSMO_DOUBLURE_BSWAP_H
#include <stdint.h>
#include <string.h>
static inline uint16_t lduw_be_p(const void *p)
{ uint16_t v; memcpy(&v, p, 2); return __builtin_bswap16(v); }
static inline uint32_t ldl_be_p(const void *p)
{ uint32_t v; memcpy(&v, p, 4); return __builtin_bswap32(v); }
static inline void stw_be_p(void *p, uint16_t v)
{ v = __builtin_bswap16(v); memcpy(p, &v, 2); }
static inline void stl_be_p(void *p, uint32_t v)
{ v = __builtin_bswap32(v); memcpy(p, &v, 4); }
static inline uint16_t lduw_le_p(const void *p)
{ uint16_t v; memcpy(&v, p, 2); return v; }
static inline uint32_t ldl_le_p(const void *p)
{ uint32_t v; memcpy(&v, p, 4); return v; }
static inline void stw_le_p(void *p, uint16_t v) { memcpy(p, &v, 2); }
static inline void stl_le_p(void *p, uint32_t v) { memcpy(p, &v, 4); }
#define be16_to_cpu(x) __builtin_bswap16(x)
#define be32_to_cpu(x) __builtin_bswap32(x)
#define cpu_to_be16(x) __builtin_bswap16(x)
#define cpu_to_be32(x) __builtin_bswap32(x)
/* Little-endian natif sur x86/ARM : conversions neutres. */
#define le16_to_cpu(x) ((uint16_t)(x))
#define le32_to_cpu(x) ((uint32_t)(x))
#define le64_to_cpu(x) ((uint64_t)(x))
#define cpu_to_le16(x) ((uint16_t)(x))
#define cpu_to_le32(x) ((uint32_t)(x))
#define cpu_to_le64(x) ((uint64_t)(x))
#endif
