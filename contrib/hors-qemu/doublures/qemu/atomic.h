#ifndef QOSMO_DOUBLURE_ATOMIC_H
#define QOSMO_DOUBLURE_ATOMIC_H
#define qatomic_read(p)      __atomic_load_n(p, __ATOMIC_RELAXED)
#define qatomic_set(p, v)    __atomic_store_n(p, v, __ATOMIC_RELAXED)
#define qatomic_inc(p)       __atomic_fetch_add(p, 1, __ATOMIC_RELAXED)
#define smp_mb()             __atomic_thread_fence(__ATOMIC_SEQ_CST)
#endif
