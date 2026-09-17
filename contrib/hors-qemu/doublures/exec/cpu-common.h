#ifndef QOSMO_DOUBLURE_CPU_COMMON_H
#define QOSMO_DOUBLURE_CPU_COMMON_H
#include <stdbool.h>
#include <stdint.h>
/* La memoire « invitee » est fournie par le harnais (voir cales-qemu.c). */
void cpu_physical_memory_rw(uint64_t addr, void *buf, uint64_t len, bool is_write);
#define cpu_physical_memory_read(a, b, l)  cpu_physical_memory_rw(a, b, l, false)
#define cpu_physical_memory_write(a, b, l) cpu_physical_memory_rw(a, b, l, true)
#endif
