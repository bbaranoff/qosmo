#ifndef QOSMO_DOUBLURE_LOG_H
#define QOSMO_DOUBLURE_LOG_H
#include <stdio.h>
#define qemu_log(...)      fprintf(stderr, __VA_ARGS__)
#define qemu_log_mask(m, ...) do { (void)(m); fprintf(stderr, __VA_ARGS__); } while (0)
#define LOG_UNIMP   0
#define LOG_GUEST_ERROR 0
#endif
