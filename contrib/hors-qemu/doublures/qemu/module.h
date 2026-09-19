#ifndef QOSMO_DOUBLURE_MODULE_H
#define QOSMO_DOUBLURE_MODULE_H
/* Turn type_init() into a plain ELF constructor: same effect, without QOM. */
#define type_init(fn) \
    static void __attribute__((constructor)) qosmo_ctor_##fn(void) { fn(); }
#endif
