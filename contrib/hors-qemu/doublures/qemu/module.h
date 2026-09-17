#ifndef QOSMO_DOUBLURE_MODULE_H
#define QOSMO_DOUBLURE_MODULE_H
/* type_init() devient un constructeur ELF ordinaire : meme effet, sans QOM. */
#define type_init(fn) \
    static void __attribute__((constructor)) qosmo_ctor_##fn(void) { fn(); }
#endif
