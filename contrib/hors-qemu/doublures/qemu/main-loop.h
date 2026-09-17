/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef QOSMO_DOUBLURE_MAINLOOP_H
#define QOSMO_DOUBLURE_MAINLOOP_H
typedef void IOHandler(void *opaque);
void qemu_set_fd_handler(int fd, IOHandler *rd, IOHandler *wr, void *opaque);
void qemu_notify_event(void);
#define qemu_mutex_lock_iothread() ((void)0)
#define qemu_mutex_unlock_iothread() ((void)0)
#define bql_lock() ((void)0)
#define bql_unlock() ((void)0)
#endif
