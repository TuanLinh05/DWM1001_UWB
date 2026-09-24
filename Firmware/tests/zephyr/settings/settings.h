/* Minimal Zephyr settings API used only by the host-side settings test. */
#ifndef TEST_ZEPHYR_SETTINGS_H
#define TEST_ZEPHYR_SETTINGS_H

#include <stddef.h>
#include <sys/types.h>

typedef ssize_t (*settings_read_cb)(void *cb_arg, void *data, size_t len);
typedef int (*settings_test_set_fn)(const char *key, size_t len,
                                    settings_read_cb read_cb, void *cb_arg);

extern settings_test_set_fn settings_test_handler;

int settings_subsys_init(void);
int settings_load_subtree(const char *subtree);
int settings_save_one(const char *name, const void *value, size_t value_len);
int settings_delete(const char *name);

#define SETTINGS_STATIC_HANDLER_DEFINE(_name, _subtree, _get, _set, _commit, _export) \
    settings_test_set_fn settings_test_handler = (_set)

#endif
