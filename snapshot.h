#include <stdint.h>

#include "z80.h"

typedef struct snapshot_message_t {
    uint8_t border;
} snapshot_message_t;

unsigned load_snapshot(z80_t *z80, const char *path, uint8_t *memory, snapshot_message_t *msg);
