#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "z80.h"
#include "snapshot.h"

#include "globals.h"

#include <assert.h>

struct __attribute__((packed)) z80header {
    uint8_t a, f;
    uint16_t bc, hl, pc, sp;
    uint8_t i, r;
    uint8_t misc;
    uint16_t de, bc_, de_, hl_;
    uint8_t a_, f_;
    uint16_t iy, ix;
    uint8_t iff1, iff2;
    uint8_t misc_2;
};

static void load_regs(z80_t *z80, struct z80header *h) {
    z80->af.hi = h->a;
    z80->af.lo = h->f;
    z80->bc.w = h->bc;
    z80->hl.w = h->hl;
    z80->pc = h->pc;
    z80->sp.w = h->sp;
    z80->i = h->i;
    z80->r = (h->misc << 7) | (h->r & 0x7f);
    z80->de.w = h->de;
    z80->bc_.w = h->bc_;
    z80->de_.w = h->de_;
    z80->hl_.w = h->hl_;
    z80->af_.hi = h->a_;
    z80->af_.lo = h->f_;
    z80->iy.w = h->iy;
    z80->ix.w = h->ix;
    z80->iff1 = h->iff1;
    z80->iff2 = h->iff2;
    z80->intmode = h->misc_2 & 3;
}

static void load_compressed_data(
    const uint8_t *data, unsigned compressed_len, uint8_t *dest
) {
    unsigned srcpos = 0, destpos = 0;
    const unsigned max_dest = 48 * 1024;

    while (srcpos < compressed_len && destpos < max_dest) {
        if ((compressed_len - srcpos >= 4) && *data == 0xed && data[1] == 0xed) {
            uint8_t count = data[2];
            uint8_t byte  = data[3];

            if (destpos + count > max_dest)
                count = max_dest - destpos;

            for (unsigned i = 0; i < count; i++)
                *dest++ = byte;

            data += 4;
            srcpos += 4;
        } else {
            *dest++ = *data++;
            srcpos++; destpos++;
        }
    }
}

static unsigned load_block(uint8_t *data, uint8_t *memory, unsigned compressed) {
    uint16_t body_len = *(uint16_t *)data;
    if (body_len == 0xffff) body_len = 0x4000;
    uint8_t page = data[2];
    unsigned offset;
    switch (page) {
        case 4: offset = 0x8000; break;
        case 5: offset = 0xc000; break;
        case 8: offset = 0x4000; break;
        default:
            fprintf(stderr, "error loading .z80: unsupported page number (%u)\n", page);
            return 1;
    }
    uint8_t *dest = memory + offset;
    data += 3;
    if (compressed) {
        load_compressed_data(data, body_len, dest);
    } else {
        memcpy(dest, data, body_len);
    }
    return body_len + 3;
}

unsigned load_snapshot(z80_t *z80, const char *path, uint8_t *memory, snapshot_message_t *msg) {
    struct stat sb;
    stat(path, &sb);
    uint8_t buf[sb.st_size];
    FILE *f = fopen(path, "rb");
    fread(buf, sb.st_size, 1, f);
    fclose(f);

    struct z80header *h = (struct z80header *)buf;
    unsigned old_format = !!h->pc;
    unsigned data_offset;
    if (old_format) data_offset = 30;
    else {
        data_offset = 86;
        h->pc = *(uint16_t *)(buf + 32);
    }
    load_regs(z80, h);
    unsigned compressed = !!(h->misc & 32);
    uint8_t *data = buf + data_offset;

    if (old_format) {
        uint8_t *dest = memory + 16384;
        unsigned body_len = sb.st_size - data_offset;
        if (compressed) {
            load_compressed_data(data, body_len, dest);
        }
        else
            memcpy(dest, data, MIN(48 * 1024, body_len));
    } else {
        unsigned pos = data_offset;
        while (pos < sb.st_size) {
            unsigned bytes_consumed = load_block(data, memory, compressed);
            if (!bytes_consumed) {
                return 1;
            }
            data += bytes_consumed;
            pos += bytes_consumed;
        }
    }
    msg->border = (h->misc >> 1) & 7;
    return 0;
}