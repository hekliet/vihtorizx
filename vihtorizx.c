#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

#include "globals.h"

#include "z80.h"
#include "zxkeys.h"

#define LINES_BEFORE_SCREEN 64
#define LINES_AFTER_SCREEN 56
#define LINES_PER_FRAME (LINES_BEFORE_SCREEN + SCREEN_HEIGHT + LINES_AFTER_SCREEN)
#define CYCLES_PER_LINE 224
#define ZX_CYCLES_PER_FRAME_SHL10 (LINES_PER_FRAME * CYCLES_PER_LINE) << 10

void *presentation_start(void *);
void presentation_blit(uint32_t *);
void presentation_refresh_keys(void);
uint64_t presentation_perf_freq(void);
uint64_t presentation_perf_counter(void);

extern volatile int presentation_sentinel;
extern uint8_t presentation_keys[40];

static z80_t z80;
static uint8_t memory[65536];
static uint8_t *const bitmap_mem = memory + 0x4000;
static uint8_t *const attrib_mem = memory + 0x5800;
static uint8_t border;
static unsigned mic_ear_on;
static unsigned keystates[40];
static unsigned flash_flip_flop, flash_frame_count;

static uint32_t pixels[PIXELS_COUNT];
static uint32_t surface[SURFACE_WIDTH * SURFACE_HEIGHT];

static const uint8_t *autotype_keys;
static unsigned autotype_on;

static volatile sig_atomic_t running = 1;

enum {DISPLAY_STAGE_TOP, DISPLAY_STAGE_MIDDLE, DISPLAY_STAGE_BOTTOM};
static const unsigned disp_stage_cycs[3] = {
    CYCLES_PER_LINE * LINES_BEFORE_SCREEN,
    CYCLES_PER_LINE * SCREEN_HEIGHT,
    CYCLES_PER_LINE * LINES_AFTER_SCREEN
};

static const uint32_t palette[16] = {
    0x000000,
    0x0000D7,
    0xD70000,
    0xD700D7,
    0x00D700,
    0x00D7D7,
    0xD7D700,
    0xD7D7D7,
    0x000000,
    0x0000FF,
    0xFF0000,
    0xFF00FF,
    0x00FF00,
    0x00FFFF,
    0xFFFF00,
    0xFFFFFF
};

static uint8_t readb(uint16_t addr) {
    return memory[addr];
}

static void writeb(uint16_t addr, uint8_t v) {
    if (addr >= 16384) memory[addr] = v;
}

static uint8_t inport(uint16_t port) {
    if (!(port & 1)) {
        uint8_t m = 1, p = port >> 8, b;
        for (b = 0; b < 8; b++) {
            if (!(p & m)) break;
            m <<= 1;
        }
        unsigned key_base = 5 * b;
        uint8_t keybits = 0x1f, bit = 1;
        for (unsigned i = key_base; i < key_base + 5; i++, bit <<= 1) {
            if (presentation_keys[i]) keybits ^= bit;
        }
        return 0xa0 | keybits; // 0 from EAR (bit 6)
    }
    return 0xff;
}

static void outport(uint16_t port, uint8_t v) {
    if (port & 1) {
        fprintf(stderr, "outport to odd port\n");
        return;
    }
    border = v & 7;
    mic_ear_on = (v >> 3) & 3 != 1;
}

static void render_line(unsigned line) {
    int pixels_line = line - LINES_BEFORE_SCREEN + BORDER_HEIGHT;

    if (pixels_line < 0 || pixels_line >= SCREEN_HEIGHT + 2 * BORDER_HEIGHT)
        return;
    
    uint32_t *pp = pixels + pixels_line * PIXELS_WIDTH;
    uint32_t bordercol = palette[border];
    
    if (!(pixels_line >= BORDER_HEIGHT && pixels_line < BORDER_HEIGHT + SCREEN_HEIGHT)) {
        for (unsigned x = 0; x < PIXELS_WIDTH; x++) *pp++ = bordercol;
        return;
    }
    
    unsigned screen_line = line - LINES_BEFORE_SCREEN;
    uint8_t *bp = bitmap_mem + (((screen_line & 0xc0) | ((screen_line & 7) << 3) | ((screen_line >> 3) & 7)) << 5);
    uint8_t *ap = attrib_mem + ((screen_line >> 3) << 5);
    for (unsigned x = 0; x < BORDER_WIDTH; x++) *pp++ = bordercol;
    for (unsigned x = 0; x < SCREEN_WIDTH; x += 8) {
        uint8_t b = *bp++;
        uint8_t a = *ap++;
        uint8_t colbase = (a >> 3) & 0x08;
        uint8_t ink = colbase | (a & 7);
        uint8_t paper = colbase | ((a >> 3) & 7);
        if (a & 0x80 && flash_flip_flop) {
            uint8_t t = ink; ink = paper; paper = t;
        }
        for (unsigned i = 0; i < 8; i++) {
            *pp++ = palette[(b & 0x80) ? ink : paper];
            b <<= 1;
        }
    }
    for (unsigned x = 0; x < BORDER_WIDTH; x++) *pp++ = bordercol;
}

static void autotype(void) {
    uint8_t k = *autotype_keys++;
    if (k == 0xFF) {
        autotype_on = 0;
        return;
    }
    memset(presentation_keys, 0, 40);
    presentation_keys[ZXKEY_CAPS_SHIFT] = !!(k & ZXKEY_MOD_CAPS_SHIFT);
    presentation_keys[ZXKEY_SYM_SHIFT] = !!(k & ZXKEY_MOD_SYM_SHIFT);
    presentation_keys[k & 0x3F] = 1;
}

static void init_z80(void) {
    z80_init(&z80);
    z80.readb = readb;
    z80.writeb = writeb;
    z80.inport = inport;
    z80.outport = outport;
}

static void load_rom(void) {
    FILE *f = fopen("48.rom", "rb");
    fread(memory, 16384, 1, f);
    fclose(f);
}

static void handle_signal(int _) {
    running = 0;
}

static uint64_t time_ns(void) {
    struct timespec ts;
    timespec_get(&ts, TIME_UTC);
    return 1000000000 * ts.tv_sec + ts.tv_nsec;
}

static unsigned step(void) {
    static unsigned cycles = 0;
    static unsigned disp_stage = DISPLAY_STAGE_TOP;
    static unsigned disp_stage_cyc_goal = disp_stage_cycs[DISPLAY_STAGE_TOP];
    static unsigned line_cyc_mod = 0;
    static unsigned line = 0;
    static int vblank_assert_cdown = 0;

    static unsigned frame_count = 0;

    unsigned c = z80_step(&z80);
    cycles += c;
    if (cycles >= disp_stage_cyc_goal) {
        cycles -= disp_stage_cyc_goal;
        disp_stage = (disp_stage + 1) % 3;
        disp_stage_cyc_goal = disp_stage_cycs[disp_stage];
    }

    line_cyc_mod += c;
    if (line_cyc_mod >= CYCLES_PER_LINE) {
        line_cyc_mod -= CYCLES_PER_LINE;
        line = (line + 1) % LINES_PER_FRAME;

        render_line(line);

        if (line == 0) {
            if (++flash_frame_count == 16) {
                flash_frame_count = 0;
                flash_flip_flop ^= 1;
            }

            frame_count++;
            if (autotype_on) {
                if (frame_count >= AUTOTYPE_FRAME_THRESHOLD && autotype_on
                    && !(frame_count % AUTOTYPE_FRAMES_PER_KEY))
                    autotype();
            } else {
                presentation_refresh_keys();
            }
            presentation_blit(pixels);

            z80_assert_maskable_int(&z80, 0xffff);
            vblank_assert_cdown = 28;
        }
    }
    else if (vblank_assert_cdown) {
        vblank_assert_cdown -= c;
        if (vblank_assert_cdown <= 0) {
            vblank_assert_cdown = 0;
            z80_clear_maskable_int(&z80);
        }
    }

    return c;
}

int main(int argc, char *argv[]) {

    autotype_keys = zxprg_border_test;
    autotype_on = 1;

    init_z80();
    load_rom();
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    pthread_t thread_id;
    pthread_create(&thread_id, NULL, presentation_start, NULL);

    uint64_t perf_freq = presentation_perf_freq();
    uint64_t next_frame_incr = (double)perf_freq / ZX_VIDEO_HZ;
    uint64_t next_frame = presentation_perf_counter();
    unsigned cycle_count = 0;
    struct timespec ts = {.tv_sec = 0};

    while (running && !presentation_sentinel) {
        while (cycle_count < ZX_CYCLES_PER_FRAME_SHL10) cycle_count += step() << 10;
        cycle_count -= ZX_CYCLES_PER_FRAME_SHL10;
        next_frame += next_frame_incr;

        // TODO: Flush sample buffer using presentation_queue_audio

        int64_t ahead = next_frame - presentation_perf_counter();
        if (ahead > 0) {
            ts.tv_nsec = ahead * 1000000000LL / perf_freq;
            nanosleep(&ts, NULL);
            while (presentation_perf_counter() < next_frame);
        }
    }

    presentation_sentinel = 1;
    pthread_join(thread_id, NULL);
    return 0;
}
