#ifndef __Z80_H
#define __Z80_H

#include <stdint.h>

typedef union {uint16_t w; struct {uint8_t lo; uint8_t hi;};} reg16_t;

typedef struct {
    uint16_t pc, memptr;
    uint8_t i, r, iff1, iff2;
    unsigned intmode, halted;

    reg16_t sp;
    reg16_t af; reg16_t bc; reg16_t de; reg16_t hl;
    reg16_t ix; reg16_t iy;

    reg16_t af_; reg16_t bc_; reg16_t de_; reg16_t hl_;

    uint8_t (*readb)(uint16_t addr);
    void (*writeb)(uint16_t addr, uint8_t v);
    uint8_t (*inport)(uint16_t port);
    void (*outport)(uint16_t port, uint8_t v);

    uint8_t prefix, swap_hl;
    int8_t displace;
    unsigned set_memptr_to_index_plus_d;

    uint8_t prev_swap_hl;
    reg16_t *p_hl_subst;
    reg16_t *regpair[4], *regpair2[4];

    unsigned cycles;

    unsigned nmi_requested;
    unsigned maskable_int_requested;
    uint16_t maskable_int_data;
    unsigned op_was_ei;
} z80_t;

void z80_init(z80_t *z80);
unsigned z80_step(z80_t *z80);
void z80_trigger_nmi(z80_t *z80);
void z80_assert_maskable_int(z80_t *z80, uint16_t bus_data);
void z80_clear_maskable_int(z80_t *z80);

#endif
