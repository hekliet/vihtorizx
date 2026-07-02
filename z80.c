#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "z80.h"

#define MASK_CF 0x01
#define MASK_NF 0x02
#define MASK_PF 0x04
#define MASK_XF 0x08
#define MASK_HF 0x10
#define MASK_YF 0x20
#define MASK_ZF 0x40
#define MASK_SF 0x80

enum {BC, DE};
enum {CF, NF, PF, XF, HF, YF, ZF, SF};

static uint8_t flag_precalc[256]; // SZY0XP00

static inline uint8_t has_even_parity(uint8_t v) {
  v ^= v >> 4;
  v &= 0xf;
  return 1 ^ ((0x6996 >> v) & 1);
}

static void precalc_flags(void) {
    for (unsigned v = 0; v < 256; v++) {
        uint8_t f = v & 0xa8; // S0Y0X000
        if (v == 0) f |= 64;
        f |= has_even_parity(v) << 2;
        flag_precalc[v] = f;
    }
}

void z80_init(z80_t *z80) {
    z80->sp.w = z80->af.w  = 0xffff;
    z80->pc   = z80->bc.w  = z80->de.w  = z80->hl.w
              = z80->bc_.w = z80->de_.w = z80->hl_.w
              = z80->ix.w  = z80->iy.w  = z80->memptr = 0;
    z80->i    = z80->r     = z80->iff1  = z80->iff2 = 0;
    z80->intmode = z80->halted  = 0;
    z80->prefix  = z80->swap_hl = z80->displace = 0;
    z80->set_memptr_to_index_plus_d = 0;

    z80->prev_swap_hl = 0;
    z80->p_hl_subst   = &z80->hl;
    z80->regpair[0]   = z80->regpair2[0] = &z80->bc;
    z80->regpair[1]   = z80->regpair2[1] = &z80->de;
    z80->regpair[2]   = z80->regpair2[2] = &z80->hl;
    z80->regpair[3]   = &z80->sp;
    z80->regpair2[3]  = &z80->af;

    z80->cycles = 0;

    z80->nmi_requested = 0;
    z80->maskable_int_requested = 0;
    z80->maskable_int_data = 0;
    z80->op_was_ei = 0;

    if (!flag_precalc[0]) precalc_flags();
}

static uint16_t readw(z80_t *z80, uint16_t addr) {
    uint8_t lo = z80->readb(addr);
    return (z80->readb(++addr) << 8) | lo;
}

static void writew(z80_t *z80, uint16_t addr, uint16_t w) {
    z80->writeb(addr, w);
    z80->writeb(addr + 1, w >> 8);
}

static uint8_t nextb(z80_t *z80) {
    return z80->readb(z80->pc++);
}

static uint16_t nextw(z80_t *z80) {
    uint16_t w = readw(z80, z80->pc);
    z80->pc += 2;
    return w;
}

uint8_t hl_subst_indirect(z80_t *z80) {
    switch (z80->swap_hl) {
        case 0:    return z80->readb(z80->hl.w);
        case 0xdd: return z80->readb(z80->ix.w + z80->displace);
        case 0xfd: return z80->readb(z80->iy.w + z80->displace);
    }
}

void set_hl_subst_indirect(z80_t *z80, uint8_t v) {
    switch (z80->swap_hl) {
        case 0:    z80->writeb(z80->hl.w, v); break;
        case 0xdd: z80->writeb(z80->ix.w + z80->displace, v); break;
        case 0xfd: z80->writeb(z80->iy.w + z80->displace, v);
    }
}

uint8_t regpair_indirect(z80_t *z80, unsigned rp) {
    if (rp == BC) return z80->readb(z80->bc.w);
    return z80->readb(z80->de.w);
}

void set_regpair_indirect(z80_t *z80, unsigned rp, uint8_t v) {
    if (rp == BC) { z80->writeb(z80->bc.w, v); return; }
    z80->writeb(z80->de.w, v);
}

uint8_t reg(z80_t *z80, uint8_t yz) {
    switch (yz) {
        case 0: return z80->bc.hi;
        case 1: return z80->bc.lo;
        case 2: return z80->de.hi;
        case 3: return z80->de.lo;
        case 4: return z80->p_hl_subst->hi;
        case 5: return z80->p_hl_subst->lo;
        case 6: return hl_subst_indirect(z80);
        case 7: return z80->af.hi;
    }
}

void set_reg(z80_t *z80, uint8_t yz, uint8_t v) {
    switch (yz) {
        case 0: z80->bc.hi = v; break;
        case 1: z80->bc.lo = v; break;
        case 2: z80->de.hi = v; break;
        case 3: z80->de.lo = v; break;
        case 4: z80->p_hl_subst->hi = v; break;
        case 5: z80->p_hl_subst->lo = v; break;
        case 6: set_hl_subst_indirect(z80, v); break;
        case 7: z80->af.hi = v;
    }
}

uint8_t reg_unsub(z80_t *z80, uint8_t yz) {
    switch (yz) {
        case 0: return z80->bc.hi;
        case 1: return z80->bc.lo;
        case 2: return z80->de.hi;
        case 3: return z80->de.lo;
        case 4: return z80->hl.hi;
        case 5: return z80->hl.lo;
        case 6: return z80->readb(z80->hl.w);
        case 7: return z80->af.hi;
    }
}


void set_reg_unsub(z80_t *z80, uint8_t yz, uint8_t v) {
    switch (yz) {
        case 0: z80->bc.hi = v; break;
        case 1: z80->bc.lo = v; break;
        case 2: z80->de.hi = v; break;
        case 3: z80->de.lo = v; break;
        case 4: z80->hl.hi = v; break;
        case 5: z80->hl.lo = v; break;
        case 6: z80->writeb(z80->hl.w, v); break;
        case 7: z80->af.hi = v;
    }
}

uint8_t overflow_add(uint8_t left, uint8_t right, uint8_t result) {
    uint8_t sa = left >> 7;
    return (1 ^ sa ^ (right >> 7)) & ((result >> 7) ^ sa);
}

uint8_t overflow_sub(uint8_t left, uint8_t right, uint8_t result) {
    uint8_t sa = left >> 7;
    return (sa ^ (right >> 7)) & ((result >> 7) ^ sa);
}

uint8_t halfcarry(uint8_t before, uint8_t arg, uint8_t after) {
    return ((before ^ arg ^ after) >> 4) & 1;
}

static void adc(z80_t *z80, uint8_t p) {
    reg16_t *hl = &z80->hl;
    reg16_t *right = z80->regpair[p];
    uint16_t hl_old = hl->w;
    uint16_t right_old = right->w;
    uint8_t fc_old = z80->af.lo & MASK_CF;
    uint16_t res_lo = hl->lo + (right_old & 255) + fc_old;
    uint8_t _carry = res_lo >> 8;
    res_lo &= 255;
    uint16_t res_hi = hl->hi + (right_old >> 8) + _carry;
    uint8_t f = (flag_precalc[res_hi & 255] & 0xa8)                   // S0Y0X000
              | (halfcarry(hl->hi, (right_old >> 8), res_hi) << HF)   // S0YHX000
              | (res_hi >> 8);                                        // S0YHX00C
    res_hi &= 255;
    hl->lo = res_lo;
    hl->hi = res_hi;
    if (hl->w == 0) f |= 64; // SZYHX00C
    uint32_t full = (uint32_t)hl_old + right_old + fc_old;
    uint8_t overflow = ((~(hl_old ^ right_old) & (hl_old ^ (uint16_t)full)) >> 15) & 1;
    f |= overflow << PF;
    z80->af.lo = f;
    z80->memptr = hl_old + 1;
    z80->cycles = 15;
}

static void add(z80_t *z80, uint8_t p) {
    reg16_t *left = z80->p_hl_subst;
    reg16_t *right = z80->regpair[p];
    uint16_t res_lo = left->lo + right->lo;
    uint8_t _carry = res_lo >> 8;
    res_lo &= 255;
    uint16_t res_hi_nocarry = left->hi + right->hi;
    uint16_t res_hi = res_hi_nocarry + _carry;
    uint8_t f = flag_precalc[res_hi & 255] | (halfcarry(left->hi, right->hi, res_hi) << HF) | (res_hi >> 8); // SZYHXP0C
    res_hi &= 255;
    z80->memptr = left->w + 1;
    left->lo = res_lo;
    left->hi = res_hi;
    z80->af.lo = (z80->af.lo & 0xc4) // SZ...P..
               | (f & 0x39);         // ..YHX.0C
    z80->cycles = z80->swap_hl ? 15 : 11;
}

static void bit(z80_t *z80, uint8_t y, uint8_t z) {
    uint8_t v, _53src;
    if (z80->swap_hl) {
        // with DD or FD, read from (ix/y + d) regardless of z
        uint16_t addr = z80->memptr = z80->p_hl_subst->w + z80->displace;
        v = z80->readb(addr);
        _53src = addr >> 8;
        z80->cycles = 20;
    }
    else {
        v = reg(z80, z);
        if (z == 6) {
            _53src = z80->memptr >> 8;
            z80->cycles = 12;
        } else {
            _53src = v;
            z80->cycles = 8;
        }
    }
    uint8_t result = v & (1 << y);
    uint8_t f = flag_precalc[result] & 0xc4
              | (z80->af.lo & MASK_CF)
              | (1 << HF)
              | (((_53src >> YF) & 1) << YF)
              | (((_53src >> XF) & 1) << XF);
    z80->af.lo = f;
}

static unsigned check_condition(z80_t *z80, uint8_t ci) {
    switch (ci) {
        case 0: return ((z80->af.lo >> ZF) & 1) ^ 1;
        case 1: return  (z80->af.lo >> ZF) & 1;
        case 2: return ((z80->af.lo >> CF) & 1) ^ 1;
        case 3: return  (z80->af.lo >> CF) & 1;
        case 4: return ((z80->af.lo >> PF) & 1) ^ 1;
        case 5: return  (z80->af.lo >> PF) & 1;
        case 6: return ((z80->af.lo >> SF) & 1) ^ 1;
        case 7: return  (z80->af.lo >> SF) & 1;
    }
}

static void call(z80_t *z80, uint8_t y, uint8_t z) {
    uint16_t nn = z80->memptr = nextw(z80);
    unsigned do_branch = z == 4 ? check_condition(z80, y) : 1;
    if (do_branch) {
        z80->sp.w -= 2;
        writew(z80, z80->sp.w, z80->pc);
        z80->pc = nn;
        z80->cycles = 17;
    } else {
        z80->cycles = 10;
    }
}

static void ccf(z80_t *z80) {
    uint8_t f =  (flag_precalc[z80->af.hi] & 0x28) // 00Y0X000
              |  (z80->af.lo & MASK_CF) << HF      // 00YHX000
              | ((z80->af.lo & MASK_CF) ^ 1);      // 00YHX00C
    z80->af.lo = (z80->af.lo & 0xc4)               // SZ000P00
               | f;                                // SZYHXP0C
    z80->cycles = 4;
}

static void cpl(z80_t *z80) {
    z80->af.hi = ~z80->af.hi;
    uint8_t f = (flag_precalc[z80->af.hi] & 0x28) // 00Y0X000
              | 0x12;                             // 00YHX0N0
    z80->af.lo = (z80->af.lo & 0xc5)              // SZ000P0C
               | f;                               // SZYHXPNC
    z80->cycles = 4;
}

static void daa(z80_t *z80) {
    uint8_t correction = 0;

    if ((z80->af.hi & 15) > 9 || (z80->af.lo & MASK_HF))
        correction += 0x06;

    uint8_t f = z80->af.lo & MASK_NF; // 000000N0
    if (z80->af.hi > 0x99 || (z80->af.lo & MASK_CF)) {
        correction += 0x60;
        f |= 1; // 000000NC
    }

    if (z80->af.lo & MASK_NF) {
        f |= ((z80->af.lo & MASK_HF) && (z80->af.hi & 15) < 0x06) << HF; // 000H00NC
        z80->af.hi -= correction;
    } else {
        f |= ((z80->af.hi & 15) > 9) << HF; // 000H00NC
        z80->af.hi += correction;
    }

    z80->af.lo = f | flag_precalc[z80->af.hi];
    z80->cycles = 4;
}

static void dec(z80_t *z80, uint8_t y, uint8_t z, uint8_t p) {
    if (z == 3) {
        z80->regpair[p]->w--;
        if (p == 2 && z80->swap_hl) z80->cycles = 10;
        else z80->cycles = 6;
    }
    else {
        uint8_t u = reg(z80, y);
        uint8_t v = u - 1;
        set_reg(z80, y, v);
        z80->af.lo = flag_precalc[v] & 0xfb         // SZY0X000
                   | (z80->af.lo & MASK_CF)         // SZY0X00C
                   | 2                              // SZY0X0NC
                   | (halfcarry(u, 1, v) << HF)     // SZYHX0NC
                   | (overflow_sub(u, 1, v) << PF); // SZYHXPNC
        if (y == 6) {
            if (z80->swap_hl) {
                z80->set_memptr_to_index_plus_d = 1;
                z80->cycles = 23;
            }
            else z80->cycles = 11;
        }
        else z80->cycles = 4;
    }
}

static void di(z80_t *z80) {
    z80->iff1 = z80->iff2 = 0;
    z80->cycles = 4;
    z80->op_was_ei = 1;
}

static void djnz(z80_t *z80) {
    if (--z80->bc.hi) {
        uint16_t addr = z80-> pc + z80->displace;
        z80->pc = z80->memptr = addr;
        z80->cycles = 13;
    }
    z80->cycles = 8;
}

static void ei(z80_t *z80) {
    z80->iff1 = z80->iff2 = 1;
    z80->cycles = 4;
    z80->op_was_ei = 1;
}

static void ex(z80_t *z80, uint8_t x, uint8_t y) {
    if (x == 0) { // ex af, af'
        uint16_t t = z80->af.w;
        z80->af.w = z80->af_.w;
        z80->af_.w = t;
    } else if (y == 4) { // ex (sp), hl
        uint16_t t = readw(z80, z80->sp.w);
        writew(z80, z80->sp.w, z80->p_hl_subst->w);
        z80->p_hl_subst->w = z80->memptr = t;
    } else { // ex de, hl
        uint16_t t = z80->hl.w;
        z80->hl.w = z80->de.w;
        z80->de.w = t;
    }
    z80->cycles = 4;
}

static void exx(z80_t *z80) {
    uint16_t t;
    t = z80->bc.w; z80->bc.w = z80->bc_.w; z80->bc_.w = t;
    t = z80->de.w; z80->de.w = z80->de_.w; z80->de_.w = t;
    t = z80->hl.w; z80->hl.w = z80->hl_.w; z80->hl_.w = t;
    z80->cycles = 4;
}

static void halt(z80_t *z80) {
    z80->halted = 1;
    z80->cycles = 4;
}

static void im(z80_t *z80, uint8_t y) {
    switch (y) {
        case 0: z80->intmode = 0; break;
        case 2: z80->intmode = 1; break;
        case 3: z80->intmode = 2;
    }
    z80->cycles = 8;
}

static void in(z80_t *z80, uint8_t x, uint8_t y) {
    unsigned pending_flags = 1;
    uint8_t v;
    if (x == 3) { // in a, (n)
        uint16_t port = (z80->af.hi << 8) | nextb(z80);
        z80->memptr = port + 1;
        z80->af.hi = z80->inport(port);
        pending_flags = 0;
    } else if (y == 6) // in (c)
        v = z80->inport(z80->bc.w);
   else {
        v = z80->inport(z80->bc.w);
        if (y == 7)
            z80->memptr = z80->bc.w + 1;
        set_reg(z80, y, v);
    }
    if (pending_flags)
        z80->af.lo = flag_precalc[v] | (z80->af.lo & MASK_CF);
    z80->cycles = x == 3 ? 11 : 12;
}

static void inc(z80_t *z80, uint8_t y, uint8_t z, uint8_t p) {
    if (z == 3) {
        z80->regpair[p]->w++;
        if (p == 2 && z80->swap_hl) z80->cycles = 10;
        else z80->cycles = 6;
    }
    else {
        uint8_t u = reg(z80, y);
        uint8_t v = u + 1;
        set_reg(z80, y, v);
        z80->af.lo = flag_precalc[v] & 0xfb         // SZY0X000
                   | (z80->af.lo & MASK_CF)         // SZY0X00C
                   | (halfcarry(u, 1, v) << HF)     // SZYHX00C
                   | (overflow_add(u, 1, v) << PF); // SZYHXP0C
        if (y == 6) {
            if (z80->swap_hl) {
                z80->set_memptr_to_index_plus_d = 1;
                z80->cycles = 23;
            }
            else z80->cycles = 11;
        }
        else z80->cycles = 4;
    }
}

static void jp(z80_t *z80, uint8_t y, uint8_t z) {
    uint16_t addr;
    unsigned cond = 1;
    switch (z) {
        case 1: // jp (hl)
            // NOTE: memptr_eng.txt says "except JP rp", but JGZ80 does set memptr here
            // addr = z80->memptr = z80->p_hl_subst->w; break;
            addr = z80->p_hl_subst->w;
            z80->cycles = z80->swap_hl ? 8 : 4;
            break;
        case 2: // jp cc, nn
            addr = z80->memptr = nextw(z80); cond = check_condition(z80, y);
            z80->cycles = 10;
            break;
        case 3: // jp nn
            addr = z80->memptr = nextw(z80);
            z80->cycles = 10;
    }
    if (cond) z80->pc = addr;
}

static void jr(z80_t *z80, uint8_t y) {
    unsigned cond = y == 3 ? 1 : check_condition(z80, y & 3);
    if (cond) {
        uint16_t addr = z80->pc + z80->displace;
        z80->pc = z80->memptr = addr;
        z80->cycles = 12;
    }
    else z80->cycles = 7;
}

static void ld(z80_t *z80, uint8_t x, uint8_t y, uint8_t z, uint8_t p, uint8_t q) {
    if (z80->prefix) {
        if (z == 3) {
            uint16_t addr = nextw(z80);
            if (q) // ld rp, (nn)
                z80->regpair[p]->w = readw(z80, addr);
            else // ld (nn), rp
                writew(z80, addr, z80->regpair[p]->w);
            z80->memptr = addr + 1;
            z80->cycles = 20;
        } else {
            switch (y) {
                case 0: z80->i = z80->af.hi; z80->cycles = 9; break; // ld i, a
                case 1: z80->r = z80->af.hi; z80->cycles = 9; break; // ld r, a
                case 2: // ld a, i
                    z80->af.hi = z80->i;
                    z80->af.lo = (flag_precalc[z80->af.hi] & 0xfb) // SZY0X000
                               | (z80->iff2 << PF)                 // SZY0XP00
                               | (z80->af.lo & MASK_CF);           // SZY0XP0C
                    z80->cycles = 9;
                    break;
                case 3: // ld a, r
                    z80->af.hi = z80->r;
                    z80->af.lo = (flag_precalc[z80->af.hi] & 0xfb) // SZY0X000
                               | (z80->iff2 << PF)                 // SZY0XP00
                               | (z80->af.lo & MASK_CF);           // SZY0XP0C
                    z80->cycles = 9;
            }
        }
    } else {
        switch (x) {
            case 0:
                switch (z) {
                    case 1: z80->regpair[p]->w = nextw(z80); z80->cycles = 10; break; // ld rp, nn
                    case 2:
                        if (q) {
                            switch (p) {
                                case 0: z80->af.hi = regpair_indirect(z80, BC); z80->memptr = z80->bc.w + 1; z80->cycles = 7; break; // ld a, (bc)
                                case 1: z80->af.hi = regpair_indirect(z80, DE); z80->memptr = z80->de.w + 1; z80->cycles = 7; break; // ld a, (de)
                                case 2: // ld hl, (nn)
                                    uint16_t addr = nextw(z80);
                                    z80->p_hl_subst->w = readw(z80, addr);
                                    z80->memptr = addr + 1;
                                    z80->cycles = 16;
                                    break;
                                case 3: // ld a, (nn)
                                    addr = nextw(z80);
                                    z80->af.hi = z80->readb(addr);
                                    z80->memptr = addr + 1;
                                    z80->cycles = 13;
                            }
                        } else {
                            switch (p) {
                                case 0: set_regpair_indirect(z80, BC, z80->af.hi); z80->memptr = (z80->af.hi << 8) | ((z80->bc.w + 1) & 255); z80->cycles = 7; break; // ld (bc), a
                                case 1: set_regpair_indirect(z80, DE, z80->af.hi); z80->memptr = (z80->af.hi << 8) | ((z80->de.w + 1) & 255); z80->cycles = 7; break; // ld (de), a
                                case 2: writew(z80, nextw(z80), z80->p_hl_subst->w); z80->cycles = 16; break; // ld (nn), hl
                                case 3: // ld (nn), a
                                    uint16_t addr = nextw(z80);
                                    z80->writeb(addr, z80->af.hi);
                                    z80->memptr = (z80->af.hi << 8) | ((addr + 1) & 255);
                                    z80->cycles = 13;
                            }
                        }
                        break;
                    case 6: // ld r, n
                        set_reg(z80, y, nextb(z80));
                        if (z80->swap_hl && y == 6) z80->set_memptr_to_index_plus_d = 1;
                        z80->cycles = 7;
                }
                break;
            case 1:
                if (z80->swap_hl && y == 6) { // ld r, r
                    set_reg(z80, y, reg_unsub(z80, z));
                    z80->set_memptr_to_index_plus_d = 1;
                }
                else if (z80->swap_hl && z == 6) { // ld r, r
                    set_reg_unsub(z80, y, reg(z80, z));
                    z80->set_memptr_to_index_plus_d = 1;
                }
                else // ld r, r
                    set_reg(z80, y, reg(z80, z));
                z80->cycles = 4;
                break;
            case 3: // ld sp, hl
                z80->sp.w = z80->p_hl_subst->w;
                z80->cycles = 6;
        }
    }
}

static void neg(z80_t *z80) {
    uint8_t a_old = z80->af.hi;
    z80->af.hi = -z80->af.hi;
    z80->af.lo = flag_precalc[z80->af.hi] & 0xf8             // SZY0X000
               | 2                                           // SZY0X0N0
               | (z80->af.hi ? 1 : 0)                        // SZY0X0NC
               | (halfcarry(0, a_old, z80->af.hi) << HF)     // SZYHX0NC
               | (overflow_sub(0, a_old, z80->af.hi) << PF); // SZYHXPNC
    z80->cycles = 8;
}

static void nop(z80_t *z80) { z80->cycles = 4; }

static void out(z80_t *z80, uint8_t x, uint8_t y) {
    if (x == 3) { // out (n), a
        uint8_t port_lo = nextb(z80);
        uint16_t port = (z80->af.hi << 8) | port_lo;
        z80->outport(port, z80->af.hi);
        z80->memptr = (port & 0xff00) | ((port_lo + 1) & 255);
        z80->cycles = 11;
    }
    else if (y == 6) { // out (c), 0
        z80->outport(z80->bc.w, 0);
        z80->cycles = 12;
    }
    else { // out (c), r
        z80->outport(z80->bc.w, reg(z80, y));
        if (y == 7) z80->memptr = z80->bc.w + 1;
        z80->cycles = 12;
    }
}

static void pop(z80_t *z80, uint8_t p) {
    z80->regpair2[p]->w = readw(z80, z80->sp.w);
    z80->sp.w += 2;
    z80->cycles = (z80->swap_hl && p == 2) ? 14 : 10;
}

static void push(z80_t *z80, uint8_t p) {
    z80->sp.w -= 2;
    writew(z80, z80->sp.w, z80->regpair2[p]->w);
    z80->cycles = (z80->swap_hl && p == 2) ? 15 : 11;
}

static void res(z80_t *z80, uint8_t y, uint8_t z) {
    if (z80->swap_hl && z != 6) { // res y, (ix/iy + d), r
        uint8_t v = hl_subst_indirect(z80) & ~(1 << y);
        set_hl_subst_indirect(z80, v);
        set_reg_unsub(z80, z, v);
        z80->set_memptr_to_index_plus_d = 1;
        z80->cycles = 23;
    } else { // res y, r
        set_reg(z80, z, reg(z80, z) & ~(1 << y));
        z80->cycles = z == 6 ? 15 : 8;
    }
    if (z80->swap_hl) z80->set_memptr_to_index_plus_d = 1;
}

static void ret(z80_t *z80, uint8_t y, uint8_t z) {
    if (z || check_condition(z80, y)) {
        z80->pc = z80->memptr = readw(z80, z80->sp.w);
        z80->sp.w += 2;
        z80->cycles = z ? 10 : 11;
    }
    else z80->cycles = 5;
}

static void retn(z80_t *z80) {
    z80->iff1 = z80->iff2;
    z80->pc = z80->memptr = readw(z80, z80->sp.w);
    z80->sp.w += 2;
    z80->cycles = 14;
}

static void rla(z80_t *z80) {
    uint8_t carry = z80->af.hi >> 7;
    z80->af.hi    = ((z80->af.hi << 1) & 255) | (z80->af.lo & MASK_CF);
    uint8_t f     = (flag_precalc[z80->af.hi] | carry)
                  & 0x29;                // 00Y0X00C
    z80->af.lo    = (z80->af.lo & 0xc4)  // SZ000P00
                  | f;                   // SZY0XP0C
    z80->cycles = 4;
}

static void rlca(z80_t *z80) {
    uint8_t carry = z80->af.hi >> 7;
    z80->af.hi    = ((z80->af.hi << 1) & 255) | carry;
    uint8_t f     = (flag_precalc[z80->af.hi] | carry)
                  & 0x29;                // 00Y0X00C
    z80->af.lo    = (z80->af.lo & 0xc4)  // SZ000P00
                  | f;                   // SZY0XP0C
    z80->cycles = 4;
}

static void rra(z80_t *z80) {
    uint8_t carry = z80->af.hi & 1;
    z80->af.hi    = ((z80->af.lo & MASK_CF) << 7) | (z80->af.hi >> 1);
    uint8_t f     = (flag_precalc[z80->af.hi] | carry)
                  & 0x29;                // 00Y0X00C
    z80->af.lo    = (z80->af.lo & 0xc4)  // SZ000P00
                  | f;                   // SZY0XP0C
    z80->cycles = 4;
}

static void rrca(z80_t *z80) {
    uint8_t carry = z80->af.hi & 1;
    z80->af.hi    = (carry << 7) | (z80->af.hi >> 1);
    uint8_t f     = (flag_precalc[z80->af.hi] | carry)
                  & 0x29;                // 00Y0X00C
    z80->af.lo    = (z80->af.lo & 0xc4)  // SZ000P00
                  | f;                   // SZY0XP0C
    z80->cycles = 4;
}

static void rld(z80_t *z80) {
    uint8_t w = z80->readb(z80->hl.w);
    uint8_t w_hi = w >> 4, w_lo = w & 15;
    uint8_t a_lo = z80->af.hi & 15;
    z80->writeb(z80->hl.w, (w_lo << 4) | a_lo);
    z80->af.hi = (z80->af.hi & 0xf0) | w_hi;
    z80->af.lo = (z80->af.lo & MASK_CF) | flag_precalc[z80->af.hi];
    z80->memptr = z80->hl.w + 1;
    z80->cycles = 18;
}

static void rrd(z80_t *z80) {
    uint8_t w = z80->readb(z80->hl.w);
    uint8_t w_hi = w >> 4, w_lo = w & 15;
    uint8_t a_lo = z80->af.hi & 15;
    z80->writeb(z80->hl.w, (a_lo << 4) | w_hi);
    z80->af.hi = (z80->af.hi & 0xf0) | w_lo;
    z80->af.lo = (z80->af.lo & MASK_CF) | flag_precalc[z80->af.hi];
    z80->memptr = z80->hl.w + 1;
    z80->cycles = 18;
}

static void rst(z80_t *z80, uint8_t y) {
    z80->sp.w -= 2;
    writew(z80, z80->sp.w, z80->pc);
    z80->pc = z80->memptr = y << 3;
    z80->cycles = 11;
}

static void sbc(z80_t *z80, uint8_t p) {
    reg16_t *hl = &z80->hl;
    reg16_t *right = z80->regpair[p];
    uint16_t hl_old = hl->w;
    uint16_t right_old = right->w;
    uint8_t fc_old = z80->af.lo & MASK_CF;
    uint16_t res_lo = hl->lo - (right_old & 255) - fc_old;
    uint8_t _carry = (res_lo >> 8) & 1;
    res_lo &= 255;
    uint16_t res_hi = hl->hi - (right_old >> 8) - _carry;
    uint8_t f = (flag_precalc[res_hi & 255] & 0xa8)                   // S0Y0X000
              | (halfcarry(hl->hi, (right_old >> 8), res_hi) << HF)   // S0YHX000
              | 2                                                     // S0YHX0N0
              | ((res_hi >> 8) & 1);                                  // S0YHX0NC
    res_hi &= 255;
    hl->lo = res_lo;
    hl->hi = res_hi;
    if (hl->w == 0) f |= 64; // SZYHX0NC
    uint32_t full = (uint32_t)hl_old - right_old - fc_old;
    uint8_t overflow = ((hl_old ^ right_old) & (hl_old ^ (uint16_t)full)) >> 15 & 1;
    f |= overflow << PF;
    z80->af.lo = f;
    z80->memptr = hl_old + 1;
    z80->cycles = 15;
}

static void scf(z80_t *z80) {
    uint8_t f = flag_precalc[z80->af.hi]
              & 0x28                 // 00Y0X000
              | 1;                   // 00Y0X00C
    z80->af.lo = (z80->af.lo & 0xc4) // SZ000P00
               | f;                  // SZY0XP0C
    z80->cycles = 4;
}

static void set(z80_t *z80, uint8_t y, uint8_t z) {
    if (z80->swap_hl && z != 6) { // set y, (ix/iy + d), r
        uint8_t v = hl_subst_indirect(z80) | (1 << y);
        set_hl_subst_indirect(z80, v);
        set_reg_unsub(z80, z, v);
        z80->cycles = 23;
    } else { // set y, r
        set_reg(z80, z, reg(z80, z) | (1 << y));
        z80->cycles = z == 6 ? 15 : 8;
    }
    if (z80->swap_hl) z80->set_memptr_to_index_plus_d = 1;
}

static void alu(z80_t *z80, uint8_t x, uint8_t y, uint8_t z) {
    uint8_t arg;
    if (x == 2) {
        arg = reg(z80, z);
        if (z == 6) {
            if (z80->swap_hl) {
                z80->set_memptr_to_index_plus_d = 1;
                z80->cycles = 19;
            }
            z80->cycles = 7;
        }
        else z80->cycles = 4;
    } else {
        arg = nextb(z80);
        z80->cycles = 7;
    }
    switch (y) {
        case 0: case 1: // add_a; adc_a
            uint8_t a_old = z80->af.hi;
            uint16_t result = z80->af.hi + arg + ((y == 1 || y == 3) ? (z80->af.lo & MASK_CF) : 0);
            uint8_t carry = result >> 8;
            z80->af.hi = result;
            z80->af.lo = flag_precalc[z80->af.hi] & ~MASK_PF          // SZY0X000
                       | (halfcarry(a_old, arg, z80->af.hi) << HF)    // SZYHX000
                       | (overflow_add(a_old, arg, z80->af.hi) << PF) // SZYHXP00
                       | carry;                                       // SZYHXP0C
            break;
        case 2: case 3: // sub; sbc_a
            a_old = z80->af.hi;
            result = z80->af.hi - arg - ((y == 1 || y == 3) ? (z80->af.lo & MASK_CF) : 0);
            carry = (result >> 8) & 1;
            z80->af.hi = result;
            z80->af.lo = flag_precalc[z80->af.hi] & ~MASK_PF          // SZY0X000
                       | (halfcarry(a_old, arg, z80->af.hi) << HF)    // SZYHX000
                       | (overflow_sub(a_old, arg, z80->af.hi) << PF) // SZYHXP00
                       | 2                                            // SZYHXPN0
                       | carry;                                       // SZYHXPNC
            break;
        case 4: // and
            z80->af.hi &= arg;
            z80->af.lo = flag_precalc[z80->af.hi] | 16;
            break;
        case 5: // xor
            z80->af.hi ^= arg;
            z80->af.lo = flag_precalc[z80->af.hi];
            break;
        case 6: // or
            z80->af.hi |= arg;
            z80->af.lo = flag_precalc[z80->af.hi];
            break;
        case 7: // cp
            uint16_t v = z80->af.hi - arg;
            carry = (v >> 8) & 1;
            v &= 255;
            z80->af.lo =  flag_precalc[v]   & 0xc0                // SZ000000
                       | (flag_precalc[arg] & 0x28)               // SZY0X000
                       | (halfcarry(z80->af.hi, arg, v) << HF)    // SZYHX000
                       | (overflow_sub(z80->af.hi, arg, v) << PF) // SZYHXP00
                       | 2                                        // SZYHXPN0
                       | carry;                                   // SZYHXPNC
    }
}

static void bli(z80_t *z80, uint8_t y, uint8_t z) {
    uint8_t v = y & 3;
    int8_t incr = y & 1 ? -1 : 1;
    unsigned is_repe = v > 1, repe;
    switch (z) {
        case 0: // ldi(r); ldd(r)
            uint8_t t = z80->readb(z80->hl.w);
            z80->writeb(z80->de.w, t);
            z80->de.w += incr;
            z80->hl.w += incr;
            z80->bc.w--;
            uint8_t n = t + z80->af.hi;
            uint8_t f = (n << 4) & MASK_YF
                      | (n & MASK_XF)
                      | ((z80->bc.w != 0) << PF); // 00Y0XP00
            z80->af.lo = (z80->af.lo & 0xc1)      // SZ00000C
                       | f;                       // SZY0XP0C
            if (is_repe && z80->bc.w != 1) z80->memptr = z80->pc - 1; // instr_addr + 1
            repe = z80->bc.w != 0;
            break;
        case 1: // cpi(r); cpd(r)
            t = z80->readb(z80->hl.w);
            z80->hl.w += incr;
            z80->bc.w--;
            uint8_t v = z80->af.hi - t;
            uint8_t hf = halfcarry(z80->af.hi, t, v);
            n = z80->af.hi - t - hf;
            f = flag_precalc[v] & 0xc0   // SZ000000
              | (hf << HF)               // SZ0H0000
              | ((n << 4) & MASK_YF)     // SZYH0000
              | (n & MASK_XF)            // SZYHX000
              | ((z80->bc.w != 0) << PF) // SYZHXP00
              | 2;                       // SYZHXPN0
            z80->af.lo = f | (z80->af.lo & MASK_CF);
            repe = z80->bc.w != 0 && v != 0;
            if (!is_repe || z80->bc.w == 1 || z80->af.hi == t) z80->memptr += incr;
            else if (is_repe) z80->memptr = z80->pc - 1; // instr_addr + 1
            break;
        case 2: // ini(r); ind(r)
            t = z80->inport(z80->bc.w);
            z80->writeb(z80->hl.w, t);
            z80->hl.w += incr;
            z80->memptr = z80->bc.w + incr;
            z80->bc.hi--;
            uint16_t k = t + ((z80->bc.lo + incr) & 255);
            uint8_t hcf = k > 255;
            uint8_t pf = has_even_parity((k & 7) ^ z80->bc.hi);
            z80->af.lo = flag_precalc[z80->bc.hi] & 0xfb // SZY0X000
                       | ((t >> 6) & MASK_NF)            // SZY0X0N0
                       | (hcf << HF)                     // SZYHX0N0
                       | (pf << PF)                      // SZYHXPN0
                       | hcf;                            // SZYHXPNC
            repe = z80->bc.hi != 0;
            if (is_repe && repe) z80->memptr = z80->pc - 1; // instr_addr + 1
            break;
        case 3: // oti(r); otd(r)
            z80->bc.hi--;
            z80->memptr = z80->bc.w + incr;
            t = z80->readb(z80->hl.w);
            z80->outport(z80->bc.w, t);
            z80->hl.w += incr;
            k = t + z80->hl.lo;
            hcf = k > 255;
            pf = has_even_parity((k & 7) ^ z80->bc.hi);
            z80->af.lo = flag_precalc[z80->bc.hi] & 0xfb // SZY0X000
                       | ((t >> 6) & MASK_NF)            // SZY0X0N0
                       | (hcf << HF)                     // SZYHX0N0
                       | (pf << PF)                      // SZYHXPN0
                       | hcf;                            // SZYHXPNC
            repe = z80->bc.hi != 0;
    }
    if (is_repe && repe) {
        z80->pc -= 2;
        uint8_t pch_leak = (z80->pc >> 8) & (MASK_YF | MASK_XF);
        z80->af.lo = (z80->af.lo & ~(MASK_YF | MASK_XF)) | pch_leak;
        z80->cycles = 21;
    }
    else z80->cycles = 16;
}

static void rot(z80_t *z80, uint8_t y, uint8_t z) {
    uint8_t v = z80->swap_hl ? hl_subst_indirect(z80) : reg(z80, z);
    uint8_t carry;
    switch (y) {
        case 0: // rlc
            carry = v >> 7;
            v = (v << 1) | carry;
            break;
        case 1: // rrc
            carry = v & 1;
            v = (carry << 7) | (v >> 1);
            break;
        case 2: // rl
            carry = v >> 7;
            v = (v << 1) | z80->af.lo & MASK_CF;
            break;
        case 3: // rr
            carry = v & 1;
            v = ((z80->af.lo & MASK_CF) << 7) | (v >> 1);
            break;
        case 4: // sla
            carry = v >> 7;
            v <<= 1;
            break;
        case 5: // sra
            carry = v & 1;
            v = (v & 128) | (v >> 1);
            break;
        case 6: // sll
            carry = v >> 7;
            v = (v << 1) | 1;
            break;
        case 7: // srl
            carry = v & 1;
            v >>= 1;
    }
    z80->af.lo = flag_precalc[v] | carry;
    if (z80->swap_hl) {
        set_hl_subst_indirect(z80, v);
        if (z != 6) set_reg_unsub(z80, z, v);        
        z80->set_memptr_to_index_plus_d = 1;
        z80->cycles = 23;
    } else {
        set_reg(z80, z, v);
        z80->cycles = z == 6 ? 15 : 8;
    }
}

static void invalid(z80_t *z80) {}

static void exec_unprefixed(z80_t *z80, uint8_t x, uint8_t y, uint8_t z, uint8_t p, uint8_t q) {
    switch (x) {
        case 0:
            switch (z) {
                case 0:
                    switch (y) {
                        case 0: nop(z80); break;
                        case 1: ex(z80, x, y); break;
                        case 2: djnz(z80); break;
                        default: jr(z80, y);
                    }
                    break;
                case 1: if (q) add(z80, p); else ld(z80, x, y, z, p, q); break;
                case 2: ld(z80, x, y, z, p, q); break;
                case 3: if (q) dec(z80, y, z, p); else inc(z80, y, z, p); break;
                case 4: inc(z80, y, z, p); break;
                case 5: dec(z80, y, z, p); break;
                case 6: ld(z80, x, y, z, p, q); break;
                case 7:
                    switch (y) {
                        case 0: rlca(z80); break;
                        case 1: rrca(z80); break;
                        case 2: rla(z80); break;
                        case 3: rra(z80); break;
                        case 4: daa(z80); break;
                        case 5: cpl(z80); break;
                        case 6: scf(z80); break;
                        case 7: ccf(z80);
                    }
            }
            break;
        case 1: if (y == z && z == 6) halt(z80); else ld(z80, x, y, z, p, q); break;
        case 2: alu(z80, x, y, z); break;
        case 3:
            switch (z) {
                case 0: ret(z80, y, z); break;
                case 1:
                    if (q) {
                        switch (p) {
                            case 0: ret(z80, y, z); break;
                            case 1: exx(z80); break;
                            case 2: jp(z80, y, z); break;
                            case 3: ld(z80, x, y, z, p, q); break;
                        }
                    } else
                        pop(z80, p);
                    break;
                case 2: jp(z80, y, z); break;
                case 3:
                    switch (y) {
                        case 0: jp(z80, y, z); break;
                        case 2: out(z80, x, y); break;
                        case 3: in(z80, x, y); break;
                        case 4: case 5: ex(z80, x, y); break;
                        case 6: di(z80); break;
                        case 7: ei(z80);
                    }
                    break;
                case 4: call(z80, y, z); break;
                case 5: if (q) call(z80, y, z); else push(z80, p); break;
                case 6: alu(z80, x, y, z); break;
                case 7: rst(z80, y);
            }
    }
}

static void exec_cb(z80_t *z80, uint8_t x, uint8_t y, uint8_t z) {
    switch (x) {
        case 0: rot(z80, y, z); break;
        case 1: bit(z80, y, z); break;
        case 2: res(z80, y, z); break;
        case 3: set(z80, y, z);
    }
}

static void exec_ed(z80_t *z80, uint8_t x, uint8_t y, uint8_t z, uint8_t p, uint8_t q) {
    switch (x) {
        case 0: case 3:
            invalid(z80); break;
        case 1:
            switch (z) {
                case 0: in(z80, x, y); break;
                case 1: out(z80, x, y); break;
                case 2: if (q) adc(z80, p); else sbc(z80, p); break;
                case 3: ld(z80, x, y, z, p, q); break;
                case 4: neg(z80); break;
                case 5: retn(z80); break;
                case 6: im(z80, y); break;
                case 7:
                    switch (y) {
                        case 4: rrd(z80); break;
                        case 5: rld(z80); break;
                        case 6: case 7: nop(z80); break;
                        default: ld(z80, x, y, z, p, q);
                    }
                    break;
            }
            break;
        case 2:
            if (z < 4 && y >= 4)
                bli(z80, y, z);
            else
                invalid(z80);
    }
}

static unsigned expect_postdisp_fn(z80_t *z80, uint8_t x, uint8_t y, uint8_t z) {
    return !z80->prefix && (
        (x == 0 && z == 0 && y > 1)
        || (z80->swap_hl && (
            (x == 0 && y == 6 && 4 <= z && z <= 6)
            || (x == 1 && (y == 6 || z == 6) && y != z)
            || (x == 2 && z == 6)
        ))
    );
}

static void update_accessors(z80_t *z80) {
    reg16_t *p;
    switch (z80->swap_hl) {
        case 0:    p = &z80->hl; break;
        case 0xdd: p = &z80->ix; break;
        case 0xfd: p = &z80->iy;
    }
    z80->p_hl_subst = z80->regpair[2] = z80->regpair2[2] = p;
}

// TODO: memptr=addr on 'Interrupt call to addr'
unsigned z80_step(z80_t *z80) {
    uint8_t last = 0; uint8_t b = 0;
    unsigned expect_predisp = 0; unsigned expect_postdisp = 0;
    unsigned is_postdisp    = 0;
    z80->cycles = 0;
    z80->op_was_ei = 0;
    while (1) {
        if (z80->halted) { 
            if (!z80->op_was_ei && (z80->nmi_requested || (z80->maskable_int_requested && z80->iff1))) {
                z80->halted = 0;
            } else {
                halt(z80);
                break;
            }
        }
        last = b;
        b = nextb(z80);
        if ((last == 0xdd || last == 0xfd) && !expect_predisp && !expect_postdisp) {
            if (b == 0xdd || b == 0xfd) {
                z80->swap_hl = b;
                continue;
            }
            if (b == 0xed) {
                z80->prefix = b; z80->swap_hl = 0;
                continue;
            }
            if (b == 0xcb) {
                z80->prefix = b; expect_predisp = 1;
                continue;
            }
        }
        if (expect_predisp) {
            z80->displace = b;
            expect_predisp = 0;
            continue;
        }
        uint8_t op = b;
        if (expect_postdisp) {
            z80->displace = b;
            expect_postdisp = 0;
            is_postdisp = 1;
            op = last;
        } else if (!z80->prefix && (b == 0xdd || b == 0xfd)) {
            z80->swap_hl = b;
            continue;
        } else if (!z80->prefix && (b == 0xcb || b == 0xed)) {
            z80->prefix = b;
            continue;
        }
        uint8_t x = op >> 6;
        uint8_t y = (op >> 3) & 7;
        uint8_t z = op & 7;
        if (!is_postdisp) {
            expect_postdisp = expect_postdisp_fn(z80, x, y, z);
            if (expect_postdisp) continue;
        }
        uint8_t p = y >> 1;
        uint8_t q = y & 1;
        z80->r = (z80->r & 0x80) | ((z80->r + ((z80->prefix || z80->swap_hl) ? 2 : 1)) & 0x7f);
        if (z80->swap_hl != z80->prev_swap_hl) {
            z80->prev_swap_hl = z80->swap_hl;
            update_accessors(z80);
        }
        if (z80->prefix == 0xcb)
            exec_cb(z80, x, y, z);
        else if (z80->prefix == 0xed)
            exec_ed(z80, x, y, z, p, q);
        else
            exec_unprefixed(z80, x, y, z, p, q);
        if (z80->set_memptr_to_index_plus_d) {
            z80->set_memptr_to_index_plus_d = 0;
            z80->memptr = ((z80->swap_hl == 0xdd) ? z80->ix.w : z80->iy.w) + z80->displace;
        }
        z80->prefix = z80->swap_hl = 0;
        break;
    }

    if (z80->op_was_ei) return z80->cycles;

    int32_t int_vec = -1;

    if (z80->nmi_requested) {
        z80->nmi_requested = 0;
        z80->iff1 = 0;
        int_vec = 0x66;
    } else if (z80->maskable_int_requested && z80->iff1) {
        z80->iff1 = z80->iff2 = 0;
        switch (z80->intmode) {
            case 0: fprintf(stderr, "interrupt mode 0 not implemented"); exit(1);
            case 1: int_vec = 0x0038; z80->cycles += 13; break;
            case 2: int_vec = z80->maskable_int_data; z80->cycles += 19;
        }
    }

    if (int_vec != -1) {
        z80->r = (z80->r & 0x80) | ((z80->r + 1) & 0x7f);
        z80->sp.w -= 2;
        writew(z80, z80->sp.w, z80->pc);
        z80->pc = z80->memptr = int_vec;
    }

    return z80->cycles;
}

void z80_trigger_nmi(z80_t *z80) {
    z80->nmi_requested = 1;
}

void z80_assert_maskable_int(z80_t *z80, uint16_t bus_data) {
    z80->maskable_int_requested = 1;
    z80->maskable_int_data = bus_data;
}

void z80_clear_maskable_int(z80_t *z80) {
    z80->maskable_int_requested = 0;
}
