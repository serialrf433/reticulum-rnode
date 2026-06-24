#pragma once
// src/KissCodec.h — pure, hardware-independent KISS escaping primitives.
//
// Header-only so both the firmware (Kiss.cpp) and the native unit tests
// share a single implementation with no Arduino dependency. See SPEC.md
// §8.1: frame = FEND || cmd || escaped(data) || FEND.

#include <stdint.h>
#include <stddef.h>

namespace rlr { namespace kisscodec {

constexpr uint8_t FEND  = 0xC0;
constexpr uint8_t FESC  = 0xDB;
constexpr uint8_t TFEND = 0xDC;
constexpr uint8_t TFESC = 0xDD;

// Escape `n` payload bytes from `in` into `out` (capacity `cap`).
// Returns the number of bytes written, or 0 if the result would overflow
// `cap` (caller treats 0-with-n>0 as "doesn't fit").
inline size_t escape(const uint8_t* in, size_t n, uint8_t* out, size_t cap) {
    size_t p = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t b = in[i];
        if (b == FEND) {
            if (p + 2 > cap) return 0;
            out[p++] = FESC; out[p++] = TFEND;
        } else if (b == FESC) {
            if (p + 2 > cap) return 0;
            out[p++] = FESC; out[p++] = TFESC;
        } else {
            if (p + 1 > cap) return 0;
            out[p++] = b;
        }
    }
    return p;
}

// Unescape `n` bytes (a payload with the surrounding FENDs already
// stripped) from `in` into `out` (capacity `cap`). Returns bytes written.
inline size_t unescape(const uint8_t* in, size_t n, uint8_t* out, size_t cap) {
    size_t p = 0;
    bool esc = false;
    for (size_t i = 0; i < n; i++) {
        uint8_t b = in[i];
        if (esc) {
            esc = false;
            if (b == TFEND) b = FEND;
            else if (b == TFESC) b = FESC;
            if (p < cap) out[p++] = b;
        } else if (b == FESC) {
            esc = true;
        } else {
            if (p < cap) out[p++] = b;
        }
    }
    return p;
}

}} // namespace rlr::kisscodec
