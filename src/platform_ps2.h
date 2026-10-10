// SPDX-License-Identifier: MIT
// Single EE worker only. No synchronization against interrupts or IOP tasks.
// Included from platform.h after core.h; never include this directly.
#pragma once

static inline void b3AtomicStoreInt( b3AtomicInt* a, int value )
{
    a->value = value;
}

static inline int b3AtomicLoadInt( b3AtomicInt* a )
{
    return a->value;
}

static inline int b3AtomicFetchAddInt( b3AtomicInt* a, int increment )
{
    int old = a->value;
    a->value = old + increment;
    return old;
}

static inline bool b3AtomicCompareExchangeInt( b3AtomicInt* a, int expected, int desired )
{
    if (a->value != expected) return false;
    a->value = desired;
    return true;
}

static inline void b3AtomicStoreU32( b3AtomicU32* a, uint32_t value )
{
    a->value = value;
}

static inline uint32_t b3AtomicLoadU32( b3AtomicU32* a )
{
    return a->value;
}

static inline uint32_t b3AtomicLoadU32Raw( uint32_t* a )
{
    return *a;
}

static inline uint32_t b3AtomicFetchOrU32( uint32_t* a, uint32_t mask )
{
    uint32_t old = *a;
    *a = old | mask;
    return old;
}

static inline int64_t b3AtomicFetchAddI64( b3AtomicI64* a, int64_t increment )
{
    int64_t old = a->value;
    a->value = old + increment;
    return old;
}

static inline int64_t b3AtomicLoadI64( b3AtomicI64* a )
{
    return a->value;
}
