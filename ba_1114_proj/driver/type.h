/*****************************************************************************
 *   type.h:  Type definition Header file for NXP Family


 *   Microprocessors
 *
 *   Copyright(C) 2006, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.04.01  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#ifndef __TYPE_H__
#define __TYPE_H__

// CodeRed - ifdef for GNU added to avoid potential clash with stdint.h
#if defined   (  __GNUC__  )
#include <stdint.h>
#else

/* exact-width signed integer types */
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

 /* exact-width unsigned integer types */
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

#endif // __GNUC__ 

/* Defined by NXP */
#if 0
#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif
#endif


#ifndef NULL
   #define NULL  ((void*)0L)
#endif

#define SET   (1U)
#define RESET (0)

#define ON    (1U)
#define OFF   (0)

#define FALSE (0)
#define TRUE  (!FALSE)


typedef uint8_t boolean_t;

typedef uint8_t  bitfield8_t;
typedef uint16_t bitfield16_t;
typedef uint32_t bitfield32_t;



#define NIBBLE_SIZE (4U)            /* nibble size in bits */
#define NIBBLES_IN_BYTE (2U)        /* number of nibbles in byte */

#define NIBBLE_LOW_MASK        (0x0FU)    /* Mask for a lower nibble of a byte  */
#define NIBBLE_HIGH_MASK       (0xF0U)    /* Mask for a higher nibble of a byte */
#define U16_LOW_BYTE_MASK      0x00FFU
#define U16_HIGH_BYTE_MASK     0xFF00U
#define U32_LOW_BYTE_MASK      0x000000FFUL
#define U32_MID_LOW_BYTE_MASK  0x0000FF00UL
#define U32_MID_HIGH_BYTE_MASK 0x00FF0000UL
#define U32_HIGH_BYTE_MASK     0xFF000000UL

#define GET_U8_LOW_NIBBLE(a)       (uint8_t)((a) & NIBBLE_LOW_MASK)
#define GET_U8_HIGH_NIBBLE(a)      (uint8_t)(((a) & NIBBLE_HIGH_MASK) >> NIBBLE_SIZE)
#define GET_U16_LOW_BYTE(a)        (uint8_t)((a) & (U16_LOW_BYTE_MASK))
#define GET_U16_HIGH_BYTE(a)       (uint8_t)((((a) & (U16_HIGH_BYTE_MASK))) >> 8U)
#define GET_U32_LOW_BYTE(a)        (uint8_t)((a) & (U32_LOW_BYTE_MASK))
#define GET_U32_MID_LOW_BYTE(a)    (uint8_t)((((a) & (U32_MID_LOW_BYTE_MASK))) >> 8U)
#define GET_U32_MID_HIGH_BYTE(a)   (uint8_t)((((a) & (U32_MID_HIGH_BYTE_MASK))) >> 16U)
#define GET_U32_HIGH_BYTE(a)       (uint8_t)((((a) & (U32_HIGH_BYTE_MASK))) >> 24U)

/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

/* x=target variable, y=mask */
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) ((x) & (y))


#endif  /* __TYPE_H__ */
