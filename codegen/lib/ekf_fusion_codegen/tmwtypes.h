/*
 * tmwtypes.h - MATLAB Coder runtime type definitions
 * Required by rtwtypes.h
 */
#ifndef TMWTYPES_H
#define TMWTYPES_H

#include <stdint.h>
#include <stddef.h>

/* Integer types */
typedef signed char    int8_T;
typedef unsigned char  uint8_T;
typedef short          int16_T;
typedef unsigned short uint16_T;
typedef int            int32_T;
typedef unsigned int   uint32_T;

#if defined(_WIN64) || defined(__LP64__)
typedef long long          int64_T;
typedef unsigned long long uint64_T;
#else
typedef long long          int64_T;
typedef unsigned long long uint64_T;
#endif

/* Float types */
typedef float  real32_T;
typedef double real64_T;
typedef double real_T;
typedef float  real32_T;

/* Boolean */
typedef unsigned char boolean_T;

/* Character */
typedef char char_T;
typedef unsigned char uchar_T;

/* Byte */
typedef char           byte_T;

/* Integer types used in generated code */
typedef int8_T   int8;
typedef uint8_T  uint8;
typedef int16_T  int16;
typedef uint16_T uint16;
typedef int32_T  int32;
typedef uint32_T uint32;

#ifndef TRUE
#define TRUE  (1U)
#endif
#ifndef FALSE
#define FALSE (0U)
#endif

#endif /* TMWTYPES_H */
