/* type and settings for compiler, common header 
 * 
 * Copyright (C) 2018 moecmks
 * This file is part of YuduliyaGB.
 * 
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 */

#ifndef SETTINGS_H
#define SETTINGS_H 1

#ifdef _WIN32
# define _CRT_SECURE_NO_DEPRECATE
#endif 

#ifdef _WIN32
# include <intrin.h>
#endif 

#include "stdint.h" // old vc++ not included stdint.h in mscrt
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <process.h>
#include <assert.h>
#include <string.h>
#include <math.h>

#ifndef NULL_DEFINED
# ifdef __cplusplus
#  define null nullptr
# else 
#  define null ((void *)0)
# endif  
#endif 

#if !defined (BOOL_DEFINED) \
 && !defined (__cplusplus)
     typedef int bool;
  # define true 1
  # define false 0
#endif 

#if defined (_MSC_VER) || defined (__ICC) || defined (__INTEL_COMPILER) /* MSVC/ICC starting... */
# define finline static  __forceinline
# define dinline         __declspec(noinline)
# define callstd         __stdcall
# define callc           __cdecl
# define ccalign(x)      __declspec(align(x))
# define cvimpl          __declspec(dllexport) 
#elif defined (__GNUC__) || defined (__GNUG__) /* MSVC/ICC end... GNUC starting */
# define finline static  __attribute__((always_inline))
# define dinline         __attribute__((noinline))
# define callstd         __attribute__((stdcall))
# define callc           __attribute__((cdecl))
# define ccalign(x)      __attribute__((aligned(x)))
# define cvimpl          __attribute__((dllexport))
#else /* unsupported */
# error unsupported compiler! 
#endif 
# define CMP0_EQUAL 0
# define CMP0_ABOVE 1
# define CMP0_LOW 2
# define CMP0_NaN 3

finline 
int cmp0_double (double *value) {
  /*  XXX: IEEE754  */
  int64_t *t = (int64_t *)(value);
  /* S:nocare M and E is 0 */
               /* 0123456701234567 */
  if (! (t[0] & 0x7FFFFFFFFFFFFFFF)) /* -0 or +0*/
    return CMP0_EQUAL;
  if (  ( ((int32_t *)&t[0])[1] & 0x80000000)) 
    return CMP0_LOW;
  else 
    return CMP0_ABOVE;
}

/* some hardware settings. */
#define JOYPAD_FREQ_IN_DMG 50.3
#define JOYPAD_FREQ_IN_CGB 50.3 
#define JOYPAD_INTERRUPT_GEN_ENABLE 
#define JOYPAD_LOAD_ACTION_IN_P1415_ALLOPEN 3 /* 0:OR 1:AND 2:XOR 3:AND-NOT */
#define TIMER_RESET_IN_RESET_FREQ_MOD_
#define PPU_POST_RENDER_TIME 0 /* 0:VBLANK START 1:VBLANK END. */
#define _DEBUG_GB

#ifdef _DEBUG_GB
# define DEBUG_OUT(...) printf (__VA_ARGS__)
#else 
# define DEBUG_OUT(...) ((void)0)
#endif 

#ifdef _WIN32
# ifdef _MSC_VER
#  ifdef _DEBUG_GB
#    define DEBUG_BREAK() __debugbreak()
#  else 
#    define DEBUG_BREAK() ((void)0)
#  endif 
# else 
# endif 
#endif 

#endif 