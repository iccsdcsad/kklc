/* YuduliyaGB Public Interface 
 *
 * Copyright (C) 2018 moecmks
 * This file is part of YuduliyaGB.
 * 
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 */

#ifndef DMG_H
#define DMG_H 1

#ifdef __cplusplus
extern "C" {
#endif

#include "settings.h" 

#define APU_SAMPLE_TYPE_BYTE 0
#define APU_SAMPLE_TYPE_WORD 1 
#define APU_SAMPLE_TYPE_FLOAT 2 
#define APU_SAMPLE_TYPE_DEFAULT APU_SAMPLE_TYPE_WORD
typedef int APU_SAMPLE_TYPE;

#define APU_SAMPLE_FREQ_11025 11025 /* 11025hz*/
#define APU_SAMPLE_FREQ_22050 22050 /* 22050hz*/
#define APU_SAMPLE_FREQ_44100 44100 /* 44100hz*/
#define APU_SAMPLE_FREQ_DEFAULT APU_SAMPLE_FREQ_44100
#define APU_SAMPLE_FREQ_MAX APU_SAMPLE_FREQ_44100
#define APU_SAMPLE_SBIT_MAX 4 /* float 32bit. */
#define APU_SAMPLE_CHANNEL_NUMS 2 /* always 2 channel in dmg/cgb */
typedef int APU_SAMPLE_FREQ;

#define APU_BUFFER_BUMS 32
#define APU_BUFFER_CHUNK_SIZE_BASE_SAMPLE 245

#define PPU_SAMPLE_RGB15 0 /* 16bit pixel */
#define PPU_SAMPLE_RGB24 1 /* 32bit pixel (alpha ignore) */
#define PPU_SAMPLE_DEFAULT PPU_SAMPLE_RGB15
typedef int PPU_SAMPLE_FORMAT;

#define PPU_DMG_PAL_WHITE 0
#define PPU_DMG_PAL_BRONW 1
#define PPU_DMG_PAL_CYAN 2
#define PPU_DMG_PAL_DEFAULT PPU_DMG_PAL_BRONW

/* host device callback for input */
struct controller_pad {
  uint8_t right:1; /* 1?down/edge gen:up */
  uint8_t left:1; /* 1?down/edge gen:up */
  uint8_t up:1; /* 1?down/edge gen:up */
  uint8_t down:1; /* 1?down/edge gen:up */
  uint8_t a:1; /* 1?down/edge gen:up */
  uint8_t b:1; /* 1?down/edge gen:up */
  uint8_t select:1; /* 1?down/edge gen:up */
  uint8_t start:1; /* 1?down/edge gen:up */
};
/* host device callback for lcd video */
struct ppu_framebuffer {
  int32_t w;
  int32_t h;

  union {
    uint16_t *buf; /* gameboy use x1b5g5r5 pixel format */
    uint32_t *buf32; /* ext AXR8G8B8 pixel format */
  };
  int32_t pitch;
};
/* host device callback for PSG Sound Synthesizer */
struct apu_framebuffer {
  void *buf; /* Channel always 2,  PCM format can be: byte/word/float PCM*/
  int32_t id;
  int32_t sam; /* sample can be 44100hz/22050hz/11025hz */
  int32_t len;
};
struct ppu_dmg_palette16 {
  uint16_t bg_pal[4];
  uint16_t sp_pal[2][4];
};
struct ppu_dmg_palette32 {
  uint32_t bg_pal[4];
  uint32_t sp_pal[2][4];
};

uint8_t callstd gameboy_read (struct gameboy *gb, uint16_t address);
uint16_t callstd  gameboy_read2 (struct gameboy *gb, uint16_t address);
void  callstd gameboy_write (struct gameboy *gb, uint16_t address, uint8_t value);
void  callstd gameboy_write2 (struct gameboy *gb, uint16_t address, uint16_t value);

void gameboy_controller_drv (struct gameboy *gb, void (*controller_hostdrv) 
             (struct gameboy *, 
             void *, 
                struct controller_pad *, /* gb-self for recv joypadbuffer */ 
             struct controller_pad * /* host-edge */), void *obj);

void gameboy_lcdvideo_drv (struct gameboy *gb, void (*lcdvideo_hostdrv) 
             (struct gameboy *, 
              void *, 
              struct ppu_framebuffer *), void *obj);

void gameboy_sound_drv (struct gameboy *gb, void (*sound_hostdrv) 
             (struct gameboy *, 
              void *, 
              struct apu_framebuffer *), void *obj);

void gameboy_run_ms (struct gameboy *gb, double ms);
void gameboy_run_frame (struct gameboy *gb);
int gameboy_loadrom (struct gameboy *gb, FILE *fp);
int gameboy_reset (struct gameboy *gb) ;
int gameboy_kinit (void); /* gameboy global env context init */
int gameboy_kuninit (void); /* gameboy global env context destroy */
int gameboy_init (struct gameboy **gb);
int gameboy_uninit (struct gameboy **gb);
int gameboy_startup (struct gameboy *gb) ;
int gameboy_suspend (struct gameboy *gb);
int gameboy_issuspend (struct gameboy *gb);
int gameboy_setpcm_format (struct gameboy *gb, APU_SAMPLE_FREQ sample, APU_SAMPLE_TYPE sample_type);
int gameboy_setppu_format (struct gameboy *gb, PPU_SAMPLE_FORMAT format);
int gameboy_getaudiobuf (struct gameboy *gb, void **buf);
int gameboy_getppubuf (struct gameboy *gb, struct ppu_framebuffer *ppuframe);
int gameboy_copy_apubuffer (struct gameboy *gb,  void **buf);
int gameboy_getbank_size_inbyte (struct gameboy *gb);
int gameboy_getbank_size_insample (struct gameboy *gb);
void gameboy_reset_sample_type (struct gameboy *gb, APU_SAMPLE_TYPE sample_type);
void gameboy_reset_sample_freq (struct gameboy *gb, APU_SAMPLE_FREQ sample_freq);
int gameboy_get_sample_bit_depth (struct gameboy *gb);
void gameboy_reset_dmgpal (struct gameboy *gb, struct ppu_dmg_palette32 *pal);
void gameboy_get_dmgpal (struct gameboy *gb, struct ppu_dmg_palette32 *pal);

#ifdef __cplusplus
 }
#endif

#endif 