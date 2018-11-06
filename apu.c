/* PSG Sound Synthesizer
 * 
 * Copyright (C) 2018 moecmks
 * This file is part of YuduliyaGB.
 * 
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 */

 /*
   TODO: more details 
   This part of the code is very bad.
  */
#include "gameboy.h"
#include "inline.inl"

static 
void default_update (struct apu *apu, void *sound_drvobj, struct apu_framebuffer *fmebuf) {
  DEBUG_OUT ("%s:%d please set sound callback( controller_setupdate function)\n", __FILE__, __LINE__);
  DEBUG_BREAK();
}
static const int16_t
dac_quant16_16bitpcm[16] = {
 // 0//,-32768 + 4096*0,  /* volume 0 */
  32767 - 4096/2*15,  /* volume 8 */
  32767 - 4096/2*14,  /* volume 9*/
  32767 - 4096/2*13,  /* volume 10 */
  32767 - 4096/2*12,  /* volume 11*/
  32767 - 4096/2*11,  /* volume 12 */
  32767 - 4096/2*10,  /* volume 13*/
  32767 - 4096/2*9,  /* volume 14 */
  32767 - 4096/2*8,  /* volume 15*/
  32767 - 4096/2*7,  /* volume 8 */
  32767 - 4096/2*6,  /* volume 9*/
  32767 - 4096/2*5,  /* volume 10 */
  32767 - 4096/2*4,  /* volume 11*/
  32767 - 4096/2*3,  /* volume 12 */
  32767 - 4096/2*2,  /* volume 13*/
  32767 - 4096/2*1,  /* volume 14 */
  32767 - 4096/2*0,  /* volume 15*/
};
static const int16_t
dac_quant16_16bitpcmt[16] = {
 0//,-32768 + 4096*0,  /* volume 0 */
 -32768 + 4096*1,  /* volume 1 */
 -32768 + 4096*2,  /* volume 2 */
 -32768 + 4096*3, /* volume 3 */
 -32768 + 4096*4,  /* volume 4 */
 -32768 + 4096*5,  /* volume 5 */
 -32768 + 4096*6,   /* volume 6 */
 -32768 + 4096*7,   /* volume 7 */
  32767 - 4096*7,  /* volume 8 */
  32767 - 4096*6,  /* volume 9*/
  32767 - 4096*5,  /* volume 10 */
  32767 - 4096*4,  /* volume 11*/
  32767 - 4096*3,  /* volume 12 */
  32767 - 4096*2,  /* volume 13*/
  32767 - 4096*1,  /* volume 14 */
  32767 - 4096*0,  /* volume 15*/
};
static const float
dac_quant16_fbitpcmt[16] = {
 0.0//,-32768 + 4096*0,  /* volume 0 */
 -1.0 + 0.125*1,  /* volume 1 */
 -1.0 + 0.125*2,  /* volume 2 */
 -1.0 + 0.125*3, /* volume 3 */
 -1.0 + 0.125*4,  /* volume 4 */
 -1.0 + 0.125*5,  /* volume 5 */
 -1.0 + 0.125*6,   /* volume 6 */
 -1.0 + 0.125*7,   /* volume 7 */
  1.0 - 0.125*7,  /* volume 8 */
  1.0 - 0.125*6,  /* volume 9*/
  1.0 - 0.125*5,  /* volume 10 */
  1.0 - 0.125*4,  /* volume 11*/
  1.0 - 0.125*3,  /* volume 12 */
  1.0 - 0.125*2,  /* volume 13*/
  1.0 - 0.125*1,  /* volume 14 */
  1.0 - 0.125*0,  /* volume 15*/
};
finline 
int16_t
AddSaturaItSign16 (int16_t u, int16_t v) {
  int32_t out = u+v;
  if (out > INT16_MAX)
    out = INT16_MAX;
  else if (out < INT16_MIN)
    out = INT16_MIN;
  return out;
}
finline 
uint8_t
AddSaturaItUSign8 (uint8_t u, uint8_t v) {
  uint16_t out = u+v;
  if (out >= 0xFF)
    out = 0xFF;
  return (uint8_t)out;
}
finline 
float
AddSaturaItFloat1 (float u, float v) {
  float out = u + v;
  if (out > 0.9999999999999999999999999999999999)
    out = 1.0;
  if (out <-0.9999999999999999999999999999999999)
    out =-1.0;
  return out;
}
finline 
intptr_t lfsr15 (intptr_t seed) {
  /* step15 lfsr: 15, 14 (14,13)*/
  intptr_t d2 = (seed & 0x4000) >> 14;
  intptr_t d3 = (seed & 0x2000) >> 13;
  intptr_t out = 1 & (d2 ^ d3);
  seed <<= 1;
  seed |= out;
  seed &= 0x7FFF;
  return seed;
}
finline 
intptr_t lfsr7 (intptr_t seed) {
  /* step7 lfsr: 7, 6 (6,5)*/
  intptr_t d2 = (seed & 0x40) >> 6;
  intptr_t d3 = (seed & 0x20) >> 5;
  intptr_t out = 1 & (d2 ^ d3);
  seed <<= 1;
  seed |= out;
  seed &= 0x7F;
  return seed;
}
finline 
void squ_gen_freq_calc (struct apu *apu, 
                        struct square_channel *squ,
                         uint8_t freq_lo, 
                           uint8_t freq_hi) 
{
# define SQU_BASE_DIV 4.0
  squ->freqT2 = (double) (( (freq_hi & 7) * 256) | freq_lo);
  squ->squGen_T = (4194304.0 / (2048.0 - squ->freqT2)) / SQU_BASE_DIV;
  squ->squGen_T = apu->gb->mach_tools->cpu_freq / squ->squGen_T;
  squ->squGen = 0.0;
  squ->squPhase = 0;
}
finline 
void wav_gen_freq_calc (struct apu *apu, 
                         uint8_t freq_lo, 
                           uint8_t freq_hi) 
{
#define WAV_BASE_DIV 2.0
  apu->wav_chan.freqT2 = (double) (( (freq_hi & 7) * 256) | freq_lo);
  apu->wav_chan.wavGen_T = (4194304.0 / (2048.0 - apu->wav_chan.freqT2)) / WAV_BASE_DIV;
  apu->wav_chan.wavGen_T = apu->gb->mach_tools->cpu_freq / apu->wav_chan.wavGen_T;
}
finline 
void noi_gen_freq_calc (struct apu *apu, uint8_t shift_div) 
{
#define NOI_BASE_DIV 2.0
  apu->noi_chan.noiGen_T = 1048576.0/ (double)((shift_div & 7)+1)/ (double)(2 << (shift_div >> 4)) / NOI_BASE_DIV;
  apu->noi_chan.noiGen_T =apu->gb->mach_tools->cpu_freq/ apu->noi_chan.noiGen_T;
}
finline
  void freq_reset (struct apu *apu) 
{
  apu->fseq.fseq = 0;
  apu->fseq.fseq_count = 0;
  apu->fseq.fseq_div = 0;
  apu->fseq.fseqT = apu->gb->mach_tools->cpu_freq/ 512.0;
}

void apu_setupdate_ (struct apu *apu, void (*update) 
            (struct apu *, 
            void *, 
              struct apu_framebuffer *), void *obj)
{
  apu->sound_hostdrv = update;
  apu->sound_drvobj = obj;
}

void apu_reset (struct apu *apu) {

  int id;
  for (id =0; (0xFF10 + id) < 0xFF26; id++)
    gameboy_write (apu->gb, 0xFF10 + id, 0);


  freq_reset (apu);
  squ_gen_freq_calc (apu, & apu->squ_chan[0], 0, 0);
  squ_gen_freq_calc (apu, & apu->squ_chan[1], 0, 0);
  wav_gen_freq_calc (apu, 0, 0);
  noi_gen_freq_calc (apu, 0);
}

void apu_reset2 (struct apu *apu) {

  int id;
  for (id =0; (0xFF10 + id) < 0xFF26; id++)
    gameboy_write (apu->gb, 0xFF10 + id, 0);

  // gameboy_write (apu->gb, 0xFF12, 0xF3);
  freq_reset (apu);
  squ_gen_freq_calc (apu, & apu->squ_chan[0], 0, 0);
  squ_gen_freq_calc (apu, & apu->squ_chan[1], 0, 0);
  wav_gen_freq_calc (apu, 0, 0);
  noi_gen_freq_calc (apu, 0);
}
void apu_write (struct apu *apu, uint16_t addr, uint8_t value) {
  int id = addr >= 0xFF16 ? 1 : 0;
  uint8_t vOld = apu->reg26_NR52;
  uint8_t vAfter=0;
  struct square_channel *chan =&  apu->squ_chan[id];
  /* TODO: When the APU is disabled, all registers in the range $FF10-$FF2F are
   * unreadable and unwritable except for register $FF26(NR52).
   */
  switch (addr) {
  case 0xFF10:  /* FF10 - NR10 - Channel 1 Sweep register (R/W) */
  case 0xFF15:  /* useless */
    if (!(chan->sweep_div = (value >> 4) & 7))
      chan->sweep_en = false;
    else 
      chan->sweep_en = true;
    chan->s_sweep = value;
    break;
  case 0xFF11:
  case 0xFF16: 
    chan->vol_len = 
    chan->vol_lenT = 64 - (value & 63);
    if (id == 0) apu->squ_on = 1;
    else apu->squ2_on = 1;
    chan->chan_en = true;
    chan->s_dutylen = value;
    break;
  case 0xFF12: 
  case 0xFF17: 
    chan->vol_init =
    chan->vol_render = value >> 4;
    chan->env_div = value & 7;
    chan->envlope_en = !!chan->env_div;
    /* check DAC, if close, channel disable */
    if (chan->vol_init == 0) {
      if (id == 0) apu->squ_on = 0;
      else apu->squ2_on = 0;
      chan->chan_en = false;
    } else {
      if (id == 0) apu->squ_on = 1;
      else apu->squ2_on = 1;
      chan->chan_en = true;
    }
    chan->s_envlope = value;
    break;
  case 0xFF13: 
  case 0xFF18: 
    chan->s_freqlo = value;
    break;
  case 0xFF14:
  case 0xFF19:
    if (value & NRX4_ON_MASK
    /*  && !(chan->s_freqhi_rst & NRX4_ON_MASK)*/)
    {
      /*  Trigger Event
       *  Writing a value to NRx4 with bit 7 set causes the following things to occur:
       * 
       *  Channel is enabled (see length counter).
       *  If length counter is zero, it is set to 64
       *  Frequency timer is reloaded with period.
       *  Volume envelope timer is reloaded with period.
       *  Channel volume is reloaded from NRx2.
       *  Square 1's sweep does several things 
       *  Note that if the channel's DAC is off, after the above actions occur the channel will be immediately disabled again.
      */
      if ( chan->vol_len  == 0)
        chan->vol_len = chan->vol_lenT = 64;
      if (id == 0) {
        apu->squ_on = 1;
        /*  During a trigger event, several things occur:

            Square 1's frequency is copied to the shadow register.
            The sweep timer is reloaded.
            The internal enabled flag is set if either the sweep period or shift are non-zero, cleared otherwise.
            If the sweep shift is non-zero, frequency calculation and the overflow check are performed immediately.
         */
        if (!(chan->sweep_div = (chan->s_sweep >> 4) & 7))
          chan->sweep_en = false;
        else chan->sweep_en = true; /* TODO:*/
      } else apu->squ2_on = 1;

      chan->vol_init =
      chan->vol_render = chan->s_envlope >> 4;
      chan->env_div = chan->s_envlope & 7;
      chan->envlope_en = !!chan->env_div;
      chan->chan_en = true;
      chan->seq.len_divc =
      chan->seq.sweep_divc = 
      chan->seq.env_divc = 0;
    }
    chan->s_freqhi_rst = value;
    squ_gen_freq_calc (apu, chan, chan->s_freqlo, chan->s_freqhi_rst);
    break;
  case 0xFF1A:
    apu->wav_chan.chan_en =
    apu->wav_on = !!(value & 0x80); 
    apu->wav_chan.w_dac = value;
    break;
  case 0xFF1B:
    apu->wav_chan.vol_len =
    apu->wav_chan.vol_lenT = 256 - value;
    apu->wav_chan.chan_en = true;
    apu->wav_on = 1; 
    apu->wav_chan.w_sndlen = value;
    break;
  case 0xFF1C:
    if ((apu->wav_chan.vol_sft = ((value & 0x60) >> 5)) == 0)
      apu->wav_chan.vol_sft = 4;
    apu->wav_chan.w_outlevel = value;
    break;
  case 0xFF1D:
    apu->wav_chan.w_freqlo = value;
    break;
  case 0xFF1E:
    if (value & NRX4_ON_MASK
     /* && !(apu->wav_chan.w_freqhi_rst & NRX4_ON_MASK)*/)
    {
      /*  Trigger Event
       *  Writing a value to NR34 with bit 7 set causes the following things to occur:
       * 
       *  Channel is enabled (see length counter).
       *  If length counter is zero, it is set to 256.
       *  Frequency timer is reloaded with period.
       *  Wave channel's position is set to 0 but sample buffer is NOT refilled.
       *  Note that if the channel's DAC is off, after the above actions occur the channel will be immediately disabled again.
      */
      if (!apu->wav_chan.vol_len)
        apu->wav_chan.vol_len = apu->wav_chan.vol_lenT = 256;
      apu->wav_on = 1;

      if ((apu->wav_chan.vol_sft = (apu->wav_chan.w_outlevel & 0x60) >> 5) == 0)
        apu->wav_chan.vol_sft = 4;
      apu->wav_chan.seq.len_divc = 0;  
      apu->wav_chan.chan_en = true;
    }
    wav_gen_freq_calc (apu, apu->wav_chan.w_freqlo, apu->wav_chan.w_freqhi_rst);
    apu->wav_chan.w_freqhi_rst = value; 
    break;
  case 0xFF1F:
    apu->noi_chan.n_useless = value;
    break;
  case 0xFF20:
    apu->noi_chan.vol_len = 
    apu->noi_chan.vol_lenT = 64 - (value & 63);
    apu->noi_chan.chan_en = true;
    apu->noi_on = 1;
    apu->noi_chan.n_sndlen = value;
    break;
  case 0xFF21:
    apu->noi_chan.vol_init = 
    apu->noi_chan.vol_render = value >> 4;
    apu->noi_chan.env_div = value & 7;
    apu->noi_chan.envlope_en = !!apu->noi_chan.env_div;
    apu->noi_on = !!apu->noi_chan.vol_init;
    apu->noi_chan.chan_en = !! apu->noi_on;
    apu->noi_chan.n_envlope = value;
    break;
  case 0xFF22:
    if ((apu->noi_chan.n_sftdiv_lfsr ^ value) & 0x08) {
      /* lfsr step adjust */
      if (value & 0x08) /* step15 -> step7*/ 
        apu->noi_chan.lfsrSeed = 0x7F;
      else/* step7 -> step15*/
        apu->noi_chan.lfsrSeed = 0x7FFF;
    }
    noi_gen_freq_calc (apu, apu->noi_chan.n_sftdiv_lfsr);
    apu->noi_chan.n_sftdiv_lfsr = value;
    break;
  case 0xFF23:
    if (value & NRX4_ON_MASK
     /* && !(apu->noi_chan.noiReg_NRX0_X4[4] & NRX4_ON_MASK)*/)
    {
      /*  Trigger Event
       *  Writing a value to NR44 with bit 7 set causes the following things to occur:
       * 
       *  Channel is enabled (see length counter).
       *  If length counter is zero, it is set to 64 
       *  Frequency timer is reloaded with period.
       *  Volume envelope timer is reloaded with period.
       *  Channel volume is reloaded from NR42.
       *  Noise channel's LFSR bits are all set to 1.
       *  Note that if the channel's DAC is off, after the above actions occur the channel will be immediately disabled again.
      */    
      if (apu->noi_chan.vol_len == 0)
        apu->noi_chan.vol_len = apu->noi_chan.vol_lenT = 64;
      apu->noi_on = 1;
      apu->noi_chan.vol_init = 
      apu->noi_chan.vol_render = apu->noi_chan.n_envlope >> 4;
      apu->noi_chan.env_div = apu->noi_chan.n_envlope & 7;
      apu->noi_chan.envlope_en = !!apu->noi_chan.env_div;
      apu->noi_chan.chan_en = true;
      apu->noi_chan.seq.len_divc = 0;
      apu->noi_chan.seq.env_divc = 0;
      if (apu->noi_chan.n_sftdiv_lfsr & 0x08)
        apu->noi_chan.lfsrSeed = 0x7F;
      else apu->noi_chan.lfsrSeed = 0x7FFF;
    }
    noi_gen_freq_calc (apu, apu->noi_chan.n_sftdiv_lfsr);
    apu->noi_chan.n_rst = value;
    break;
  case 0xFF24:
    apu->reg24_NR50 = value;
    break;
  case 0xFF25:
    apu->reg25_NR51 = value;
    break;
  case 0xFF26:
    if (value & NRX4_ON_MASK 
       && !apu->snd_master_en )
    {
      apu_reset2 (apu);
    }
    else if (!(value & NRX4_ON_MASK) 
       && apu->snd_master_en )
    {
      apu_reset (apu);
    }
    apu->reg26_NR52 = value & 0xF0;
    break;
  case 0xFF30: /* AUD3WAVERAM */
  case 0xFF31: /* AUD3WAVERAM */
  case 0xFF32: /* AUD3WAVERAM */
  case 0xFF33: /* AUD3WAVERAM */
  case 0xFF34: /* AUD3WAVERAM */
  case 0xFF35: /* AUD3WAVERAM */
  case 0xFF36: /* AUD3WAVERAM */
  case 0xFF37: /* AUD3WAVERAM */
  case 0xFF38: /* AUD3WAVERAM */
  case 0xFF39: /* AUD3WAVERAM */
  case 0xFF3A: /* AUD3WAVERAM */
  case 0xFF3B: /* AUD3WAVERAM */
  case 0xFF3C: /* AUD3WAVERAM */
  case 0xFF3D: /* AUD3WAVERAM */
  case 0xFF3E: /* AUD3WAVERAM */
  case 0xFF3F: /* AUD3WAVERAM */
    apu->wav_chan.pcm4_NR30_3F[addr - 0xFF30] = value;
    apu->wav_chan.pcm4_T[(addr - 0xFF30) * 2+0] = value >> 4;
    apu->wav_chan.pcm4_T[(addr - 0xFF30) * 2+1] = value & 15;
    break;
  default:
    apu->gb->unknow_ram[addr] = value;
    return ;
  }
  // vAfter = apu->reg26_NR52;
  // DEBUG_OUT ("APU_WRITE:%04X VALUE:%02X?%02X%%%02X++%02X\n", addr, value, vOld, vAfter, apu->squ_chan[0].squReg_NRX0_X4[2]>>4);
}

uint8_t apu_read (struct apu *apu, uint16_t addr) {
  uint8_t value;
  static uint8_t regm[] = {
  /* NRx0  NRx1  NRx2  NRx3  NRx4 */
     0x80, 0x3F, 0x00, 0xFF, 0xBF,
     0xFF, 0x3F, 0x00, 0xFF, 0xBF,
     0x7F, 0xFF, 0x9F, 0xFF, 0xBF,
     0xFF, 0xFF, 0x00, 0x00, 0xBF,
     0x00, 0x00, 0x70,
     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };
  switch (addr) {
  case 0xFF10:
  case 0xFF11:
  case 0xFF12:
  case 0xFF13:
  case 0xFF14:
    value = apu->squ_chan[0].s_nx_start[addr - 0xFF10];
    break;
  case 0xFF15:
  case 0xFF16:
  case 0xFF17:
  case 0xFF18:
  case 0xFF19:
    value = apu->squ_chan[1].s_nx_start[addr - 0xFF15];
    break;
  case 0xFF1A:
  case 0xFF1B:
  case 0xFF1C:
  case 0xFF1D:
  case 0xFF1E:
    value = apu->wav_chan.w_nx_start[addr - 0xFF1A];
    break;
  case 0xFF1F:
  case 0xFF20:
  case 0xFF21:
  case 0xFF22:
  case 0xFF23:
    value = apu->noi_chan.n_nx_start[addr - 0xFF1F];
    break;
  case 0xFF30: /* AUD3WAVERAM */
  case 0xFF31: /* AUD3WAVERAM */
  case 0xFF32: /* AUD3WAVERAM */
  case 0xFF33: /* AUD3WAVERAM */
  case 0xFF34: /* AUD3WAVERAM */
  case 0xFF35: /* AUD3WAVERAM */
  case 0xFF36: /* AUD3WAVERAM */
  case 0xFF37: /* AUD3WAVERAM */
  case 0xFF38: /* AUD3WAVERAM */
  case 0xFF39: /* AUD3WAVERAM */
  case 0xFF3A: /* AUD3WAVERAM */
  case 0xFF3B: /* AUD3WAVERAM */
  case 0xFF3C: /* AUD3WAVERAM */
  case 0xFF3D: /* AUD3WAVERAM */
  case 0xFF3E: /* AUD3WAVERAM */
  case 0xFF3F: /* AUD3WAVERAM */
    value = apu->wav_chan.pcm4_NR30_3F[addr - 0xFF30];
    break;
  case 0xFF24:
    value = apu->reg24_NR50;
    break;
  case 0xFF25:
    value = apu->reg25_NR51;
    break;
  case 0xFF26:
    value = apu->reg26_NR52;
    break;
  default:
    value = apu->gb->unknow_ram[addr]; /* for sound rom test */
    break;
  }
  if (addr <= 0xFF30)
    value |= regm[addr - 0xFF10];
  return value;
}

static 
const uint8_t square_lut[4][8] = {
 { 0, 0, 0, 0, 0, 0, 0, 1 }, /* 00: 12.5% ( _-------_-------_------- ) */
 { 1, 0, 0, 0, 0, 0, 0, 1 },		/* 01: 25%   ( __------__------__------ ) */
 { 1, 0, 0, 0, 0, 1, 1, 1 },		/* 10: 50%   ( ____----____----____---- ) (normal) */
 { 0, 1, 1, 1, 1, 1, 1, 0 }		/* 11: 75%   ( ______--______--______-- ) */
};


static 
void ticks (struct apu *apu) {
  
  int id;
  bool chan_enT[4];

  if (!(apu->reg26_NR52 & NR52_SOUND_CIRCUITS_OPEN_MASK)) {
    /* no sound happening. */
    apu->samGen += apu->gb->cpu_clks_moment;
    while (apu->samGen > apu->samGenT) {
      if (apu->sam_type == APU_SAMPLE_TYPE_WORD) {
        apu->poll_bank.pcm_buf16[apu->sam].left = 0;
        apu->poll_bank.pcm_buf16[apu->sam].right = 0;
      } else if (apu->sam_type == APU_SAMPLE_TYPE_FLOAT) {
        apu->poll_bank.pcm_buff[apu->sam].left = 0.0;
        apu->poll_bank.pcm_buff[apu->sam].right = 0.0;
      } else if (apu->sam_type == APU_SAMPLE_TYPE_BYTE) {
        apu->poll_bank.pcm_buf8[apu->sam].left = 0;
        apu->poll_bank.pcm_buf8[apu->sam].right = 0;
      } else {
        assert (0);
      }
      apu->sam++;
      apu->samGen -= apu->samGenT;
      if (apu->sam >= apu->samT) {
        /*
         * post pcm buffer to host sound drv
         */
        struct apu_framebuffer snd;
        snd.buf = apu->poll_bank.pcm_buf;
        snd.sam = apu->sam_freq;
        snd.len = apu->samT;
        snd.id = apu->pcmbank % APU_BUFFER_BUMS;

        apu->sound_hostdrv (apu, apu->sound_drvobj, & snd);
        apu->pcmbank++;
        apu->pcmbank%= APU_BUFFER_BUMS;
        apu->sam = 0;
        apu->poll_bank.pcm_buf8r = & apu->base_bank.pcm_buf8r[apu->pcmbank*apu->pcmbank_size*apu->samBitDepth/8*apu->pcm_channel];
      }
    }
  } else {
    /* --------------------    Clock Divider   -------------------------- */
    apu->fseq.fseq += apu->gb->cpu_clks_moment;
    apu->samGen += apu->gb->cpu_clks_moment;
    apu->squ_chan[0].squGen += apu->gb->cpu_clks_moment;
    apu->squ_chan[1].squGen += apu->gb->cpu_clks_moment;   
    apu->wav_chan.wavGen += apu->gb->cpu_clks_moment;     
    apu->noi_chan.noiGen += apu->gb->cpu_clks_moment;     
    /* ------------------- Update Frame Sequencer ---------------------- */ 
    while (apu->fseq.fseq > apu->fseq.fseqT) {
      apu->fseq.fseq -= apu->fseq.fseqT;
      switch (apu->fseq.fseq_div) {
      case 6:
      case 2:
        apu->squ_chan[0].seq.sweep_divc++;
      case 0:
      case 4:
        apu->squ_chan[0].seq.len_divc++;
        apu->squ_chan[1].seq.len_divc++;
        apu->wav_chan.seq.len_divc++;
        apu->noi_chan.seq.len_divc++;
      case 1:
      case 3:
      case 5:
        break;
      case 7:
        apu->squ_chan[0].seq.env_divc++;
        apu->squ_chan[1].seq.env_divc++;
        apu->noi_chan.seq.env_divc++;
        break;
      default:
        assert (0);
      }
      apu->fseq.fseq_div++;
      apu->fseq.fseq_div &= 7;
    }
    /* ------------------- Square Channel Render ---------------------- */
    for (id = 0; id != 2; id++) {
      struct square_channel *chan = & apu->squ_chan[id];
      if ( (chan_enT[id] = chan->chan_en) != false) {     
        /* check square wave generate */
        while (chan->squGen > chan->squGen_T) {
          chan->squGen -= chan->squGen_T;
          chan->signal = square_lut[chan->s_dutylen >> 6][chan->squPhase];
          chan->squPhase++;
          chan->squPhase &= 7;
        }
        /* check square volume envlope */
        if (chan->envlope_en != false) {
          while (chan->seq.env_divc >= chan->env_div) {
            chan->seq.env_divc -= chan->env_div;
            chan->vol_render += (chan->s_envlope & NRX2_ENVLOPE_INC_MASK) ? 1 : -1;
            if (chan->vol_render & 0x80)
              chan->vol_render = 0;
            else if (chan->vol_render > 15)
              chan->vol_render = 15;
          }
        }
        /* check square length */
        while (chan->seq.len_divc > 0) {
          chan->seq.len_divc --;     
          if (chan->vol_len != 0)
            chan->vol_len--;
          if (chan->vol_len == 0) {
            /* wave length burning out, check consecutive selection */
            if (chan->squReg_NRX0_X4[4] & NRX4_DIS_CONSECUTIVE_MASK) {
              chan->chan_en = false;
              if (id == 0) apu->squ_on = 0;
              else  apu->squ2_on = 0;
            } else {
              /* reload length */
              chan->vol_len = chan->vol_lenT;
            }
          }
        }
        /* check freq sweep */
        if (id == 0 && (chan->sweep_en != false)) {
          while (chan->seq.sweep_divc >= chan->sweep_div) {
            chan->seq.sweep_divc -= chan->sweep_div;
            chan->shadow2 = chan->freqT2 / (double)(1 << (chan->squReg_NRX0_X4[0] & 7));
            chan->shadow = (uint32_t) chan->shadow2;
            if (chan->squReg_NRX0_X4[0] & NR10_FREQ_SWEEP_SUB_MASK) 
              chan->freqT2 -= chan->shadow2;
            else 
              chan->freqT2 += chan->shadow2;   
            if (cmp0_double (& chan->freqT2) != CMP0_ABOVE) 
              chan->freqT2 = 0.0000000999999;
            else if (chan->freqT2 > 2047.788888)
#           if 0 /* maybe dis channel ..*/
#           else 
              chan->freqT2 = 2047.0;
#           endif    
            chan->squGen_T = apu->gb->mach_tools->cpu_freq;
            chan->squGen_T/= (131072.0 / (2048.0 - chan->freqT2));
          }
        }
      }

    } 
    /* ------------------- Wave Channel Render ---------------------- */
    if ( (chan_enT[2] = apu->wav_chan.chan_en) != false) {
      /* check wave generate */
      while (apu->wav_chan.wavGen > apu->wav_chan.wavGen_T) {
        apu->wav_chan.wavGen -= apu->wav_chan.wavGen_T;
        apu->wav_chan.vol_cur = apu->wav_chan.pcm4_T[apu->wav_chan.wavPhase];
        apu->wav_chan.vol_cur &= 0xF;
        apu->wav_chan.wavPhase++;
        apu->wav_chan.wavPhase &= 0x1F;
      }
      /* check wave length */
      while (apu->wav_chan.seq.len_divc > 0) {
        apu->wav_chan.seq.len_divc--;     
        if (apu->wav_chan.vol_len != 0)
          apu->wav_chan.vol_len--;
        if (apu->wav_chan.vol_len == 0) {
          /* wave length burning out, check consecutive selection */
          if (apu->wav_chan.w_freqhi_rst & NRX4_DIS_CONSECUTIVE_MASK) {
            apu->wav_chan.chan_en = false;
            apu->wav_on = false;
          } else {
            /* reload length */
            apu->wav_chan.vol_len = apu->wav_chan.vol_lenT;
          }
        }
      }
    } else apu->wav_chan.vol_cur = 0;

    /* ------------------- Noise Channel Render ---------------------- */
    if ( (chan_enT[3] = apu->noi_chan.chan_en) != false) {
      /* check random noise generate */
      while (apu->noi_chan.noiGen > apu->noi_chan.noiGen_T) {     
        apu->noi_chan.lfsrSeed = (apu->noi_chan.n_sftdiv_lfsr & 0x08) ? lfsr7 (apu->noi_chan.lfsrSeed & 0x7F) 
           : lfsr15 (apu->noi_chan.lfsrSeed & 0x7FFF);
        apu->noi_chan.signal = apu->noi_chan.lfsrSeed & 1;
        apu->noi_chan.noiGen -= apu->noi_chan.noiGen_T;
      }
      /* check noise length */
      while (apu->noi_chan.seq.len_divc > 0) {
        apu->noi_chan.seq.len_divc --;     
        if (apu->noi_chan.vol_len != 0)
          apu->noi_chan.vol_len--;
        if (apu->noi_chan.vol_len == 0) {
          /* wave length burning out, check consecutive selection */
          if (apu->noi_chan.noiReg_NRX0_X4[4] & NRX4_DIS_CONSECUTIVE_MASK) {
            apu->noi_chan.chan_en = false;
            apu->noi_on = 0;
          } else {
            /* reload length */
            apu->noi_chan.vol_len = apu->noi_chan.vol_lenT;
          }
        }
      }
      /* check noise volume envlope */
      if (apu->noi_chan.envlope_en != false) {
        while (apu->noi_chan.seq.env_divc >= apu->noi_chan.env_div) {
          apu->noi_chan.seq.env_divc -= apu->noi_chan.env_div;
          apu->noi_chan.vol_render += (apu->noi_chan.noiReg_NRX0_X4[2] & NRX2_ENVLOPE_INC_MASK) ? 1 : -1;
          if (apu->noi_chan.vol_render & 0x80)
            apu->noi_chan.vol_render = 0;
          else if (apu->noi_chan.vol_render > 15)
            apu->noi_chan.vol_render = 15;
        }
      }
    }

    /* ------------------- Mixer ---------------------- */
    while (apu->samGen > apu->samGenT) {
      if (apu->sam_type == APU_SAMPLE_TYPE_WORD) {
        /* Synthesizer- 16bit PCM 
         *             Square1 Square2 Wave Noise 
         * Amplifier       
         */
        int16_t squ_vol[2] = { 0, 0 };
        int16_t wav_vol = 0;
        int16_t noi_vol = 0;
        int16_t left_out = 0;
        int16_t right_out = 0;
        int16_t volume;
        if (chan_enT[2] != false) {
          wav_vol = apu->wav_chan.vol_cur >> (apu->wav_chan.vol_sft >> 1);
          wav_vol = dac_quant16_16bitpcm[wav_vol]/4;
          if (apu->wav_chan.vol_sft >= 4)
            wav_vol = 0;
          else ;
        }                   
        if (chan_enT[0] != false) {
          squ_vol[0] = apu->squ_chan[0].signal * apu->squ_chan[0].vol_render;
          squ_vol[0] = dac_quant16_16bitpcm[squ_vol[0]]/4;
        } 
        if (chan_enT[1] != false) {
          squ_vol[1] = apu->squ_chan[1].signal * apu->squ_chan[1].vol_render;
          squ_vol[1] = dac_quant16_16bitpcm[squ_vol[1]]/4;
        } 
        if (chan_enT[3] != false) {
          noi_vol = apu->noi_chan.signal * apu->noi_chan.vol_render;
          noi_vol = dac_quant16_16bitpcm[noi_vol]/4;
        } 
        /* Check Output SO2 Volume */
        volume = apu->so2_outlevel + 1;
        /* check channel out */
        if (apu->so2out_squ) left_out = AddSaturaItSign16 (left_out, (squ_vol[0] * volume)/ 8);
        if (apu->so2out_squ2) left_out = AddSaturaItSign16 (left_out, (squ_vol[1] * volume)/ 8);
        if (apu->so2out_wave) left_out = AddSaturaItSign16 (left_out, (wav_vol * volume)/ 8);
        if (apu->so2out_noi) left_out = AddSaturaItSign16 (left_out, (noi_vol * volume)/ 8); 
        apu->poll_bank.pcm_buf16[apu->sam].left = left_out;

        volume = apu->so1_outlevel + 1;
        /* check channel out */
        if (apu->so1out_squ) right_out = AddSaturaItSign16 (right_out, (squ_vol[0] * volume)/ 8);
        if (apu->so1out_squ2) right_out = AddSaturaItSign16 (right_out, (squ_vol[1] * volume)/ 8);
        if (apu->so1out_wave) right_out = AddSaturaItSign16 (right_out, (wav_vol * volume)/8);
        if (apu->so1out_noi) right_out = AddSaturaItSign16 (right_out, (noi_vol * volume)/ 8); 
        apu->poll_bank.pcm_buf16[apu->sam].right = right_out;
      } else if (apu->sam_type == APU_SAMPLE_TYPE_FLOAT) {
        /* Synthesizer- float PCM
         */
        int squ_vol[2] = { 0, 0 };
        int wav_vol = 0;
        int noi_vol = 0;
        float squ_vol_[2] = { 0.0, 0.0 };
        float wav_vol_ = 0.0;
        float noi_vol_ = 0.0;
        float left_out = 0.0;
        float right_out = 0.0;
        float volume;
        if (chan_enT[2] != false) {
          wav_vol = apu->wav_chan.vol_cur >> (apu->wav_chan.vol_sft >> 1);
          wav_vol_ = dac_quant16_fbitpcmt[wav_vol]/4.0;
          if (apu->wav_chan.vol_sft >= 4)
            wav_vol = 0.0;
          else ;
        }                   
        if (chan_enT[0] != false) {
          squ_vol[0] = apu->squ_chan[0].signal * apu->squ_chan[0].vol_render;
          squ_vol_[0] = dac_quant16_fbitpcmt[squ_vol[0]]/4.0;
        } 
        if (chan_enT[1] != false) {
          squ_vol[1] = apu->squ_chan[1].signal * apu->squ_chan[1].vol_render;
          squ_vol_[1] = dac_quant16_fbitpcmt[squ_vol[1]]/4.0;
        } 
        if (chan_enT[3] != false) {
          noi_vol = apu->noi_chan.signal * apu->noi_chan.vol_render;
          noi_vol_ = dac_quant16_fbitpcmt[noi_vol]/4.0;
        } 
        /* Check Output SO2 Volume */
        volume = (float)(apu->so2_outlevel + 1);
        /* check channel out */
        if (apu->so2out_squ) left_out = AddSaturaItFloat1 (left_out, (squ_vol_[0] * volume)/ 8.0);
        if (apu->so2out_squ2) left_out = AddSaturaItFloat1 (left_out, (squ_vol_[1] * volume)/ 8.0);
        if (apu->so2out_wave) left_out = AddSaturaItFloat1 (left_out, (wav_vol_ * volume)/ 8.0);
        if (apu->so2out_noi) left_out = AddSaturaItFloat1 (left_out, (noi_vol_ * volume)/ 8.0); 
        apu->poll_bank.pcm_buff[apu->sam].left = left_out;

        volume = (float)(apu->so1_outlevel + 1);
        /* check channel out */
        if (apu->so1out_squ) right_out = AddSaturaItFloat1 (right_out, (squ_vol_[0] * volume)/ 8.0);
        if (apu->so1out_squ2) right_out = AddSaturaItFloat1 (right_out, (squ_vol_[1] * volume)/ 8.0);
        if (apu->so1out_wave) right_out = AddSaturaItFloat1 (right_out, (wav_vol_ * volume)/8.0);
        if (apu->so1out_noi) right_out = AddSaturaItFloat1 (right_out, (noi_vol_ * volume)/ 8.0); 
        apu->poll_bank.pcm_buff[apu->sam].right = right_out;
      } else if (apu->sam_type == APU_SAMPLE_TYPE_BYTE) {
        /* Synthesizer- float PCM
         */
        uint8_t squ_vol[2] = { 0, 0 };
        uint8_t wav_vol = 0;
        uint8_t noi_vol = 0;
        uint8_t left_out = 0;
        uint8_t right_out = 0;
        uint8_t volume;
        if (chan_enT[2] != false) {
          wav_vol = apu->wav_chan.vol_cur >> (apu->wav_chan.vol_sft >> 1);
          wav_vol = wav_vol*4;
          if (apu->wav_chan.vol_sft >= 4)
            wav_vol = 0;
          else ;
        }                   
        if (chan_enT[0] != false) {
          squ_vol[0] = apu->squ_chan[0].signal * apu->squ_chan[0].vol_render;
          squ_vol[0] = squ_vol[0]*4;
        } 
        if (chan_enT[1] != false) {
          squ_vol[1] = apu->squ_chan[1].signal * apu->squ_chan[1].vol_render;
          squ_vol[1] = squ_vol[1]*4;
        } 
        if (chan_enT[3] != false) {
          noi_vol = apu->noi_chan.signal * apu->noi_chan.vol_render;
          noi_vol = noi_vol*4;
        } 
        /* Check Output SO2 Volume */
        volume = apu->so2_outlevel + 1;
        /* check channel out */
        if (apu->so2out_squ) left_out = AddSaturaItUSign8 (left_out, (squ_vol[0] * volume)/ 8);
        if (apu->so2out_squ2) left_out = AddSaturaItUSign8 (left_out, (squ_vol[1] * volume)/ 8);
        if (apu->so2out_wave) left_out = AddSaturaItUSign8 (left_out, (wav_vol * volume)/ 8);
        if (apu->so2out_noi) left_out = AddSaturaItUSign8 (left_out, (noi_vol * volume)/ 8); 
        apu->poll_bank.pcm_buf8[apu->sam].left = left_out;

        volume = apu->so1_outlevel + 1;
        /* check channel out */
        if (apu->so1out_squ) right_out = AddSaturaItUSign8 (right_out, (squ_vol[0] * volume)/ 8);
        if (apu->so1out_squ2) right_out = AddSaturaItUSign8 (right_out, (squ_vol[1] * volume)/ 8);
        if (apu->so1out_wave) right_out = AddSaturaItUSign8 (right_out, (wav_vol * volume)/8);
        if (apu->so1out_noi) right_out = AddSaturaItUSign8 (right_out, (noi_vol * volume)/ 8); 
        apu->poll_bank.pcm_buf8[apu->sam].right = right_out;
      } else {
        assert (0);
      }
      apu->sam++;
      apu->samGen -= apu->samGenT;
      if (apu->sam >= apu->samT) {
        /*
         * post pcm buffer to host sound drv
         */
        struct apu_framebuffer snd;
        snd.buf = apu->poll_bank.pcm_buf;
        snd.sam = apu->sam_freq;
        snd.len = apu->samT;
        snd.id = apu->pcmbank % APU_BUFFER_BUMS;
        // DEBUG_OUT ("sam:%d samT:%d old bank:%d\n", apu->sam, apu->samT, apu->pcmbank);

        apu->sound_hostdrv (apu, apu->sound_drvobj, & snd);
        apu->pcmbank++;
        apu->pcmbank%= APU_BUFFER_BUMS;
        apu->sam = 0;
        apu->poll_bank.pcm_buf8r = & apu->base_bank.pcm_buf8r[apu->pcmbank*apu->pcmbank_size*apu->samBitDepth/8*apu->pcm_channel];
        
        // memset (apu->pcm_buf[apu->pcmbank], 0, APU_POST_RENDER_WHEN_SAMPLE_N * 4);
      }
    }
  }
}



void apu_reset_sample_type (struct apu *apu, APU_SAMPLE_TYPE sample_type) {

  /* reset buffer and ctx. */
  apu->sam_type = sample_type;
  /* clear pcm buffer */
  memset (apu->base_bank.pcm_buf, 0, apu->pcmringbuf_size);
  apu->samT = APU_BUFFER_CHUNK_SIZE_BASE_SAMPLE;
  apu->sam = 0;
  apu->samGen = 0.0;
  apu->pcmbank = 0;
  apu->pcm_channel = 2;
  apu->poll_bank.pcm_buf = apu->base_bank.pcm_buf;
  apu->samBitDepth = (sample_type == APU_SAMPLE_TYPE_FLOAT) ? 32 : (sample_type == APU_SAMPLE_TYPE_WORD) ? 16 : 8;
}

void apu_reset_sample_freq (struct apu *apu, APU_SAMPLE_FREQ sample_freq) {

  /* reset buffer and ctx. */
  apu->sam_freq = sample_freq;
  /* clear pcm buffer */
  memset (apu->base_bank.pcm_buf, 0, apu->pcmringbuf_size);
  
  apu->sam = 0;
  apu->pcm_channel = 2;
  apu->pcmbank = 0;
  apu->poll_bank.pcm_buf = apu->base_bank.pcm_buf;
  // apu->samGenT = apu->gb->mach_tools->cpu_freq / (double) apu->sam_freq;
  apu->samGenT = std_machine.cpu_freq / (double) apu->sam_freq;
  apu->samGen = 0.0;
}

int apu_init (struct apu **apu) {

  struct apu *apu_ = null;
  assert (apu != null);

  apu_ = (struct apu *)
     calloc (sizeof (struct apu), 1);
  apu_->clks = ticks;
  apu_->samT =
  apu_->pcmbank_size = APU_BUFFER_CHUNK_SIZE_BASE_SAMPLE;
  apu_->pcmringbuf_size = apu_->pcmbank_size*APU_SAMPLE_CHANNEL_NUMS*APU_SAMPLE_SBIT_MAX*APU_BUFFER_BUMS;
  /* build sampe buffer (simple ring buffer - (not MT's ringbuffer ^_-) */
  apu_->base_bank.pcm_buf = calloc (apu_->pcmringbuf_size, 1);
  apu_->poll_bank.pcm_buf = apu_->base_bank.pcm_buf;

  apu_reset_sample_type (apu_, APU_SAMPLE_TYPE_DEFAULT);
  apu_reset_sample_freq (apu_, APU_SAMPLE_FREQ_DEFAULT);

  // apu_->fseq.fseqT /= 5.0;

  assert (apu_ != null);
  // apu_reset (apu_);
  * apu = apu_;
  return 0;
}


void apu_uninit (struct apu **apu) {
  struct apu *apu_;
  assert (apu != null);
  apu_ = *apu;
  *apu = null;
  if (apu_ != null)
    free (apu_);
  else ;
}

int apu_getbank_size_inbyte (struct apu *apu) {
  return (apu->pcmbank_size*apu->samBitDepth/8*apu->pcm_channel);
}
int apu_getbank_size_insample (struct apu *apu) {
  return (apu->pcmbank_size);
}
int apu_get_sample_bit_depth (struct apu *apu) {
  return apu->samBitDepth;
}