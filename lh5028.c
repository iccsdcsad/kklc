/* Game boy's GPU and LCD Screen
 * LCD is Sharp LH5028 http://www.datasheetarchive.com/pdf/download.php?id=c615e5d8551c6b559c3db61b709b3234af856c&type=O&term=LH5028
 * 
 * Copyright (C) 2018 moecmks
 * This file is part of YuduliyaGB.
 * 
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 */

#include "gameboy.h"
#include "inline.inl"

static struct ppu_dmg_palette16 dmgcol_white; 
static struct ppu_dmg_palette16 dmgcol_brown;
static struct ppu_dmg_palette16 dmgcol_cyan;

void dmgpal_init (void) {
  /* init white palette */
  dmgcol_white.bg_pal[0] = dmgcol_white.sp_pal[0][0] = dmgcol_white.sp_pal[1][0] = (0x1F << 0) | (0x1F << 5) | (0x1F << 10);
  dmgcol_white.bg_pal[1] = dmgcol_white.sp_pal[0][1] = dmgcol_white.sp_pal[1][1] = (0x15 << 0) | (0x15 << 5) | (0x15 << 10);
  dmgcol_white.bg_pal[2] = dmgcol_white.sp_pal[0][2] = dmgcol_white.sp_pal[1][2] = (0x0A << 0) | (0x0A << 5) | (0x0A << 10);
  dmgcol_white.bg_pal[3] = dmgcol_white.sp_pal[0][3] = dmgcol_white.sp_pal[1][3] = (0x00 << 0) | (0x00 << 5) | (0x00 << 10);
  /* init brown palette */
  dmgcol_brown.bg_pal[0] = dmgcol_brown.sp_pal[0][0] = dmgcol_brown.sp_pal[1][0] = (0x8c/8 << 0) | (0xe8/8 << 5) | (0xfc/8 << 10);
  dmgcol_brown.bg_pal[1] = dmgcol_brown.sp_pal[0][1] = dmgcol_brown.sp_pal[1][1] = (0x5c/8 << 0) | (0xb4/8 << 5) | (0xdc/8 << 10);
  dmgcol_brown.bg_pal[2] = dmgcol_brown.sp_pal[0][2] = dmgcol_brown.sp_pal[1][2] = (0x3c/8 << 0) | (0x7c/8 << 5) | (0x98/8 << 10);
  dmgcol_brown.bg_pal[3] = dmgcol_brown.sp_pal[0][3] = dmgcol_brown.sp_pal[1][3] = (0x1c/8 << 0) | (0x3c/8 << 5) | (0x4c/8 << 10);
  /* init cyan palette */
  dmgcol_cyan.bg_pal[0] = dmgcol_cyan.sp_pal[0][0] = dmgcol_cyan.sp_pal[1][0] = (0xe0/8 << 0) | (0xf8/8 << 5) | (0xd0/8 << 10);
  dmgcol_cyan.bg_pal[1] = dmgcol_cyan.sp_pal[0][1] = dmgcol_cyan.sp_pal[1][1] = (0x95/8 << 0) | (0xad/8 << 5) | (0x98/8 << 10);
  dmgcol_cyan.bg_pal[2] = dmgcol_cyan.sp_pal[0][2] = dmgcol_cyan.sp_pal[1][2] = (0x5a/8 << 0) | (0x62/8 << 5) | (0x50/8 << 10);
  dmgcol_cyan.bg_pal[3] = dmgcol_cyan.sp_pal[0][3] = dmgcol_cyan.sp_pal[1][3] = (0x20/8 << 0) | (0x18/8 << 5) | (0x08/8 << 10);
}

static 
void default_update (struct ppu *_5028, void *obj, struct ppu_framebuffer *fbuf) {
  DEBUG_OUT ("%s:%d please set ppu_update callback( controller_setupdate function)\n", __FILE__, __LINE__);
  DEBUG_BREAK();
}

void ppu_setupdate_ (struct ppu *lh5028, void (*update) 
            (struct ppu *, 
            void *, 
              struct ppu_framebuffer *), void *obj)
{
  lh5028->device_blit = update;
  lh5028->obj = obj;
}

int ppu_reset (struct ppu *lh5028) {
  
  /* init register http://gbdev.gg8.se/files/docs/mirrors/pandocs.html#powerupsequence */
  lh5028->reg40_LCDC = 0x91;
  lh5028->reg42_SCY = 0x00;
  lh5028->reg43_SCX = 0x00;
  lh5028->reg45_LYC = 0x00;
  lh5028->reg47_BGP = 0xFC;
  lh5028->reg48_OBP0 = 0xFF;
  lh5028->reg49_OBP1 = 0xFF;
  lh5028->reg4A_WY = 0x00;
  lh5028->reg4B_WX = 0x00;
  /* reset cache */
  lh5028->reg41_LCDM_T = lh5028->reg41_LCDS;

  lh5028->hbl_clks_st = 9999.8;
  return 0;
}

finline 
uint32_t make_rgb15_torgb32 (uint16_t rgb15) {

  uint32_t rt1 = (rgb15 & (0x1F << 0)) >> 0;
  uint32_t rt2 = (rgb15 & (0x1F << 5)) >> 5;
  uint32_t rt3 = (rgb15 & (0x1F <<10)) >>10;
  uint32_t out;
  rt1 <<= 3;
  rt2 <<= 3;
  rt3 <<= 3;
  out = rt1 | (rt2 << 8) | (rt3 << 16);
  return out;
}

finline 
uint16_t make_rgb32_torgb15 (uint32_t rgb32) {

  uint16_t rt1 = (rgb32 & (0xFF << 0)) >> 0;
  uint16_t rt2 = (rgb32 & (0xFF << 8)) >> 8;
  uint16_t rt3 = (rgb32 & (0xFF <<16)) >>16;
  uint16_t out;
  rt1 >>= 3;
  rt2 >>= 3;
  rt3 >>= 3;
  out = rt1 | (rt2 << 5) | (rt3 << 10);
  return out;
}

finline 
uint16_t make_rgb15_rev (uint16_t rgb15) {

  uint32_t rt1 = (rgb15 & (0x1F << 0)) >> 0;
  uint32_t rt2 = (rgb15 & (0x1F << 5)) >> 5;
  uint32_t rt3 = (rgb15 & (0x1F <<10)) >>10;
  uint32_t out;
  out = rt3 | (rt2 << 5) | (rt1 << 10);
  return out;
}

static 
void set_dmgpal32_bydmgpal16 (struct ppu *lh5028) {
  /* update dmg pal */
  int id;
  for (id = 0; id != 4; id++) {
    lh5028->dmg_pal32.bg_pal[id] =
            make_rgb15_torgb32 (lh5028->dmg_pal16.bg_pal[id]);
    lh5028->dmg_pal32.sp_pal[0][id] =
            make_rgb15_torgb32 (lh5028->dmg_pal16.sp_pal[0][id]);
    lh5028->dmg_pal32.sp_pal[1][id] =
            make_rgb15_torgb32 (lh5028->dmg_pal16.sp_pal[1][id]);          
  }
}
static 
void set_dmgpal16_bydmgpal32 (struct ppu *lh5028) {
  /* update dmg pal */
  int id;
  for (id = 0; id != 4; id++) {
    lh5028->dmg_pal16.bg_pal[id] =
            make_rgb32_torgb15 (lh5028->dmg_pal32.bg_pal[id]);
    lh5028->dmg_pal16.sp_pal[0][id] =
            make_rgb32_torgb15 (lh5028->dmg_pal32.sp_pal[0][id]);
    lh5028->dmg_pal16.sp_pal[1][id] =
            make_rgb32_torgb15 (lh5028->dmg_pal32.sp_pal[1][id]);          
  }
}

static 
void set_cgbpal32_bycgbpal16 (struct ppu *lh5028) {
  /* update cgb pal */
  int d, q;
  for (q = 0; q != 4; q++) {
    for (d = 0; d != 8; d++) {
    lh5028->cgb_bg_pal32[d][q].rgb32 =
            make_rgb15_torgb32 (lh5028->cgb_bg_pal16[d][q].rgb15);
        lh5028->cgb_sp_pal32[d][q].rgb32 =
            make_rgb15_torgb32 (lh5028->cgb_sp_pal16[d][q].rgb15);  
    lh5028->cgb_bg_pal32_x0r8g8b8[d][q].rgb32 =
            make_rgb15_torgb32 (lh5028->cgb_bg_pal16_x1r5g5b5[d][q].rgb15);
        lh5028->cgb_sp_pal32_x0r8g8b8[d][q].rgb32 =
            make_rgb15_torgb32 (lh5028->cgb_sp_pal16_x1r5g5b5[d][q].rgb15);  

    }
  }
}
static void update_dmgpal_full (struct ppu *lh5028) {

  lh5028->dmg_pal16.bg_pal[0] = lh5028->dmg_pal16T.bg_pal[(lh5028->reg47_BGP & 0x03) >> 0];
  lh5028->dmg_pal16.bg_pal[1] = lh5028->dmg_pal16T.bg_pal[(lh5028->reg47_BGP & 0x0C) >> 2];
  lh5028->dmg_pal16.bg_pal[2] = lh5028->dmg_pal16T.bg_pal[(lh5028->reg47_BGP & 0x30) >> 4];
  lh5028->dmg_pal16.bg_pal[3] = lh5028->dmg_pal16T.bg_pal[(lh5028->reg47_BGP & 0xC0) >> 6];
  lh5028->dmg_pal16.sp_pal[0][0] = lh5028->dmg_pal16T.sp_pal[0][(lh5028->reg48_OBP0 & 0x03) >> 0];
  lh5028->dmg_pal16.sp_pal[0][1] = lh5028->dmg_pal16T.sp_pal[0][(lh5028->reg48_OBP0 & 0x0C) >> 2];
  lh5028->dmg_pal16.sp_pal[0][2] = lh5028->dmg_pal16T.sp_pal[0][(lh5028->reg48_OBP0 & 0x30) >> 4];
  lh5028->dmg_pal16.sp_pal[0][3] = lh5028->dmg_pal16T.sp_pal[0][(lh5028->reg48_OBP0 & 0xC0) >> 6];
  lh5028->dmg_pal16.sp_pal[1][0] = lh5028->dmg_pal16T.sp_pal[1][(lh5028->reg49_OBP1 & 0x03) >> 0];
  lh5028->dmg_pal16.sp_pal[1][1] = lh5028->dmg_pal16T.sp_pal[1][(lh5028->reg49_OBP1 & 0x0C) >> 2];
  lh5028->dmg_pal16.sp_pal[1][2] = lh5028->dmg_pal16T.sp_pal[1][(lh5028->reg49_OBP1 & 0x30) >> 4];
  lh5028->dmg_pal16.sp_pal[1][3] = lh5028->dmg_pal16T.sp_pal[1][(lh5028->reg49_OBP1 & 0xC0) >> 6];

  set_dmgpal32_bydmgpal16 (lh5028);
}

void ppu_uninit (struct ppu **lh5028);

void ppu_write (struct ppu *lh5028, uint16_t address, uint8_t value) {

  uint8_t c, s;
  switch (address) {
  case 0xFF40: /* FF40 - LCDC - LCD Control (R/W) **/
    /*Bit 7 - LCD Display Enable             (0=Off, 1=On)
      Bit 6 - Window Tile Map Display Select (0=9800-9BFF, 1=9C00-9FFF)
      Bit 5 - Window Display Enable          (0=Off, 1=On)
      Bit 4 - BG & Window Tile Data Select   (0=8800-97FF, 1=8000-8FFF)
      Bit 3 - BG Tile Map Display Select     (0=9800-9BFF, 1=9C00-9FFF)
      Bit 2 - OBJ (Sprite) Size              (0=8x8, 1=8x16)
      Bit 1 - OBJ (Sprite) Display Enable    (0=Off, 1=On)
      Bit 0 - BG/Window Display/Priority     (0=Off, 1=On)
     */
    if (!(lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)&& (value & LCDC_DISPLAY_MASK)) {
      /* re-enable display */
#   if 0
      lh5028->gb->cpu_clks_ppu = 0.0;
      lh5028->reg41_LCDM_T =LCDS_MODE_FLAG_SERACH_OAM;
      lh5028->reg41_IRQf |= LCDS_INTERRUPET_ALL_MASK;
      lh5028->reg41_LCDS &= ~3;
      lh5028->reg41_LCDS |= LCDS_MODE_FLAG_SERACH_OAM;
      lh5028->reg40_NMIf = 1;
      lh5028->vscan = -1;
      lh5028->vscanR = 0;
      lh5028->vscan40 = 0;
      lh5028->uscan = -1;
      lh5028->uscanR = 0;
      lh5028->win_stuff = false;
      lh5028->hbl_clks_st = 9999.8;
      lh5028->reg44_LY = 0;

#   endif 
      lh5028->reg4A_WYRSC = 0;
    }
    if (!(value & LCDC_DISPLAY_MASK))
      lh5028->reg44_LY = 0;
    else ;
    lh5028->reg40_LCDC = value;
    break;
  case 0xFF41: /* FF41 - STAT - LCDC Status (R/W) */
    /*Bit 6 - LYC=LY Coincidence Interrupt (1=Enable) (Read/Write)
      Bit 5 - Mode 2 OAM Interrupt         (1=Enable) (Read/Write)
      Bit 4 - Mode 1 V-Blank Interrupt     (1=Enable) (Read/Write)
      Bit 3 - Mode 0 H-Blank Interrupt     (1=Enable) (Read/Write)
      Bit 2 - Coincidence Flag  (0:LYC<>LY, 1:LYC=LY) (Read Only)
      Bit 1-0 - Mode Flag                             (Read Only)
              0: During H-Blank
              1: During V-Blank
              2: During Searching OAM
              3: During Transferring Data to LCD Driver
    */
    lh5028->reg41_LCDS &= 7;
    lh5028->reg41_LCDS |= (value & ~7);
    break;
  case 0xFF42: /* FF42 - SCY - Scroll Y (R/W) */
    lh5028->reg42_SCY = value;
    break;
  case 0xFF43: /* FF43 - SCX - Scroll X (R/W) */
    lh5028->reg43_SCX = value;
    break;
  case 0xFF44: /* FF44 - LY - LCDC Y-Coordinate (R) */
    DEBUG_BREAK();
    break;
  case 0xFF45: /* FF45 - LYC - LY Compare (R/W) */
    lh5028->reg45_LYC = value;
    break;
  case 0xFF4A: /* FF4A - WY - Window Y Position (R/W) */
    lh5028->reg4A_WY = value;
    break;
  case 0xFF4B: /* FF4B - WX - Window X Position minus 7 (R/W) */
    lh5028->reg4B_WX = value;
    break; 
  case 0xFF47: /* FF47 - BGP - BG Palette Data (R/W) - Non CGB Mode Only */
    lh5028->dmg_pal16.bg_pal[0] = lh5028->dmg_pal16T.bg_pal[(value & 0x03) >> 0];
    lh5028->dmg_pal16.bg_pal[1] = lh5028->dmg_pal16T.bg_pal[(value & 0x0C) >> 2];
    lh5028->dmg_pal16.bg_pal[2] = lh5028->dmg_pal16T.bg_pal[(value & 0x30) >> 4];
    lh5028->dmg_pal16.bg_pal[3] = lh5028->dmg_pal16T.bg_pal[(value & 0xC0) >> 6];
    lh5028->reg47_BGP = value;
    set_dmgpal32_bydmgpal16 (lh5028);
    break ;
  case 0xFF48: /* FF48 - OBP0 - Object Palette 0 Data (R/W) - Non CGB Mode Only */
    lh5028->dmg_pal16.sp_pal[0][0] = lh5028->dmg_pal16T.sp_pal[0][(value & 0x03) >> 0];
    lh5028->dmg_pal16.sp_pal[0][1] = lh5028->dmg_pal16T.sp_pal[0][(value & 0x0C) >> 2];
    lh5028->dmg_pal16.sp_pal[0][2] = lh5028->dmg_pal16T.sp_pal[0][(value & 0x30) >> 4];
    lh5028->dmg_pal16.sp_pal[0][3] = lh5028->dmg_pal16T.sp_pal[0][(value & 0xC0) >> 6];
    lh5028->reg48_OBP0 = value;
    set_dmgpal32_bydmgpal16 (lh5028);
    break ;
  case 0xFF49: /* FF49 - OBP1 - Object Palette 1 Data (R/W) - Non CGB Mode Only */
    lh5028->dmg_pal16.sp_pal[1][0] = lh5028->dmg_pal16T.sp_pal[1][(value & 0x03) >> 0];
    lh5028->dmg_pal16.sp_pal[1][1] = lh5028->dmg_pal16T.sp_pal[1][(value & 0x0C) >> 2];
    lh5028->dmg_pal16.sp_pal[1][2] = lh5028->dmg_pal16T.sp_pal[1][(value & 0x30) >> 4];
    lh5028->dmg_pal16.sp_pal[1][3] = lh5028->dmg_pal16T.sp_pal[1][(value & 0xC0) >> 6];
    lh5028->reg49_OBP1 = value;
    set_dmgpal32_bydmgpal16 (lh5028);
    break ;
  case 0xFF68: /* FF68 - BCPS/BGPI - CGB Mode Only - Background Palette Index */
    lh5028->reg68_BCPS = value;
    break ;
  case 0xFF69: /* FF69 - BCPD/BGPD - CGB Mode Only - Background Palette Data */
    /*x1b5g5r5*/
    if ((c = lh5028->reg68_BCPS & 0x3F) & 1) /*g3r5*/
      lh5028->cgb_bg_pal16[c >> 3][(c & 7) >> 1]._hi = value;
    else /*g5r2*/
      lh5028->cgb_bg_pal16[c >> 3][(c & 7) >> 1]._lo = value;
    /* update x1r5g5b5 palette */
    lh5028->cgb_bg_pal16_x1r5g5b5[c >> 3][(c & 7) >> 1].rgb15 = 
         make_rgb15_rev (lh5028->cgb_bg_pal16[c >> 3][(c & 7) >> 1].rgb15);
    if (lh5028->reg68_BCPS & 0x80) {
      lh5028->reg68_BCPS++;
      lh5028->reg68_BCPS &= 0x3F;
      lh5028->reg68_BCPS |= 0x80;
    }
    for (c = 0; c != 0; c++) {
        DEBUG_OUT ("bg_pal bcp index:%d 0:%02X 1:%02X 2:%02X 3:%02X\n",
          c, lh5028->cgb_bg_pal16[c][0].rgb15,
          lh5028->cgb_bg_pal16[c][1].rgb15,
          lh5028->cgb_bg_pal16[c][2].rgb15,
          lh5028->cgb_bg_pal16[c][3].rgb15);
    }
    lh5028->reg69_BCPD = value;
    set_cgbpal32_bycgbpal16 (lh5028);
    break;
  case 0xFF6A: /* FF6A - OCPS/OBPI - CGB Mode Only - Sprite Palette Index */
    lh5028->reg6A_OCPS = value;
    break;
  case 0xFF6B: /* FF6B - OCPD/OBPD - CGB Mode Only - Sprite Palette Data */
    if ((c = lh5028->reg6A_OCPS & 0x3F) & 1)
      lh5028->cgb_sp_pal16[c >> 3][(c & 7) >> 1]._hi = value;
    else 
      lh5028->cgb_sp_pal16[c >> 3][(c & 7) >> 1]._lo = value;
    /* update x1r5g5b5 palette */
    lh5028->cgb_sp_pal16_x1r5g5b5[c >> 3][(c & 7) >> 1].rgb15 = 
         make_rgb15_rev (lh5028->cgb_sp_pal16[c >> 3][(c & 7) >> 1].rgb15);
    if (lh5028->reg6A_OCPS & 0x80) {
      lh5028->reg6A_OCPS++;
      lh5028->reg6A_OCPS &= 0x3F;
      lh5028->reg6A_OCPS |= 0x80;
    }

    lh5028->reg6B_OCPD = value;
    set_cgbpal32_bycgbpal16 (lh5028);
    break;
  case 0xFF46: /* FF46 - DMA - DMA Transfer and Start Address (R/W) */
    for (c = 0; c != 160; c++) {
      s = gameboy_read (lh5028->gb, value * 256 + c);
      ((int8_t *)& lh5028->sp[0])[c] = s;
    }
    /* OAMDMA ~ 160 us 
       OAMDMA is a parallel DMA */
    /* lh5028->gb->cpu_clks_dma += lh5028->gb->mach_tools->oamdma_clks; */
    lh5028->reg46_DMA = value;
    break ;
  case 0xFF51: /* FF51 - HDMA1 - CGB Mode Only - New DMA Source, High */
    lh5028->reg51_HDMA1 = value;
    break;
  case 0xFF52: /* FF52 - HDMA2 - CGB Mode Only - New DMA Source, Low*/
    lh5028->reg52_HDMA2 = value;
    break;
  case 0xFF53: /* FF53 - HDMA3 - CGB Mode Only - New DMA Destination, High */
    lh5028->reg53_HDMA3 = value;
    break;
  case 0xFF54: /* FF54 - HDMA4 - CGB Mode Only - New DMA Destination, Low*/
    lh5028->reg54_HDMA4 = value;
    break;
  case 0xFF55: /* FF55 - HDMA5 - CGB Mode Only - New DMA Length/Mode/Start */
    if (value & 0x80) {
      /* H-Blank DMA. **/
      lh5028->hdma_src = lh5028->reg51_HDMA1 * 256 + lh5028->reg52_HDMA2;
      lh5028->hdma_dst = lh5028->reg53_HDMA3 * 256 + lh5028->reg54_HDMA4;
      lh5028->hdma_src &= 0xFFF0;
      lh5028->hdma_dst &= 0xFFF0;
      lh5028->hdma_r16 = value & 0x7F;
      lh5028->hdma_r16 ++;
      lh5028->hdma_gen = true;
      /* set HDMA uncompelete/active */
      lh5028->reg55_HDMA5 &= 0x7F;
    } else {
      /* GDMA **/
      
      uint16_t src = lh5028->reg51_HDMA1 * 256 + lh5028->reg52_HDMA2;
      uint16_t dst = lh5028->reg53_HDMA3 * 256 + lh5028->reg54_HDMA4;
      uint16_t id = 0;
      uint16_t K;

      src &= 0xFFF0;
      dst &= 0xFFF0;
      K = value & 0x7F;
      K+= 1;
      /* copy it*/
      for (; id != K; id++) {
        uint16_t c;
        for (c =0; c != 16; c++) {
          s = gameboy_read (lh5028->gb, src + id *16 + c);
          gameboy_write (lh5028->gb, dst + id *16 + c, s);
        }
      }
      /* burning OAM clks */
      /* It takes (220 + (n * 7.63)) microseconds in single speed 
         and (110 + (n * 7.63)) microseconds in double speed mode */
      lh5028->gb->cpu_clks_dma += (lh5028->gb->mach_tools->gdma_clks_b 
        + lh5028->gb->mach_tools->gdma_clks_per16 * (double) K);
      /* lh5028->reg55_HDMA5 |= 0x80;/* set GDMA compelete/unactive */
    }
    lh5028->reg55_HDMA5 = value;
    break;
  case 0xFF4F: /* FF4F - VBK - CGB Mode Only - VRAM Bank (R/W) */
    lh5028->reg4F_VBK = value & 1;
    break;
  default:
    return;
  }
}

uint8_t ppu_read (struct ppu *lh5028, uint16_t address) {

  switch (address) {
  case 0xFF40: /* FF40 - LCDC - LCD Control (R/W) **/
    /*Bit 7 - LCD Display Enable             (0=Off, 1=On)
      Bit 6 - Window Tile Map Display Select (0=9800-9BFF, 1=9C00-9FFF)
      Bit 5 - Window Display Enable          (0=Off, 1=On)
      Bit 4 - BG & Window Tile Data Select   (0=8800-97FF, 1=8000-8FFF)
      Bit 3 - BG Tile Map Display Select     (0=9800-9BFF, 1=9C00-9FFF)
      Bit 2 - OBJ (Sprite) Size              (0=8x8, 1=8x16)
      Bit 1 - OBJ (Sprite) Display Enable    (0=Off, 1=On)
      Bit 0 - BG/Window Display/Priority     (0=Off, 1=On)
     */
    return lh5028->reg40_LCDC;
  case 0xFF41: /* FF41 - STAT - LCDC Status (R/W) */
    /*Bit 6 - LYC=LY Coincidence Interrupt (1=Enable) (Read/Write)
      Bit 5 - Mode 2 OAM Interrupt         (1=Enable) (Read/Write)
      Bit 4 - Mode 1 V-Blank Interrupt     (1=Enable) (Read/Write)
      Bit 3 - Mode 0 H-Blank Interrupt     (1=Enable) (Read/Write)
      Bit 2 - Coincidence Flag  (0:LYC<>LY, 1:LYC=LY) (Read Only)
      Bit 1-0 - Mode Flag       (Mode 0-3, see below) (Read Only)
              0: During H-Blank
              1: During V-Blank
              2: During Searching OAM
              3: During Transferring Data to LCD Driver
    */
    return lh5028->reg41_LCDS;
  case 0xFF42: /* FF42 - SCY - Scroll Y (R/W) */
    return lh5028->reg42_SCY;
  case 0xFF43: /* FF43 - SCX - Scroll X (R/W) */
    return lh5028->reg43_SCX;
  case 0xFF44: /* FF44 - LY - LCDC Y-Coordinate (R) */
    return lh5028->reg44_LY;
  case 0xFF45: /* FF45 - LYC - LY Compare (R/W) */
    return lh5028->reg45_LYC;
  case 0xFF4A: /* FF4A - WY - Window Y Position (R/W) */
    return lh5028->reg4A_WY;
  case 0xFF4B: /* FF4B - WX - Window X Position minus 7 (R/W) */
    return lh5028->reg4B_WX;
  case 0xFF47: /* FF47 - BGP - BG Palette Data (R/W) - Non CGB Mode Only */
    return lh5028->reg47_BGP;
  case 0xFF48: /* FF48 - OBP0 - Object Palette 0 Data (R/W) - Non CGB Mode Only */
    return lh5028->reg48_OBP0;
  case 0xFF49: /* FF49 - OBP1 - Object Palette 1 Data (R/W) - Non CGB Mode Only */
    return lh5028->reg49_OBP1;
  case 0xFF68: /* FF68 - BCPS/BGPI - CGB Mode Only - Background Palette Index */
    return lh5028->reg68_BCPS;
  case 0xFF69: /* FF69 - BCPD/BGPD - CGB Mode Only - Background Palette Data */
    return lh5028->reg69_BCPD;
  case 0xFF6A: /* FF6A - OCPS/OBPI - CGB Mode Only - Sprite Palette Index */
    return lh5028->reg6A_OCPS;
  case 0xFF6B: /* FF6B - OCPD/OBPD - CGB Mode Only - Sprite Palette Data */
    return lh5028->reg6B_OCPD;
  case 0xFF46: /* FF46 - DMA - DMA Transfer and Start Address (R/W) */
    return lh5028->reg46_DMA;
  case 0xFF51: /* FF51 - HDMA1 - CGB Mode Only - New DMA Source, High */
    return lh5028->reg51_HDMA1;
  case 0xFF52: /* FF52 - HDMA2 - CGB Mode Only - New DMA Source, Low*/
    return lh5028->reg52_HDMA2;
  case 0xFF53: /* FF53 - HDMA3 - CGB Mode Only - New DMA Destination, High */
    return lh5028->reg53_HDMA3;
  case 0xFF54: /* FF54 - HDMA4 - CGB Mode Only - New DMA Destination, Low*/
    return lh5028->reg54_HDMA4;
  case 0xFF55: /* FF55 - HDMA5 - CGB Mode Only - New DMA Length/Mode/Start */
    return lh5028->reg55_HDMA5;
  case 0xFF4F: /* FF4F - VBK - CGB Mode Only - VRAM Bank (R/W) */
    return lh5028->reg4F_VBK;
    break;
  default:
    return 0xFF;
  }
}
static /* this method for gameboy color */
void bgwin_render_cgb (struct ppu *lh5028, int16_t scanline) {
  struct {
    union { uint16_t blk;
      struct { uint8_t lo; uint8_t hi; }; };
  } chrdat,  chrcac;
  /* always scan 168 pixel in every line (21 tiles), 
    evenif omfx is ZERO .
      fit buffer offset, so that every time we can scan a complete tile, 
          no matter how much omfx is.
    */
  int32_t omfx;
  int32_t ofx; 
  int32_t obx; 
  int32_t omfy; 
  int32_t ofy; 
  int32_t vsc;
  uint8_t tid;
  uint8_t attr; 
  uint8_t col;
  int32_t tidaddr;
  uint16_t pixcac;
  uint8_t *tdat;
  int32_t rxpos;
  uint32_t c, q, c2, s;
  int32_t c3;
  uint16_t *vptrWinDrawStart;
  uint16_t *vptrScrollStart;
  uint32_t *vptrWinDrawStart32;
  uint32_t *vptrScrollStart32;
  /* check current scan region in BG or WINDOW (if WINDOW enable)*/
  if ( lh5028->reg4A_WYLineHit != 0 && (lh5028->reg40_LCDC & LCDC_WINDOW_MASK)) {
    /* draw window  */
    goto windraw;
  } else {
    /* draw background */
    vsc = lh5028->uscanR;
    omfx = lh5028->reg43_SCX & 7;
    ofx = lh5028->reg43_SCX >> 3; 
    omfy = lh5028->reg42_SCY & 7; 
    ofy = lh5028->reg42_SCY >> 0;
    ofx = ofx + vsc;
    ofy = ofy + scanline;
    omfy = ofy & 7;
    ofy = ofy >> 3;
    ofx = ofx - (ofx & 32); 
    ofy = ofy - (ofy & 32);  
    obx = vsc << 3;
    vptrScrollStart = & lh5028->fmebuf.buf[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf[0]))-omfx];
    vptrScrollStart32 = & lh5028->fmebuf.buf32[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf32[0]))-omfx];
    /* pick tileid and attr from ::Bit 3 - BG Tile Map Display Select (0=9800-9BFF, 1=9C00-9FFF) */
    tidaddr = 0x9800 + (((lh5028->reg40_LCDC & 0x08) >> 3) << 10);
    tidaddr = (tidaddr-0x8000)+(ofy<< 5)+ofx;
    tid = lh5028->ram[tidaddr]; // fetch tileid 
    attr = lh5028->ram[0x2000+tidaddr]; // fetch tileid attr
    tdat = & lh5028->ram[attr & LCD_BG_VRAMBANK1_MASK ? 0x2000 : 0]; 
    if (lh5028->reg40_LCDC & 0x10) // 0x8000 unsigned address 
      tdat = & tdat[tid<<4];
    else 
      tdat = & tdat[0x1000+(((int8_t)tid)*16)]; // TODO: done.
    if (attr & LCD_BG_VFLIP_MASK)
      tdat = & tdat[(omfy^7)*2];
    else 
      tdat = & tdat[omfy*2];

    chrdat.blk = *(uint16_t *)tdat;
    if (attr & LCD_BG_HFLIP_MASK) {
      chrcac.blk = chrdat.blk & 0x8080;
      pixcac =          (chrcac.lo << 7) | (chrcac.hi << 8);
      chrcac.blk = chrdat.blk & 0x4040;
      pixcac = pixcac | (chrcac.lo << 6) | (chrcac.hi << 7);
      chrcac.blk = chrdat.blk & 0x2020;
      pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
      chrcac.blk = chrdat.blk & 0x1010;
      pixcac = pixcac | (chrcac.lo << 4) | (chrcac.hi << 5);
      chrcac.blk = chrdat.blk & 0x0808;
      pixcac = pixcac | (chrcac.lo << 3) | (chrcac.hi << 4);
      chrcac.blk = chrdat.blk & 0x0404;
      pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
      chrcac.blk = chrdat.blk & 0x0202;
      pixcac = pixcac | (chrcac.lo << 1) | (chrcac.hi << 2);
      chrcac.blk = chrdat.blk & 0x0101;
      pixcac = pixcac | (chrcac.lo << 0) | (chrcac.hi << 1);        
    } else {
      chrcac.blk = chrdat.blk & 0x8080;
      pixcac =         (chrcac.lo >> 7) | (chrcac.hi >> 6);
      chrcac.blk = chrdat.blk & 0x4040;
      pixcac = pixcac | (chrcac.lo >> 4) | (chrcac.hi >> 3);
      chrcac.blk = chrdat.blk & 0x2020;
      pixcac = pixcac | (chrcac.lo >> 1) | (chrcac.hi >> 0);
      chrcac.blk = chrdat.blk & 0x1010;
      pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
      chrcac.blk = chrdat.blk & 0x0808;
      pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
      chrcac.blk = chrdat.blk & 0x0404;
      pixcac = pixcac | (chrcac.lo << 8) | (chrcac.hi << 9);
      chrcac.blk = chrdat.blk & 0x0202;
      pixcac = pixcac | (chrcac.lo <<11) | (chrcac.hi <<12);
      chrcac.blk = chrdat.blk & 0x0101;
      pixcac = pixcac | (chrcac.lo <<14) | (chrcac.hi <<15);
    }
    rxpos = obx - omfx;
    col = attr & 7;
    if (lh5028->pixel_format == PPU_SAMPLE_RGB15) {
      if (!(lh5028->reg40_LCDC & 0x01)) {
        vptrScrollStart = & vptrScrollStart[obx];
        /* When Bit 0 is cleared, both background and window become blank (white), 
          ie. the Window Display Bit (Bit 5) is ignored in that case. 
            Only Sprites may still be displayed (if enabled in Bit 1).
          */ 
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
            vptrScrollStart[c] = lh5028->olf16[s].pixel;
          else 
            vptrScrollStart[c] = lh5028->cgb_bg_pal16_x1r5g5b5[col][pixcac & 3].rgb15;
          pixcac >>= 2;
        }
      } else  {
        vptrScrollStart = & vptrScrollStart[obx];
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          c2 = pixcac & 3;
          if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS)
            if (!(lh5028->olf16[s].attr & PIXEL_SPRITE_BACK))
              vptrScrollStart[c] = lh5028->olf16[s].pixel;
            else if ( c2 == 0)
              vptrScrollStart[c] = lh5028->olf16[s].pixel;
            else 
              vptrScrollStart[c] = lh5028->cgb_bg_pal16_x1r5g5b5[col][pixcac & 3].rgb15;
          else 
            vptrScrollStart[c] = lh5028->cgb_bg_pal16_x1r5g5b5[col][pixcac & 3].rgb15;
          pixcac >>= 2;
        }
      }
    } else if (lh5028->pixel_format == PPU_SAMPLE_RGB24) {
      if (!(lh5028->reg40_LCDC & 0x01)) {
        vptrScrollStart32 = & vptrScrollStart32[obx];
        /* When Bit 0 is cleared, both background and window become blank (white), 
          ie. the Window Display Bit (Bit 5) is ignored in that case. 
            Only Sprites may still be displayed (if enabled in Bit 1).
          */ 
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
            vptrScrollStart32[c] = lh5028->olf32[s].pixel;
          else 
            vptrScrollStart32[c] = lh5028->cgb_bg_pal32[col][pixcac & 3].rgb32;
          pixcac >>= 2;
        }
      } else  {
        vptrScrollStart32 = & vptrScrollStart32[obx];
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          c2 = pixcac & 3;
          if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS)
            if (!(lh5028->olf32[s].attr & PIXEL_SPRITE_BACK))
              vptrScrollStart32[c] = lh5028->olf32[s].pixel;
            else if ( c2 == 0)
              vptrScrollStart32[c] = lh5028->olf32[s].pixel;
            else 
              vptrScrollStart32[c] = lh5028->cgb_bg_pal32[col][pixcac & 3].rgb32;
          else 
            vptrScrollStart32[c] = lh5028->cgb_bg_pal32[col][pixcac & 3].rgb32;
          pixcac >>= 2;
        }
      }
    }
    rxpos = obx - omfx; // -7 | 25
    rxpos+= 8; // 1 | 33
    rxpos+= 7; // 8 | 40 
    if ((lh5028->reg40_LCDC & 0x20)
        && (lh5028->reg4B_WX <= 166 && lh5028->reg4B_WX < rxpos) /* check X**/
        && (lh5028->reg4A_WY <= scanline && (lh5028->reg4A_WY <= 143))) 
    {
      lh5028->xscanR = 0;
      q = 15 - omfx; // 8 | 16-> 9 
      lh5028->reg4A_WYLineHit = 1;
      // 7->0
      // 8->1 
      while (lh5028->reg4B_WX >= q) // 15 >= q / 16
        {
          lh5028->xscanR ++; //  1 or 2 
          q += 8;
        }
      goto windraw;
    }
  }
  return ;
windraw:
  // return ;
  ofx = lh5028->uscanR - lh5028->xscanR; // espl x  
  c = lh5028->reg4A_WYRSC;
  omfx = 0;
  omfy = c & 7; 
  ofy = c >> 3;
  c3 = lh5028->reg4B_WX - 7;
  c3 = c3 + (ofx<<3);
  vptrWinDrawStart = & lh5028->fmebuf.buf[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf[0]))+c3];
  vptrWinDrawStart32 = & lh5028->fmebuf.buf32[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf32[0]))+c3];
  /* pick tileid and attr from ::Bit 6 - Tile Tile Map Display Select (0=9800-9BFF, 1=9C00-9FFF) */
  tidaddr = 0x9800 + (((lh5028->reg40_LCDC & 0x40) >> 6) << 10);
  tidaddr = (tidaddr-0x8000)+(ofy<< 5)+ofx;
  tid = lh5028->ram[tidaddr]; // fetch tileid 
  attr = lh5028->ram[0x2000+tidaddr]; 
  tdat = & lh5028->ram[attr & LCD_BG_VRAMBANK1_MASK ? 0x2000 : 0]; // bank select.

  if (lh5028->reg40_LCDC & 0x10) // 0x8000 unsigned address 
    tdat = & tdat[tid<<4];
  else 
    tdat = & tdat[0x1000+(((int8_t)tid)*16)]; // TODO: done.
  if (attr & LCD_BG_VFLIP_MASK)
    tdat = & tdat[(omfy^7)*2];
  else 
    tdat = & tdat[omfy*2];

  chrdat.blk = *(uint16_t *)tdat;
  if (attr & LCD_BG_HFLIP_MASK) {
    chrcac.blk = chrdat.blk & 0x8080;
    pixcac =          (chrcac.lo << 7) | (chrcac.hi << 8);
    chrcac.blk = chrdat.blk & 0x4040;
    pixcac = pixcac | (chrcac.lo << 6) | (chrcac.hi << 7);
    chrcac.blk = chrdat.blk & 0x2020;
    pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
    chrcac.blk = chrdat.blk & 0x1010;
    pixcac = pixcac | (chrcac.lo << 4) | (chrcac.hi << 5);
    chrcac.blk = chrdat.blk & 0x0808;
    pixcac = pixcac | (chrcac.lo << 3) | (chrcac.hi << 4);
    chrcac.blk = chrdat.blk & 0x0404;
    pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
    chrcac.blk = chrdat.blk & 0x0202;
    pixcac = pixcac | (chrcac.lo << 1) | (chrcac.hi << 2);
    chrcac.blk = chrdat.blk & 0x0101;
    pixcac = pixcac | (chrcac.lo << 0) | (chrcac.hi << 1);        
  } else {
    chrcac.blk = chrdat.blk & 0x8080;
    pixcac =         (chrcac.lo >> 7) | (chrcac.hi >> 6);
    chrcac.blk = chrdat.blk & 0x4040;
    pixcac = pixcac | (chrcac.lo >> 4) | (chrcac.hi >> 3);
    chrcac.blk = chrdat.blk & 0x2020;
    pixcac = pixcac | (chrcac.lo >> 1) | (chrcac.hi >> 0);
    chrcac.blk = chrdat.blk & 0x1010;
    pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
    chrcac.blk = chrdat.blk & 0x0808;
    pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
    chrcac.blk = chrdat.blk & 0x0404;
    pixcac = pixcac | (chrcac.lo << 8) | (chrcac.hi << 9);
    chrcac.blk = chrdat.blk & 0x0202;
    pixcac = pixcac | (chrcac.lo <<11) | (chrcac.hi <<12);
    chrcac.blk = chrdat.blk & 0x0101;
    pixcac = pixcac | (chrcac.lo <<14) | (chrcac.hi <<15);
  }

  col = attr & 7;
  if (lh5028->pixel_format == PPU_SAMPLE_RGB15) {
    if (!(lh5028->reg40_LCDC & 0x01)) {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
          vptrWinDrawStart[c] = lh5028->olf16[s].pixel;
        else 
          vptrWinDrawStart[c] = lh5028->dmg_pal16.bg_pal[0];
      }
    } else  {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        c2 = pixcac & 3;
        if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS)
          if (!(lh5028->olf16[s].attr & PIXEL_SPRITE_BACK))
            vptrWinDrawStart[c] = lh5028->olf16[s].pixel;
          else if ( c2 == 0)
            vptrWinDrawStart[c] = lh5028->olf16[s].pixel;
          else 
            vptrWinDrawStart[c] = lh5028->cgb_bg_pal16_x1r5g5b5[col][pixcac & 3].rgb15;
        else 
          vptrWinDrawStart[c] = lh5028->cgb_bg_pal16_x1r5g5b5[col][pixcac & 3].rgb15;
        pixcac >>= 2;
      }
    }
  } else if (lh5028->pixel_format == PPU_SAMPLE_RGB24) {
    if (!(lh5028->reg40_LCDC & 0x01)) {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
          vptrWinDrawStart32[c] = lh5028->olf32[s].pixel;
        else 
          vptrWinDrawStart32[c] = lh5028->dmg_pal32.bg_pal[0];
      }
    } else  {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        c2 = pixcac & 3;
        if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS)
          if (!(lh5028->olf32[s].attr & PIXEL_SPRITE_BACK))
            vptrWinDrawStart32[c] = lh5028->olf32[s].pixel;
          else if ( c2 == 0)
            vptrWinDrawStart32[c] = lh5028->olf32[s].pixel;
          else 
            vptrWinDrawStart32[c] = lh5028->cgb_bg_pal32[col][pixcac & 3].rgb32;
        else 
          vptrWinDrawStart32[c] = lh5028->cgb_bg_pal32[col][pixcac & 3].rgb32;
        pixcac >>= 2;
      }
    }
  }
}
static /* this method for gameboy */
void bgwin_render_dmg (struct ppu *lh5028, int16_t scanline) {
  struct {
    union { uint16_t blk;
      struct { uint8_t lo; uint8_t hi; }; };
  } chrdat,  chrcac;
  /* always scan 168 pixel in every line (21 tiles), 
    evenif omfx is ZERO .
      fit buffer offset, so that every time we can scan a complete tile, 
          no matter how much omfx is.
    */
  int32_t omfx;
  int32_t ofx; 
  int32_t obx; 
  int32_t omfy; 
  int32_t ofy; 
  int32_t vsc;
  uint8_t tid;
  int32_t tidaddr;
  uint16_t pixcac;
  uint8_t *tdat;
  int32_t rxpos;
  uint32_t c, q, c2, s;
  int32_t c3;
  uint16_t *vptrWinDrawStart;
  uint16_t *vptrScrollStart;
  uint32_t *vptrWinDrawStart32;
  uint32_t *vptrScrollStart32;
  /* check current scan region in BG or WINDOW (if WINDOW enable)*/
  if ( lh5028->reg4A_WYLineHit != 0 && (lh5028->reg40_LCDC & LCDC_WINDOW_MASK)) {
    /* draw window  */
    goto windraw;
  } else {
    /* draw background */
    vsc = lh5028->uscanR;
    omfx = lh5028->reg43_SCX & 7;
    ofx = lh5028->reg43_SCX >> 3; 
    omfy = lh5028->reg42_SCY & 7; 
    ofy = lh5028->reg42_SCY >> 0;
    ofx = ofx + vsc;
    ofy = ofy + scanline;
    omfy = ofy & 7;
    ofy = ofy >> 3;
    ofx = ofx - (ofx & 32); 
    ofy = ofy - (ofy & 32);  
    obx = vsc << 3;
    vptrScrollStart = & lh5028->fmebuf.buf[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf[0]))-omfx];
    vptrScrollStart32 = & lh5028->fmebuf.buf32[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf32[0]))-omfx];
    /* pick tileid and attr from ::Bit 3 - BG Tile Map Display Select (0=9800-9BFF, 1=9C00-9FFF) */
    tidaddr = 0x9800 + (((lh5028->reg40_LCDC & 0x08) >> 3) << 10);
    tidaddr = (tidaddr-0x8000)+(ofy<< 5)+ofx;
    tid = lh5028->ram[tidaddr]; // fetch tileid 
    tdat = & lh5028->ram[0]; 
#   if  1
    if (lh5028->reg40_LCDC & 0x10) // 0x8000 unsigned address 
      tdat = & tdat[tid<<4];
    else // 
      tdat = & tdat[0x1000+(((int8_t)tid)*16)]; // TODO: done.
#   else 
    c2 = (lh5028->reg40_LCDC & 0x10) << 8;
    c2^= 0x1000;
    tdat = & tdat[c2+((int8_t)(c2 >>5)) & (((int8_t)tid) << 4)];
#   endif 
    tdat = & tdat[omfy*2];
    chrdat.blk = *(uint16_t *)tdat;
    chrcac.blk = chrdat.blk & 0x8080;
    pixcac =          (chrcac.lo >> 7) | (chrcac.hi >> 6);
    chrcac.blk = chrdat.blk & 0x4040;
    pixcac = pixcac | (chrcac.lo >> 4) | (chrcac.hi >> 3);
    chrcac.blk = chrdat.blk & 0x2020;
    pixcac = pixcac | (chrcac.lo >> 1) | (chrcac.hi >> 0);
    chrcac.blk = chrdat.blk & 0x1010;
    pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
    chrcac.blk = chrdat.blk & 0x0808;
    pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
    chrcac.blk = chrdat.blk & 0x0404;
    pixcac = pixcac | (chrcac.lo << 8) | (chrcac.hi << 9);
    chrcac.blk = chrdat.blk & 0x0202;
    pixcac = pixcac | (chrcac.lo <<11) | (chrcac.hi <<12);
    chrcac.blk = chrdat.blk & 0x0101;
    pixcac = pixcac | (chrcac.lo <<14) | (chrcac.hi <<15);
    rxpos = obx - omfx;
    
    if (lh5028->pixel_format == PPU_SAMPLE_RGB15) {
      if (!(lh5028->reg40_LCDC & 0x01)) {
        vptrScrollStart = & vptrScrollStart[obx];
        /* When Bit 0 is cleared, both background and window become blank (white), 
          ie. the Window Display Bit (Bit 5) is ignored in that case. 
            Only Sprites may still be displayed (if enabled in Bit 1).
          */ 
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
            vptrScrollStart[c] = lh5028->olf16[s].pixel;
          else 
            vptrScrollStart[c] = lh5028->dmg_pal16.bg_pal[0];
          pixcac >>= 2;
        }
      } else  {
        vptrScrollStart = & vptrScrollStart[obx];
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          c2 = pixcac & 3;
          if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS)
            if (!(lh5028->olf16[s].attr & PIXEL_SPRITE_BACK))
              vptrScrollStart[c] = lh5028->olf16[s].pixel;
            else if ( c2 == 0)
              vptrScrollStart[c] = lh5028->olf16[s].pixel;
            else 
              vptrScrollStart[c] = lh5028->dmg_pal16.bg_pal[pixcac & 3];
          else 
            vptrScrollStart[c] = lh5028->dmg_pal16.bg_pal[pixcac & 3];
          pixcac >>= 2;
        }
      }
    } else if (lh5028->pixel_format == PPU_SAMPLE_RGB24) {
      if (!(lh5028->reg40_LCDC & 0x01)) {
        vptrScrollStart32 = & vptrScrollStart32[obx];
        /* When Bit 0 is cleared, both background and window become blank (white), 
          ie. the Window Display Bit (Bit 5) is ignored in that case. 
            Only Sprites may still be displayed (if enabled in Bit 1).
          */ 
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
            vptrScrollStart32[c] = lh5028->olf32[s].pixel;
          else 
            vptrScrollStart32[c] = lh5028->dmg_pal32.bg_pal[0];
          pixcac >>= 2;
        }
      } else  {
        vptrScrollStart32 = & vptrScrollStart32[obx];
        for (c = 0; c != 8; c++) {
          s = rxpos+c;
          c2 = pixcac & 3;
          if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS)
            if (!(lh5028->olf32[s].attr & PIXEL_SPRITE_BACK))
              vptrScrollStart32[c] = lh5028->olf32[s].pixel;
            else if ( c2 == 0)
              vptrScrollStart32[c] = lh5028->olf32[s].pixel;
            else 
              vptrScrollStart32[c] = lh5028->dmg_pal32.bg_pal[pixcac & 3];
          else 
            vptrScrollStart32[c] = lh5028->dmg_pal32.bg_pal[pixcac & 3];
          pixcac >>= 2;
        }
      }
    }
    rxpos = obx - omfx; // -7 | 25
    rxpos+= 8; // 1 | 33
    rxpos+= 7; // 8 | 40 
    if ((lh5028->reg40_LCDC & 0x20)
        && (lh5028->reg4B_WX <= 166 && lh5028->reg4B_WX < rxpos) /* check X**/
        && (lh5028->reg4A_WY <= scanline && (lh5028->reg4A_WY <= 143))) 
    {
      lh5028->xscanR = 0;
      q = 15 - omfx; // 8 | 16-> 9 
      lh5028->reg4A_WYLineHit = 1;
      // 7->0
      // 8->1 
      while (lh5028->reg4B_WX >= q) // 15 >= q / 16
        {
          lh5028->xscanR ++; //  1 or 2 
          q += 8;
        }
      goto windraw;
    }
  }
  return ;
windraw:
  // return ;
  ofx = lh5028->uscanR - lh5028->xscanR; // espl x  
  c = lh5028->reg4A_WYRSC;
  omfx = 0;
  omfy = c & 7; 
  ofy = c >> 3;
  c3 = lh5028->reg4B_WX - 7;
  c3 = c3 + (ofx<<3);
  vptrWinDrawStart = & lh5028->fmebuf.buf[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf[0]))+c3];
  vptrWinDrawStart32 = & lh5028->fmebuf.buf32[scanline *(lh5028->fmebuf.pitch/sizeof (lh5028->fmebuf.buf32[0]))+c3];
  /* pick tileid and attr from ::Bit 6 - Tile Tile Map Display Select (0=9800-9BFF, 1=9C00-9FFF) */
  tidaddr = 0x9800 + (((lh5028->reg40_LCDC & 0x40) >> 6) << 10);
  tidaddr = (tidaddr-0x8000)+(ofy<< 5)+ofx;
  tid = lh5028->ram[tidaddr]; // fetch tileid 
  tdat = & lh5028->ram[0]; // bank select.
#   if  1
  if (lh5028->reg40_LCDC & 0x10) // 0x8000 unsigned address 
    tdat = & tdat[tid<<4];
  else // 
    tdat = & tdat[0x1000+(((int8_t)tid)*16)]; // TODO: done.
#   else 
  cti16 = (lh5028->reg40_LCDC & 0x10) << 8;
  cti16^= 0x1000;
  tdat = & tdat[cti16+((int8_t)(cti16 >>5)) & (((int8_t)tid) << 4)];
#   endif 
  tdat = & tdat[omfy*2];
  chrdat.blk = *(uint16_t *)tdat;
  chrcac.blk = chrdat.blk & 0x8080;
  pixcac =          (chrcac.lo >> 7) | (chrcac.hi >> 6);
  chrcac.blk = chrdat.blk & 0x4040;
  pixcac = pixcac | (chrcac.lo >> 4) | (chrcac.hi >> 3);
  chrcac.blk = chrdat.blk & 0x2020;
  pixcac = pixcac | (chrcac.lo >> 1) | (chrcac.hi >> 0);
  chrcac.blk = chrdat.blk & 0x1010;
  pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
  chrcac.blk = chrdat.blk & 0x0808;
  pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
  chrcac.blk = chrdat.blk & 0x0404;
  pixcac = pixcac | (chrcac.lo << 8) | (chrcac.hi << 9);
  chrcac.blk = chrdat.blk & 0x0202;
  pixcac = pixcac | (chrcac.lo <<11) | (chrcac.hi <<12);
  chrcac.blk = chrdat.blk & 0x0101;
  pixcac = pixcac | (chrcac.lo <<14) | (chrcac.hi <<15);

  if (lh5028->pixel_format == PPU_SAMPLE_RGB15) {
    if (!(lh5028->reg40_LCDC & 0x01)) {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
          vptrWinDrawStart[c] = lh5028->olf16[s].pixel;
        else 
          vptrWinDrawStart[c] = lh5028->dmg_pal16.bg_pal[0];
      }
    } else  {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        c2 = pixcac & 3;
        if (lh5028->olf16[s].attr & PIXEL_SPRITE_NOTRANS)
          if (!(lh5028->olf16[s].attr & PIXEL_SPRITE_BACK))
            vptrWinDrawStart[c] = lh5028->olf16[s].pixel;
          else if ( c2 == 0)
            vptrWinDrawStart[c] = lh5028->olf16[s].pixel;
          else 
            vptrWinDrawStart[c] = lh5028->dmg_pal16.bg_pal[pixcac & 3];
        else 
          vptrWinDrawStart[c] = lh5028->dmg_pal16.bg_pal[pixcac & 3];
        pixcac >>= 2;
      }
    }
  } else if (lh5028->pixel_format == PPU_SAMPLE_RGB24) {
    if (!(lh5028->reg40_LCDC & 0x01)) {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS) // spline ptr to +8
          vptrWinDrawStart32[c] = lh5028->olf32[s].pixel;
        else 
          vptrWinDrawStart32[c] = lh5028->dmg_pal32.bg_pal[0];
      }
    } else  {
      for (c = 0; c != 8; c++) {
        s = c3+c;
        c2 = pixcac & 3;
        if (lh5028->olf32[s].attr & PIXEL_SPRITE_NOTRANS)
          if (!(lh5028->olf32[s].attr & PIXEL_SPRITE_BACK))
            vptrWinDrawStart32[c] = lh5028->olf32[s].pixel;
          else if ( c2 == 0)
            vptrWinDrawStart32[c] = lh5028->olf32[s].pixel;
          else 
            vptrWinDrawStart32[c] = lh5028->dmg_pal32.bg_pal[pixcac & 3];
        else 
          vptrWinDrawStart32[c] = lh5028->dmg_pal32.bg_pal[pixcac & 3];
        pixcac >>= 2;
      }
    }
  }
}
 /* this method for gameboy color,
         compared with DMG, bank addressable and high color palette has been added.*/
static
void sprite_render_cgb (struct ppu *lh5028, int16_t delta) {

  struct {
    union {
      uint16_t blk;
      struct {
        uint8_t lo;
        uint8_t hi;
      };
    };
  } chrdat,  chrcac;

  while (delta > 0 && (lh5028->vscan40 < 40) && (lh5028->vscanR < 10)) {
    intptr_t size16 = (lh5028->reg40_LCDC & LCDC_OAM_SIZE16_MASK) ? 0 : 8;
    struct oam *ops = & lh5028->sp[lh5028->vscan40];
    lh5028->vscan40++;
    if ( !(ops->y == 0  || ops->y >= 160
     || ops->x == 0 || ops->x >= 168)
     && (lh5028->reg44_LY >= (ops->y - 16))
     && (lh5028->reg44_LY < (ops->y - size16)))
    {
      /*draw scanline */
      uint16_t pixcac ;
      uint8_t *chr;
      uint32_t id;
      uint32_t vbank_base = ops->attr & LCD_OAM_VRAMBANK1_MASK ? 0x2000 : 0x0000;
      uint32_t col;
      int x;

      lh5028->vscanR++;
      delta--;
      if (size16 == 0)
        chr = & lh5028->ram[vbank_base + ((ops->id & 0xFE)<<4)]; // always pointer tile16's high part in init state .
      else 
        chr = & lh5028->ram[vbank_base + ((ops->id & 0xFF)<<4)];
      // DEBUG_OUT ("reg44_LY:%d\n", lh5028->reg44_LY);
      if (size16 == 0) {
        if (ops->attr & LCD_OAM_FLIP_Y_MASK) {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          /* get oppo base tile byte8 (^source pos[high or low])*/
          if (cc < 7) /* high, switch to low */ {
            cc = 7 -cc; /* flip Y in one tile */
            cc<<= 1;
            cc += 16;
          } else { /* low switch to high, nodone, already in high.*/    
            cc = 15 -cc; /* flip Y in one tile */
            cc<<= 1;
          }
          chrdat.blk = *(int16_t *)& chr[cc];
        } else {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          cc<<= 1;
          chrdat.blk = *(int16_t *)& chr[cc];
        }
      } else {
        if (ops->attr & LCD_OAM_FLIP_Y_MASK) {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          cc = 7 -cc; /* flip Y in one tile */
          cc *= 2;
          chrdat.blk = *(int16_t *)& chr[cc];
        } else {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          cc *= 2;
          chrdat.blk = *(int16_t *)& chr[cc];
        }
      }
      /* mix gbc's pixel (d1d0) 
       * see GBCPUman.pdf:: 2.8.1. Tiles 
       * check x flip  
       */
      if (ops->attr & LCD_OAM_FLIP_X_MASK) {
        chrcac.blk = chrdat.blk & 0x8080;
        pixcac =          (chrcac.lo << 7) | (chrcac.hi << 8);
        chrcac.blk = chrdat.blk & 0x4040;
        pixcac = pixcac | (chrcac.lo << 6) | (chrcac.hi << 7);
        chrcac.blk = chrdat.blk & 0x2020;
        pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
        chrcac.blk = chrdat.blk & 0x1010;
        pixcac = pixcac | (chrcac.lo << 4) | (chrcac.hi << 5);
        chrcac.blk = chrdat.blk & 0x0808;
        pixcac = pixcac | (chrcac.lo << 3) | (chrcac.hi << 4);
        chrcac.blk = chrdat.blk & 0x0404;
        pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
        chrcac.blk = chrdat.blk & 0x0202;
        pixcac = pixcac | (chrcac.lo << 1) | (chrcac.hi << 2);
        chrcac.blk = chrdat.blk & 0x0101;
        pixcac = pixcac | (chrcac.lo << 0) | (chrcac.hi << 1);        
      } else {
        chrcac.blk = chrdat.blk & 0x8080;
        pixcac =         (chrcac.lo >> 7) | (chrcac.hi >> 6);
        chrcac.blk = chrdat.blk & 0x4040;
        pixcac = pixcac | (chrcac.lo >> 4) | (chrcac.hi >> 3);
        chrcac.blk = chrdat.blk & 0x2020;
        pixcac = pixcac | (chrcac.lo >> 1) | (chrcac.hi >> 0);
        chrcac.blk = chrdat.blk & 0x1010;
        pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
        chrcac.blk = chrdat.blk & 0x0808;
        pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
        chrcac.blk = chrdat.blk & 0x0404;
        pixcac = pixcac | (chrcac.lo << 8) | (chrcac.hi << 9);
        chrcac.blk = chrdat.blk & 0x0202;
        pixcac = pixcac | (chrcac.lo <<11) | (chrcac.hi <<12);
        chrcac.blk = chrdat.blk & 0x0101;
        pixcac = pixcac | (chrcac.lo <<14) | (chrcac.hi <<15);
      }
      x = ops->x - 8;
      col = ops->attr & 7; /* get cgb obp col palette index */
      if (lh5028->pixel_format == PPU_SAMPLE_RGB15) {
        for (id = 8; id != 0; id--) {
          uint8_t idpal = pixcac & 3; 
          /* check trans or */
          if (lh5028->olf16[x].attr & PIXEL_SPRITE_NOTRANS) {
            /* notrans */
            if (lh5028->olf16[x].attr & PIXEL_SPRITE_BACK) {
              if (!(ops->attr & LCD_OAM_BACKGROUND_MASK) && idpal != 0) {
  #             if  1 /* maybe error, it's not a big problem. */
                /* cur pick oam is foreground sprite */
                lh5028->olf16[x].pixel = lh5028->cgb_sp_pal16_x1r5g5b5[col][idpal].rgb15;
                lh5028->olf16[x].attr = PIXEL_SPRITE_NOTRANS;            
  #             endif 
              }
            } else {
              /* nodone in foreground sprite */
            }
          } else {
            /* trans, write directly */
            lh5028->olf16[x].pixel = lh5028->cgb_sp_pal16_x1r5g5b5[col][idpal].rgb15;
            lh5028->olf16[x].attr = 0;

            if (ops->attr & LCD_OAM_BACKGROUND_MASK)
              lh5028->olf16[x].attr |= PIXEL_SPRITE_BACK;
            if (idpal != 0)
              lh5028->olf16[x].attr |= PIXEL_SPRITE_NOTRANS; 
          }
          x ++;
          pixcac >>= 2;
        } /* render loop, TODO: pri sprite */
      } else if (lh5028->pixel_format == PPU_SAMPLE_RGB24) {
        for (id = 8; id != 0; id--) {
          uint8_t idpal = pixcac & 3; 
          /* check trans or */
          if (lh5028->olf32[x].attr & PIXEL_SPRITE_NOTRANS) {
            /* notrans */
            if (lh5028->olf32[x].attr & PIXEL_SPRITE_BACK) {
              if (!(ops->attr & LCD_OAM_BACKGROUND_MASK) && idpal != 0) {
  #             if  1 /* maybe error, it's not a big problem. */
                /* cur pick oam is foreground sprite */
                lh5028->olf32[x].pixel = lh5028->cgb_sp_pal32[col][idpal].rgb32;
                lh5028->olf32[x].attr = PIXEL_SPRITE_NOTRANS;            
  #             endif 
              }
            } else {
              /* nodone in foreground sprite */
            }
          } else {
            /* trans, write directly */
            lh5028->olf32[x].pixel = lh5028->cgb_sp_pal32[col][idpal].rgb32;
            lh5028->olf32[x].attr = 0;

            if (ops->attr & LCD_OAM_BACKGROUND_MASK)
              lh5028->olf32[x].attr |= PIXEL_SPRITE_BACK;
            if (idpal != 0)
              lh5028->olf32[x].attr |= PIXEL_SPRITE_NOTRANS; 
          }
          x ++;
          pixcac >>= 2;
        } /* render loop, TODO: pri sprite */
      } else {
        assert (0);
      }
    }
  }
}

static /* this method for gameboy */
void sprite_render_dmg (struct ppu *lh5028, int16_t delta) {

  struct {
    union {
      uint16_t blk;
      struct {
        uint8_t lo;
        uint8_t hi;
      };
    };
  } chrdat,  chrcac;

  while (delta > 0 && (lh5028->vscan40 < 40) && (lh5028->vscanR < 10)) {
    intptr_t size16 = (lh5028->reg40_LCDC & LCDC_OAM_SIZE16_MASK) ? 0 : 8;
    struct oam *ops = & lh5028->sp[lh5028->vscan40];
    lh5028->vscan40++;
    if ( !(ops->y == 0  || ops->y >= 160
     || ops->x == 0 || ops->x >= 168)
     && (lh5028->reg44_LY >= (ops->y - 16))
     && (lh5028->reg44_LY < (ops->y - size16)))
    {
      /*draw scanline */
      uint16_t pixcac ;
      uint8_t *chr;
      uint32_t id;
      int x;

      lh5028->vscanR++;
      delta--;
      if (size16 == 0)
        chr = & lh5028->ram[(ops->id & 0xFE)<<4]; // always pointer tile16's high part in init state .
      else 
        chr = & lh5028->ram[(ops->id & 0xFF)<<4];
      // DEBUG_OUT ("reg44_LY:%d\n", lh5028->reg44_LY);
      if (size16 == 0) {
        if (ops->attr & LCD_OAM_FLIP_Y_MASK) {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          /* get oppo base tile byte8 (^source pos[high or low])*/
          if (cc < 7) /* high, switch to low */ {
            cc = 7 -cc; /* flip Y in one tile */
            cc<<= 1;
            cc += 16;
          } else { /* low switch to high, nodone, already in high.*/    
            cc = 15 -cc; /* flip Y in one tile */
            cc<<= 1;
          }
          chrdat.blk = *(int16_t *)& chr[cc];
        } else {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          cc<<= 1;
          chrdat.blk = *(int16_t *)& chr[cc];
        }
      } else {
        if (ops->attr & LCD_OAM_FLIP_Y_MASK) {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          cc = 7 -cc; /* flip Y in one tile */
          cc *= 2;
          chrdat.blk = *(int16_t *)& chr[cc];
        } else {
          int cc = lh5028->reg44_LY - (ops->y - 16);
          cc *= 2;
          chrdat.blk = *(int16_t *)& chr[cc];
        }
      }
      /* mix gbc's pixel (d1d0) 
       * see GBCPUman.pdf:: 2.8.1. Tiles 
       * check x flip  
       */
      if (ops->attr & LCD_OAM_FLIP_X_MASK) {
        chrcac.blk = chrdat.blk & 0x8080;
        pixcac =          (chrcac.lo << 7) | (chrcac.hi << 8);
        chrcac.blk = chrdat.blk & 0x4040;
        pixcac = pixcac | (chrcac.lo << 6) | (chrcac.hi << 7);
        chrcac.blk = chrdat.blk & 0x2020;
        pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
        chrcac.blk = chrdat.blk & 0x1010;
        pixcac = pixcac | (chrcac.lo << 4) | (chrcac.hi << 5);
        chrcac.blk = chrdat.blk & 0x0808;
        pixcac = pixcac | (chrcac.lo << 3) | (chrcac.hi << 4);
        chrcac.blk = chrdat.blk & 0x0404;
        pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
        chrcac.blk = chrdat.blk & 0x0202;
        pixcac = pixcac | (chrcac.lo << 1) | (chrcac.hi << 2);
        chrcac.blk = chrdat.blk & 0x0101;
        pixcac = pixcac | (chrcac.lo << 0) | (chrcac.hi << 1);        
      } else {
        chrcac.blk = chrdat.blk & 0x8080;
        pixcac =         (chrcac.lo >> 7) | (chrcac.hi >> 6);
        chrcac.blk = chrdat.blk & 0x4040;
        pixcac = pixcac | (chrcac.lo >> 4) | (chrcac.hi >> 3);
        chrcac.blk = chrdat.blk & 0x2020;
        pixcac = pixcac | (chrcac.lo >> 1) | (chrcac.hi >> 0);
        chrcac.blk = chrdat.blk & 0x1010;
        pixcac = pixcac | (chrcac.lo << 2) | (chrcac.hi << 3);
        chrcac.blk = chrdat.blk & 0x0808;
        pixcac = pixcac | (chrcac.lo << 5) | (chrcac.hi << 6);
        chrcac.blk = chrdat.blk & 0x0404;
        pixcac = pixcac | (chrcac.lo << 8) | (chrcac.hi << 9);
        chrcac.blk = chrdat.blk & 0x0202;
        pixcac = pixcac | (chrcac.lo <<11) | (chrcac.hi <<12);
        chrcac.blk = chrdat.blk & 0x0101;
        pixcac = pixcac | (chrcac.lo <<14) | (chrcac.hi <<15);
      }
      x = ops->x - 8;
      if (lh5028->pixel_format == PPU_SAMPLE_RGB15) {
        for (id = 8; id != 0; id--) {
          uint8_t idpal = pixcac & 3; 
          /* check trans or */
          if (lh5028->olf16[x].attr & PIXEL_SPRITE_NOTRANS) {
            /* notrans */
            if (lh5028->olf16[x].attr & PIXEL_SPRITE_BACK) {
              if (!(ops->attr & LCD_OAM_BACKGROUND_MASK) && idpal != 0) {
  #             if  1 /* maybe error, it's not a big problem. */
                /* cur pick oam is foreground sprite */
                lh5028->olf16[x].pixel = lh5028->dmg_pal16.sp_pal[(ops->attr & 0x10) >> 4][idpal];
                lh5028->olf16[x].attr = PIXEL_SPRITE_NOTRANS;            
  #             endif 
              }
            } else {
              /* nodone in foreground sprite */
            }
          } else {
            /* trans, write directly */
            lh5028->olf16[x].pixel = lh5028->dmg_pal16.sp_pal[(ops->attr & 0x10) >> 4][idpal];
            lh5028->olf16[x].attr = 0;

            if (ops->attr & LCD_OAM_BACKGROUND_MASK)
              lh5028->olf16[x].attr |= PIXEL_SPRITE_BACK;
            if (idpal != 0)
              lh5028->olf16[x].attr |= PIXEL_SPRITE_NOTRANS; 
          }
          x ++;
          pixcac >>= 2;
        } /* render loop, TODO: pri sprite */
      } else if (lh5028->pixel_format == PPU_SAMPLE_RGB24) {
        for (id = 8; id != 0; id--) {
          uint8_t idpal = pixcac & 3; 
          /* check trans or */
          if (lh5028->olf32[x].attr & PIXEL_SPRITE_NOTRANS) {
            /* notrans */
            if (lh5028->olf32[x].attr & PIXEL_SPRITE_BACK) {
              if (!(ops->attr & LCD_OAM_BACKGROUND_MASK) && idpal != 0) {
  #             if  1 /* maybe error, it's not a big problem. */
                /* cur pick oam is foreground sprite */
                lh5028->olf32[x].pixel = lh5028->dmg_pal32.sp_pal[(ops->attr & 0x10) >> 4][idpal];
                lh5028->olf32[x].attr = PIXEL_SPRITE_NOTRANS;            
  #             endif 
              }
            } else {
              /* nodone in foreground sprite */
            }
          } else {
            /* trans, write directly */
            lh5028->olf32[x].pixel = lh5028->dmg_pal32.sp_pal[(ops->attr & 0x10) >> 4][idpal];
            lh5028->olf32[x].attr = 0;

            if (ops->attr & LCD_OAM_BACKGROUND_MASK)
              lh5028->olf32[x].attr |= PIXEL_SPRITE_BACK;
            if (idpal != 0)
              lh5028->olf32[x].attr |= PIXEL_SPRITE_NOTRANS; 
          }
          x ++;
          pixcac >>= 2;
        } /* render loop, TODO: pri sprite */
      } else {
        assert (0);
      }
    }
  }
}
/* XXX:so bad */
static 
void ticks (struct ppu *lh5028) {
  
  int32_t scanline;
  int16_t interv;
  int16_t delta;
  double clkline;
  
  if (lh5028->gb->cpu_clks_ppu > lh5028->gb->mach_tools->frame_cycles) {
    lh5028->gb->cpu_clks_ppu -= lh5028->gb->mach_tools->frame_cycles; // sub a frame block 
    /* vblank Interrupt, check missing interruption */
    if (lh5028->reg40_NMIf != 0 && (lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)) 
      lh5028->gb->reg0F_IF |= IRQ_1;
    /* vblank stat Interrupt, check missing interruption */
    if ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
       && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_INTERRUPET_VBLANK_MASK))
      lh5028->gb->reg0F_IF |= IRQ_2;
    /* the beginning of a new frame  reset ppu's context */
    lh5028->reg41_IRQf |= LCDS_INTERRUPET_ALL_MASK;
    lh5028->reg40_NMIf = 1;
    lh5028->vscan = -1;
    lh5028->vscanR = 0;
    lh5028->vscan40 = 0;
    lh5028->uscan = -1;
  }
  scanline = (int32_t) (lh5028->gb->cpu_clks_ppu/ lh5028->gb->mach_tools->line_cycles);
  clkline = lh5028->gb->cpu_clks_ppu - lh5028->gb->mach_tools->line_cycles * (double) scanline;
  /* clear mask */
  lh5028->reg41_LCDS &= ~LCDS_MODE_FLAG_ALL_MASK;
  lh5028->reg44_LY = scanline;

  /*
    PPU time series

    Mode 0: The LCD controller is in the H-Blank period and
         the CPU can access both the display RAM (8000h-9FFFh)
         and OAM (FE00h-FE9Fh)

    Mode 1: The LCD controller is in the V-Blank period (or the
         display is disabled) and the CPU can access both the
         display RAM (8000h-9FFFh) and OAM (FE00h-FE9Fh)

    Mode 2: The LCD controller is reading from OAM memory.
         The CPU <cannot> access OAM memory (FE00h-FE9Fh)
         during this period.

    Mode 3: The LCD controller is reading from both OAM and VRAM,
         The CPU <cannot> access OAM and VRAM during this period.
         CGB Mode: Cannot access Palette Data (FF69,FF6B) either.
    The following are typical when the display is enabled:

    Mode 2  2_____2_____2_____2_____2_____2___________________2____
    Mode 3  _33____33____33____33____33____33__________________3___
    Mode 0  ___000___000___000___000___000___000________________000
    Mode 1  ____________________________________11111111111111_____

    The Mode Flag goes through the values 0, 2, and 3 at a cycle of about 109uS. 
    0 is present about 48.6uS, 2 about 19uS, 
    and 3 about 41uS. This is interrupted every 16.6ms by the VBlank (1). 
    The mode flag stays set at 1 for about 1.08 ms.

    Mode 0 is present between 201-207 clks, 2 about 77-83 clks, and 3 about 169-175 clks.
    A complete cycle through these states takes 456 clks. 
    VBlank lasts 4560 clks. A complete screen refresh occurs every 70224 clks.)
 */
  if (lh5028->reg44_LY_T != lh5028->reg44_LY) {
    lh5028->reg41_IRQf |= LCDS_INTERRUPET_LINE_MASK;
    lh5028->reg4A_WYLineHit = 0;
  }
  /* check FF41::Bit 2 Coincidence Flag  (0:LYC<>LY, 1:LYC=LY) (Read Only)*/
  if (lh5028->reg44_LY != lh5028->reg45_LYC)
    lh5028->reg41_LCDS &= ~0x04;
  else {
    if ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
       && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_INTERRUPET_LINE_MASK)) {
       lh5028->reg41_IRQf &= ~LCDS_INTERRUPET_LINE_MASK;
       lh5028->gb->reg0F_IF |= IRQ_2;
    }
    lh5028->reg41_LCDS |= 0x04;
  }

  /* check mode */
  if (lh5028->gb->cpu_clks_ppu < lh5028->gb->mach_tools->vbl_clk_st) {
    if (clkline > lh5028->hbl_clks_st) {
      /* ------------------------------ MODE0-HBLANK ------------------------------ */
      lh5028->reg41_LCDS |= LCDS_MODE_FLAG_HBLANK;
      /*  check edge */
      if (lh5028->reg41_LCDM_T != LCDS_MODE_FLAG_HBLANK) {
        /* oamvram to current, (low to high) */     
        while (lh5028->uscanR < 21) {
          lh5028->bgwin_done (lh5028, scanline); 
          lh5028->uscanR ++;
        }
        if (lh5028->reg4A_WYLineHit != 0)
          lh5028->reg4A_WYRSC++;
        /* reset next mode-2 | mode- 3*/ 
        lh5028->reg41_IRQf |= LCDS_INTERRUPET_OAM_MASK | LCDS_INTERRUPET_VBLANK_MASK;
        lh5028->vscan = -1;
        lh5028->vscan40 = 0;
        lh5028->uscan = -1;
        lh5028->vscanR = 0;
        lh5028->uscanR = 0;
        /* check HDMA. **/
        if (lh5028->reg44_LY >= 0 && lh5028->reg44_LY <= 143 && lh5028->hdma_gen && lh5028->hdma_r16)  {
          /* copy 16 bytes  */
          uint16_t c;
          for (c =0; c != 16; c++) {
            uint8_t s;
            s = gameboy_read (lh5028->gb, lh5028->hdma_src + c);
            gameboy_write (lh5028->gb, lh5028->hdma_dst + c, s);
          }
          lh5028->hdma_src += 16;
          lh5028->hdma_dst += 16;
          lh5028->hdma_r16 --;
          /* add hdma cycles, ~ 8us */
          lh5028->gb->cpu_clks_dma += (lh5028->gb->mach_tools->clk_ns * 8.0);
        }
        /* in HDMA, no hblank perido */
        lh5028->reg41_IRQf &= ~LCDS_INTERRUPET_HBLANK_MASK;
      }
      /* check hblank interrupt  */
      if ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
         && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_INTERRUPET_HBLANK_MASK)) {
         lh5028->reg41_IRQf &= ~LCDS_INTERRUPET_HBLANK_MASK;
         lh5028->gb->reg0F_IF |= IRQ_2;
      }
    } else if (clkline > lh5028->gb->mach_tools->oambg_clk_st) {
      /* ------------------------------ MODE3-OAMBG ------------------------------ */
      clkline -= lh5028->gb->mach_tools->oambg_clk_st; // sub, get start clcks epls.
      lh5028->reg41_LCDS |= LCDS_MODE_FLAG_SERACH_OAMVRAM;
      /* check edge */
      if (lh5028->reg41_LCDM_T != LCDS_MODE_FLAG_SERACH_OAMVRAM) {
        lh5028->sprite_done (lh5028, 10);
        /*  check sprite interrupt  */    
        if ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
           && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_INTERRUPET_OAM_MASK))
          lh5028->gb->reg0F_IF |= IRQ_2;
        /* adjust BGOAM clk */
        lh5028->hbl_clks_st = (lh5028->gb->mach_tools->oambg_b_cycles + 
              lh5028->vscanR * lh5028->gb->mach_tools->oam_clk_add_hbl_per);
        lh5028->oambg_clks_divider21 = 
            lh5028->hbl_clks_st / 21.0;
        lh5028->hbl_clks_st += lh5028->gb->mach_tools->oam_cycles;
        lh5028->reg41_IRQf |= (LCDS_INTERRUPET_OAM_MASK | LCDS_INTERRUPET_HBLANK_MASK | LCDS_INTERRUPET_VBLANK_MASK);
      }
      /* check render bg *.*/
      if ((interv = (int16_t) (clkline/ lh5028->oambg_clks_divider21)) != lh5028->uscan
        && ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK))) {
        delta = interv - lh5028->uscan;
        lh5028->uscan += delta;
        do { 
          lh5028->bgwin_done (lh5028, scanline); 
          lh5028->uscanR ++;
        }  while (--delta);
      } 
    } else   {
      /* ------------------------------ MODE2-OAM ------------------------------ */
      lh5028->reg41_LCDS |= LCDS_MODE_FLAG_SERACH_OAM;
      lh5028->hbl_clks_st = 571583.0;
      lh5028->reg40_NMIf = 1;
      /* check edge */
      if (lh5028->reg41_LCDM_T == LCDS_MODE_FLAG_HBLANK) {
        /* hblank to current, (low to high), check stride interrupt */
        if ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
           && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_MODE_FLAG_HBLANK))
         lh5028->gb->reg0F_IF |= IRQ_2;
        lh5028->reg41_IRQf |= (LCDS_INTERRUPET_HBLANK_MASK | LCDS_INTERRUPET_VBLANK_MASK);
        /* clear sprite ctx */
        if (lh5028->pixel_format == PPU_SAMPLE_RGB15)
          memset (& lh5028->olf16[0], 0, sizeof (lh5028->olf16));
        else 
          memset (& lh5028->olf32[0], 0, sizeof (lh5028->olf32));
        // DEBUG_OUT ("sprite line debug %d\n", lh5028->reg44_LY);
      } else if (lh5028->reg41_LCDM_T == LCDS_MODE_FLAG_VLANK) {
        /* vblank to current, (low to high), check stride interrupt */
        lh5028->reg41_IRQf |= (LCDS_INTERRUPET_HBLANK_MASK | LCDS_INTERRUPET_VBLANK_MASK);
        /* clear sprite ctx */
        if (lh5028->pixel_format == PPU_SAMPLE_RGB15)
          memset (& lh5028->olf16[0], 0, sizeof (lh5028->olf16));
        else 
          memset (& lh5028->olf32[0], 0, sizeof (lh5028->olf32));
      }
      /* check render oam *.*/
      if ( (lh5028->reg40_LCDC & LCDC_OAM_MASK)
        && lh5028->vscan != (interv = (int16_t)  (clkline/ lh5028->gb->mach_tools->oam_clk_pick_per))) {
        delta = interv - lh5028->vscan;
        lh5028->vscan += delta;
        /* render sprite */
        lh5028->sprite_done (lh5028, delta);
      }
      /*  check sprite interrupt  */
      if (  (lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
         && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_INTERRUPET_OAM_MASK)) {
         lh5028->reg41_IRQf &= ~LCDS_INTERRUPET_OAM_MASK;
         lh5028->gb->reg0F_IF |= IRQ_2;
      }  
    }
  } else {
    /* ------------------------------ MODE1-VLANK ------------------------------ */
    lh5028->reg41_LCDS |= LCDS_MODE_FLAG_VLANK;
    /* check edge */
    if (lh5028->reg41_LCDM_T != LCDS_MODE_FLAG_VLANK) {
      /* post device render */
      lh5028->device_blit (lh5028, lh5028->obj, & lh5028->fmebuf);
      /* check stride */
      if ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
         && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_INTERRUPET_HBLANK_MASK))
       lh5028->gb->reg0F_IF |= IRQ_2;
      lh5028->reg4A_WYRSC = 0;
      lh5028->reg41_IRQf |= (LCDS_INTERRUPET_HBLANK_MASK | LCDS_INTERRUPET_OAM_MASK);
    }
    if (lh5028->reg40_NMIf != 0 && (lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)) {
      lh5028->reg40_NMIf = 0;
      lh5028->gb->reg0F_IF |= IRQ_1;
    }
    if ((lh5028->reg40_LCDC & LCDC_DISPLAY_MASK)
        && (lh5028->reg41_IRQf & lh5028->reg41_LCDS & LCDS_INTERRUPET_VBLANK_MASK)) {
      lh5028->reg41_IRQf &= ~LCDS_INTERRUPET_VBLANK_MASK;
      lh5028->gb->reg0F_IF |= IRQ_2;
    } 
  }
  lh5028->reg44_LY_T = lh5028->reg44_LY;
  lh5028->reg41_LCDM_T = lh5028->reg41_LCDS & LCDS_MODE_FLAG_ALL_MASK;
}

int ppu_cgb_mode (struct ppu *lh5028) {
  lh5028->bgwin_done = bgwin_render_cgb;
  lh5028->sprite_done = sprite_render_cgb;
  return 0;
}
int ppu_dmg_mode (struct ppu *lh5028) {
  lh5028->bgwin_done = bgwin_render_dmg;
  lh5028->sprite_done = sprite_render_dmg;
  return 0;
}

// int ppu_set_dmg_palette (struct ppu *lh5028, 


void ppu_reset_pixel_format (struct ppu *lh5028, PPU_SAMPLE_FORMAT format) {

  if ((lh5028->pixel_format = format) == PPU_SAMPLE_RGB15) {
    lh5028->fmebuf.buf = (uint16_t *)(((((intptr_t)& lh5028->bufb16[0]) + 31) & -32) + 16);
    lh5028->fmebuf.pitch = 512;
  } else if (format == PPU_SAMPLE_RGB24) {
    lh5028->fmebuf.buf32 = (uint32_t *)(((((intptr_t)& lh5028->bufb32[0]) + 31) & -32) + 16);
    lh5028->fmebuf.pitch = 1024;
  } else {
    assert (0);
  }
}

void ppu_reset_dmgpal (struct ppu *lh5028, struct ppu_dmg_palette32 *pal) {
  int setid = (int) pal;
  switch (setid) {
  case PPU_DMG_PAL_WHITE:
    memcpy (& lh5028->dmg_pal16T, & dmgcol_white, sizeof (dmgcol_white));
    break;
  case PPU_DMG_PAL_BRONW:
    memcpy (& lh5028->dmg_pal16T, & dmgcol_brown, sizeof (dmgcol_brown));
    break;
  case PPU_DMG_PAL_CYAN:
    memcpy (& lh5028->dmg_pal16T, & dmgcol_cyan, sizeof (dmgcol_cyan));
    break;
  default:
    memcpy (& lh5028->dmg_pal32T, pal, sizeof (*pal));
    set_dmgpal16_bydmgpal32 (lh5028);
    update_dmgpal_full (lh5028);
    return;
  }
  set_dmgpal32_bydmgpal16 (lh5028);
  update_dmgpal_full (lh5028);
}
void ppu_get_dmgpal (struct ppu *lh5028, struct ppu_dmg_palette32 *pal) {
  memcpy (pal, & lh5028->dmg_pal32T, sizeof (lh5028->dmg_pal32T));
}
int ppu_init (struct ppu **lh5028) {

  struct ppu *sys = null;
  assert (lh5028 != null);

  sys = (struct ppu *)calloc (sizeof (struct ppu), 1);
  assert (sys != null);
  *lh5028 = sys;

  sys->fmebuf.w = 160;
  sys->fmebuf.h = 144;
  sys->bufb16 = (uint8_t *)malloc ( (sys->fmebuf.w + 96) * (sys->fmebuf.h + 16) * 2 + 128);
  sys->bufb32 = (uint8_t *)malloc ( (sys->fmebuf.w + 96) * (sys->fmebuf.h + 16) * 4 + 128);

  dmgpal_init ();
  sys->device_blit = default_update;
  sys->clks = ticks;
  sys->reg41_LCDM_T =LCDS_MODE_FLAG_SERACH_OAM;

  sys->dmg_pal16.bg_pal[0] = sys->dmg_pal16T.bg_pal[(0xE8 & 0x03) >> 0];
  sys->dmg_pal16.bg_pal[1] = sys->dmg_pal16T.bg_pal[(0xE8 & 0x0C) >> 2];
  sys->dmg_pal16.bg_pal[2] = sys->dmg_pal16T.bg_pal[(0xE8 & 0x30) >> 4];
  sys->dmg_pal16.bg_pal[3] = sys->dmg_pal16T.bg_pal[(0xE8 & 0xC0) >> 6];
  sys->reg47_BGP = 0xE8;

  ppu_reset_dmgpal (sys, (struct ppu_dmg_palette32 *)PPU_DMG_PAL_CYAN);
  ppu_reset_pixel_format (sys, PPU_SAMPLE_DEFAULT);

  return 0;
}

