/* dmg/cgb cartidge 
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

/* Add mapper --*/
int cartridge_load (struct cartridge *cart, FILE *cartmem) {

  uintptr_t sig = -1;
  uintptr_t ctype;
  uintptr_t calc;
  uintptr_t promsize;
  uintptr_t sramsize;
  uint8_t *promworks = null;
  uint8_t *sramworks = null;
  uint8_t nintendo_magicnumber[] =
  { 0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83, 0x00, 0x0C, 0x00, 0x0D,
    0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E, 0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99,
    0xBB, 0xBB, 0x67, 0x63, 0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E };
  uint8_t magicnumber_buffer[sizeof(nintendo_magicnumber)];
  bool battery = true;
  struct romheader rh;

  int cartridge_mbc0_init (struct cartridge *cartridge);
  int cartridge_mbc1_init (struct cartridge *cartridge);
  int cartridge_mbc2_init (struct cartridge *cartridge);
  int cartridge_mbc5_init (struct cartridge *cartridge);
  void cpu_reset (struct cpu *cpu);
  int ppu_reset (struct ppu *ppu);
  int ppu_cgb_mode (struct ppu *ppu);
  int ppu_dmg_mode (struct ppu *ppu);

  if (fseek (cartmem, 0x104, SEEK_SET) != 0)
    return -1;
  if (fread (& magicnumber_buffer[0], sizeof (nintendo_magicnumber), 1, cartmem) != 1)
    return -1;
  if (memcmp (nintendo_magicnumber, 
                magicnumber_buffer, sizeof (magicnumber_buffer) != 0))
    return -1;
  if (fseek (cartmem, 0x134, SEEK_SET) != 0)
    return -1;
  if (fread (& rh, sizeof (rh), 1, cartmem)  != 1)
    return -1;

  /* 
   00h  ROM ONLY                 19h  MBC5
   01h  MBC1                     1Ah  MBC5+RAM
   02h  MBC1+RAM                 1Bh  MBC5+RAM+BATTERY
   03h  MBC1+RAM+BATTERY         1Ch  MBC5+RUMBLE
   05h  MBC2                     1Dh  MBC5+RUMBLE+RAM
   06h  MBC2+BATTERY             1Eh  MBC5+RUMBLE+RAM+BATTERY
   08h  ROM+RAM                  20h  MBC6
   09h  ROM+RAM+BATTERY          22h  MBC7+SENSOR+RUMBLE+RAM+BATTERY
   0Bh  MMM01
   0Ch  MMM01+RAM
   0Dh  MMM01+RAM+BATTERY
   0Fh  MBC3+TIMER+BATTERY
   10h  MBC3+TIMER+RAM+BATTERY   FCh  POCKET CAMERA
   11h  MBC3                     FDh  BANDAI TAMA5
   12h  MBC3+RAM                 FEh  HuC3
   13h  MBC3+RAM+BATTERY         FFh  HuC1+RAM+BATTERY
 */ 

  /* check error device */
  switch (rh.ctype) {
  case 0x00: 
    ctype = MBC_0;
    break;
  case 0x01: 
  case 0x02: 
  case 0x03:
    ctype = MBC_1;
    break;
  case 0x05: 
  case 0x06: 
    ctype = MBC_2;
    break;
  case 0x08: 
  case 0x09:
    ctype = MBC_0;
    break;
  case 0x0B: 
  case 0x0C: 
  case 0x0D: 
    ctype = MMM0;
    break;
  case 0x0F:
  case 0x10: 
  case 0x11: 
  case 0x12: 
  case 0x13:
    ctype = MBC_3;
    break;
  case 0x19: 
  case 0x1A:
  case 0x1B:
  case 0x1C:
  case 0x1D: 
  case 0x1E:
    ctype = MBC_5;
    break;  
  case 0x20:
    ctype = MBC_6;
    break;  
  case 0x22:
    ctype = MBC_7;
    break;  
  case 0xFC: 
    ctype = POCKER_CAM;
    break;  
  case 0xFD:
    ctype = TAMA5;
    break;
  case 0xFE:
    ctype = HUCL3;
    break;
  case 0xFF:
    ctype = HUCL1;
    break;
  default : 
    return -1;
  }
  /* check prom size */
  switch (rh.promsize) {
  case 0x00: promsize = 2; break;
  case 0x01: promsize = 4; break;
  case 0x02: promsize = 8; break;
  case 0x03: promsize =16; break;
  case 0x04: promsize =32; break;
  case 0x05: promsize =64; break;
  case 0x06: promsize =128; break; break;
  case 0x07: promsize =256; break;
  case 0x08: promsize =512; break;
  case 0x52: promsize =72; break;
  case 0x53: promsize =80; break;
  case 0x54: promsize =96; break;
  default : return -1;
  }
  
  /* check sram size */
  switch (rh.ramsize) {
  case 0x00: sramsize = 0; break;
  case 0x01: sramsize = 1; break; /* 2K sram, same as 8K */
  case 0x02: sramsize = 1; break;
  case 0x03: sramsize = 4; break;
  case 0x04: sramsize =16; break;
  case 0x05: sramsize = 8; break;
  default : return -1;
  }
  /* init prom and sram */
  promworks = (uint8_t *)malloc (promsize * 0x4000);
  sramworks = (uint8_t *)malloc (sramsize * 0x2000);
  assert (promworks != null);
  assert (sramworks != null); /* random data in init time */

  fseek (cartmem, 0, SEEK_END);
  calc = ftell(cartmem);

  if (calc != promsize * 0x4000)
    promsize = calc / 0x4000;
  if (fseek (cartmem, 0, SEEK_SET) != 0)
    goto _cleanup;
  if (fread (promworks, promsize * 0x4000, 1, cartmem) != 1)
    goto _cleanup;
  if (cart->promworks != null)
    free (cart->promworks);
  if (cart->sramworks != null)
    free (cart->sramworks);

  cart->promworks = promworks;
  cart->sramworks = sramworks;
  cart->promsize = promsize;
  cart->sramsize = sramsize;
  cart->battery = battery;

  memcpy (& cart->header, & rh, sizeof (rh));

  /* reset cart chip device */
  switch (ctype) {
  case MBC_0:
    cartridge_mbc0_init (cart);
    break;
  case MBC_1:
    cartridge_mbc1_init (cart);
    break;
  case MBC_2:
  case MBC_3:
  case MBC_4:
  case MBC_5:
    cartridge_mbc5_init (cart);
    break;
  case MBC_6:
  case MBC_7:
  case TAMA5:
  case HUCL1:
  case HUCL3:
  case MMM0:
  case POCKER_CAM:
  default :assert (0); /*never reach here */
  } 
  if (rh.title[15] & 0x80) 
     ppu_cgb_mode (cart->gb->lh5028);
  else 
    ppu_dmg_mode (cart->gb->lh5028);
  cpu_reset (cart->gb->lr35902);
  ppu_reset (cart->gb->lh5028);
  /* set std freq. */
  cart->gb->mach_tools = (struct machine_setup *)& std_machine;
  return 0;
_cleanup:
  free (sramworks);
  free (promworks);
  return -1;
}

int cartridge_init (struct cartridge **cartridge) {
  struct cartridge *cartridge_ =null;
  assert (cartridge != null);

  cartridge_ = (struct cartridge *)
     calloc (sizeof (struct cartridge), 1);
  cartridge_->clks = null;
  assert (cartridge_ != null);
  * cartridge = cartridge_;

  return 0;
}

void cartridge_uninit (struct cartridge **cartridge) {
  struct cartridge *cartridge_;
  assert (cartridge != null);
  cartridge_ = *cartridge;
  *cartridge = null;
  if (cartridge_ != null)
    free (cartridge_);
  else ;
}