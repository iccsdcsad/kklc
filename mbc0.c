/* MAPPER0 - Standard 32K ROM.
 * 
 * Copyright (C) 2018 moecmks
 * This file is part of YuduliyaGB.
 * 
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 */

#include "inline.inl"

static 
void clks (struct cartridge *cartridge) {
  /* nodone in device mapper0*/
}

static 
void  write (struct cartridge *cartridge, uint16_t address, uint8_t value) {
  switch (address >> 13)  {
  case 5: /* 0xA000-0xBFFF - SRAM or ext device */
    if (cartridge->sramworks != null 
     && cartridge->sramsize !=0)
     cartridge->sramworks[cartridge->sramsize*0x2000+address-0xA000] = value;
    else 
  case 0: /* 0x0000-0x1FFF */
  case 1: /* 0x2000-0x3FFF  Bank0 - 16KB ROM bank 00 */
  case 2: /* 0x4000-0x5FFF */
  case 3: /* 0x6000-0x7FFF Bank N-  */
  default:
    cartridge->gb->unknow_ram[address] = value;
    break;
  }
}

static 
uint8_t read (struct cartridge *cartridge, uint16_t address) {
  switch (address >> 13)  {
  case 0: /* 0x0000-0x1FFF */
  case 1: /* 0x2000-0x3FFF  Bank0 - 16KB ROM bank 00 */
  case 2: /* 0x4000-0x5FFF */
  case 3: /* 0x6000-0x7FFF Bank N-  */
    return cartridge->promworks[address];
  case 5: /* 0xA000-0xBFFF - SRAM or ext device */
    if (cartridge->sramworks != null 
     && cartridge->sramsize !=0)
     return cartridge->sramworks[cartridge->sramsize*0x2000+address-0xA000];
  default: 
    return cartridge->gb->unknow_ram[address];
  }
}

void cartridge_mbc0_uninit (struct cartridge *cartridge) {
  /* nodone */
}

int cartridge_mbc0_init (struct cartridge *cartridge) {
  cartridge->read = read;
  cartridge->write = write;
  cartridge->clks = clks;

  return 0;
}