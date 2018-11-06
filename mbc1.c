/* MAPPER1
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
  /* nodone in device mapper1*/
}

static 
void  write (struct cartridge *cartridge, uint16_t address, uint8_t value) {
  struct mbc1_chip *chip = (struct mbc1_chip *)cartridge->ubdata_user;
  switch (address >> 13)  {
  case 5: /* 0xA000-0xBFFF - SRAM or ext device */
    if (cartridge->sramworks != null 
     && cartridge->sramsize !=0)
     cartridge->sramworks[chip->srambank *0x2000+address-0xA000] = value;
    else 
  default:
    cartridge->gb->unknow_ram[address] = value;
    break;
  case 0: /* 0x0000-0x1FFF RAM Enable (Write Only) */
    if ((value & 0x0A) == 0x0A)
      chip->ram_en = true;
    else 
      chip->ram_en = false;
    break;
  case 1: /* 0x2000-0x3FFF  ROM Bank Number (Write Only) */
    /* Writing to this address space selects the lower 5 bits of the ROM Bank Number (in range 01-1Fh).
         When 00h is written, the MBC translates that to bank 01h also. 
        That doesn't harm so far, because ROM Bank 00h can be always 
          directly accessed by reading from 0000-3FFF. 
            But (when using the register below to specify the upper ROM Bank bits), 
          the same happens for Bank 20h, 40h, and 60h. Any attempt to address these ROM Banks 
           will select Bank 21h, 41h, and 61h instead. */
    if (chip->mode == MBC1_MODE1_512KROM_32KRAM)
      chip->prombank = value & 0x1F;
    else 
      chip->prombank = (chip->prombank & ~0x1F) | (value & 0x1F);
    if ((chip->prombank & ~0x60) == 0) /* check 0x00, 0x20, 0x40, 0x60 bank */
      chip->prombank++;
    break;
  case 2: /* 0x4000-0x5FFF RAM Bank Number - or - Upper Bits of ROM Bank Number (Write Only)*/
    if (chip->mode == MBC1_MODE1_512KROM_32KRAM)
      chip->srambank = value & 3;
    else {
      chip->prombank = (chip->prombank & ~0x60) | ((value & 0x3) << 5);
      if ((chip->prombank & ~0x60) == 0) /* check 0x00, 0x20, 0x40, 0x60 bank */
        chip->prombank++;
      else ;
    }
    break;
  case 3: /* 0x6000-0x7FFF - ROM/RAM Mode Select (Write Only)-  */
    if (value == 0)
      chip->mode = MBC1_MODE0_2MROM_8KRAM;
    else 
      chip->mode = MBC1_MODE1_512KROM_32KRAM;
    break;
  }
}

static 
uint8_t read (struct cartridge *cartridge, uint16_t address) {
  struct mbc1_chip *chip = (struct mbc1_chip *)cartridge->ubdata_user;
  switch (address >> 13)  {
  case 0: /* 0x0000-0x1FFF */
  case 1: /* 0x2000-0x3FFF  Bank0 - 16KB ROM bank 00 */
    return cartridge->promworks[address];
  case 2: /* 0x4000-0x5FFF */
  case 3: /* 0x6000-0x7FFF Bank N-  XXX: mbc1 chip's 20h, 40h, and 60h cannot be used*/
    // printf ("pc:%04x prombank:%d\n", cartridge->gb->lr35902->PC, chip->prombank);
    return cartridge->promworks[chip->prombank * 0x4000+(address - 0x4000)];
  case 5: /* 0xA000-0xBFFF - SRAM or ext device */
    /* RAM Bank 00-03, if any (Read/Write) */
    if (cartridge->sramworks != null 
     && cartridge->sramsize !=0
     && chip->ram_en != false)
     return cartridge->sramworks[chip->srambank *0x2000+address-0xA000];
  default: 
    return cartridge->gb->unknow_ram[address];
  }
}

void cartridge_mbc1_uninit (struct cartridge *cartridge) {
  free (cartridge->ubdata_user);
  cartridge->ubdata_user = null;
}

int cartridge_mbc1_init (struct cartridge *cartridge) {
  struct mbc1_chip *chip;
  cartridge->read = read;
  cartridge->write = write;
  cartridge->clks = clks;
  chip = (struct mbc1_chip *)malloc (sizeof (struct mbc1_chip));
  chip->bankcac =0;
  chip->mode = MBC1_MODE0_2MROM_8KRAM;
  chip->srambank = 0;
  chip->ram_en = false;
  chip->prombank = 1;
  cartridge->ubdata_user = chip;
  return 0;
}