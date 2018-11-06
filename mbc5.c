/* MAPPER5 It's actually mapper4, because the Japanese are very annoying with the number 4.
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
  /* nodone in device mapper5*/
}

static 
void  write (struct cartridge *cartridge, uint16_t address, uint8_t value) {
  struct mbc5_chip *chip = (struct mbc5_chip *)cartridge->ubdata_user;
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
    if (address <= 0x2FFF) 
      chip->prombank = (chip->prombank & 0x100) | value; /* low 8 bit mapper */
    else 
      chip->prombank = (chip->prombank &~0x100) | ((value & 1) * 256);/* high 1 bit mapper */
    break;
  case 2: /* 0x4000-0x5FFF RAM Bank Number (Write Only)*/
    chip->srambank = value & 15;
    break;
  }
}

static 
uint8_t read (struct cartridge *cartridge, uint16_t address) {
  struct mbc5_chip *chip = (struct mbc5_chip *)cartridge->ubdata_user;
  switch (address >> 13)  {
  case 0: /* 0x0000-0x1FFF */
  case 1: /* 0x2000-0x3FFF  Bank0 - 16KB ROM bank 00 */
    return cartridge->promworks[address];
  case 2: /* 0x4000-0x5FFF */
  case 3: /* 0x6000-0x7FFF Bank N-  XXX: mbc1 chip's 20h, 40h, and 60h cannot be used*/
    // printf ("pc:%04x prombank:%d\n", cartridge->gb->lr35902->PC, chip->prombank);
    return cartridge->promworks[chip->prombank * 0x4000+(address - 0x4000)];
  case 5: /* 0xA000-0xBFFF - SRAM or ext device */
    /* RAM Bank 00-0F, if any (Read/Write) */
    if (cartridge->sramworks != null 
     && cartridge->sramsize !=0
     && chip->ram_en != false)
     return cartridge->sramworks[chip->srambank *0x2000+address-0xA000];
  default: 
    return cartridge->gb->unknow_ram[address];
  }
}

void cartridge_mbc5_uninit (struct cartridge *cartridge) {
  free (cartridge->ubdata_user);
  cartridge->ubdata_user = null;
}

int cartridge_mbc5_init (struct cartridge *cartridge) {
  struct mbc5_chip *chip;
  cartridge->read = read;
  cartridge->write = write;
  cartridge->clks = clks;
  chip = (struct mbc5_chip *)malloc (sizeof (struct mbc5_chip));
  chip->bankcac =0;
  chip->mode = MBC1_MODE0_2MROM_8KRAM;
  chip->srambank = 0;
  chip->ram_en = false;
  chip->prombank = 1;
  cartridge->ubdata_user = chip;
  return 0;
}