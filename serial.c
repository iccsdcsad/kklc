/* Serial device for gameboy
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

void serial_write (struct serial *serial, uint16_t addr, uint8_t value) {
}

uint8_t serial_read (struct serial *serial, uint16_t addr) {
  return 0;
}

static 
void serial_update (struct serial *serial) {
}

int serial_init (struct serial **serial) {
  struct serial *serial_ =null;
  assert (serial != null);

  serial_ = (struct serial *)
     calloc (sizeof (struct serial), 1);
  serial_->clks = serial_update;
  assert (serial_ != null);
  * serial = serial_;
  return 0;
}

void serial_uninit (struct serial **serial) {
  struct serial *serial_;
  assert (serial != null);
  serial_ = *serial;
  *serial = null;
  if (serial_ != null)
    free (serial_);
  else ;
}