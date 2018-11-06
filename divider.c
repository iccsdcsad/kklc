/* DIV - fixed frequency distributor
 * It's just 1/256 of the cpu frequency.
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

void divider_write (struct divider *divider, uint8_t value) {

  divider->reg04_DIV = 0;
}

uint8_t divider_read (struct divider *divider) {

  return divider->reg04_DIV;
}

static 
void ticks (struct divider *divider) {
  /* !must update gameboy's context about divider before this call */
  /*   double cpu_cldivider;
  double cpu_clel */
  while (divider->gb->cpu_clks_divider > 256.0) {
    /* update divider */
    divider->reg04_DIV++;
    divider->gb->cpu_clks_divider -= 256.0;
  }
}

int divider_init (struct divider **divider) {
  struct divider *divider_ =null;
  assert (divider != null);

  divider_ = (struct divider *)
     calloc (sizeof (struct divider), 1);
  divider_->clks = ticks;
  assert (divider_ != null);
  * divider = divider_;
  return 0;
}

void divider_uninit (struct divider **divider) {
  struct divider *divider_;
  assert (divider != null);
  divider_ = *divider;
  *divider = null;
  if (divider_ != null)
    free (divider_);
  else ;
}