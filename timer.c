/* timer about TIMA, TMA, TAC
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

void timer_write (struct timer *timer, uint16_t addr, uint8_t value) {

  switch (addr) {
  case 0xFF05: /* TIMA */
    timer->reg05_TIMA = value;
    break;
  case 0xFF06: /* TMA */
    timer->reg06_TMA = value;
    break;
  case 0xFF07: /* TAC */
    if ( (value & 0x04) && !(timer->reg07_TAC & 0x04)) {
      timer->gb->cpu_clks_timer = 0.0; /* low to high, open pulse, init timestamp. */
      // timer->reg05_TIMA = timer->reg06_TMA;
      timer->gb->cpu_clks_timerdbg = timer->gb->cpu_clks_total;
    }/* else if (!(value & 0x04) && (timer->reg07_TAC & 0x04)) {
      timer->gb->reg0F_IF &= ~IRQ_3;
    } */
    timer->reg07_TAC = value;
    break;
  default:
    assert(0);
  } 
}

uint8_t timer_read (struct timer *timer, uint16_t addr) {

  switch (addr) {
  case 0xFF05: /* TIMA */
    return timer->reg05_TIMA;
  case 0xFF06: /* TMA */
    return timer->reg06_TMA;
  case 0xFF07: /* TAC */
    return timer->reg07_TAC;
  default:
    assert(0);
  }
  return 0;
}

static 
void ticks (struct timer *timer) {
  /* !must update gameboy's context about timer before this call */
  /*   double cpu_clks_timer;
  double cpu_clks_el */
  double clks_t;
  assert (timer != null);
  /* enable timing???*/
  if ( !(timer->reg07_TAC & 0x04))
     return  ;
  else ;

  /* convert to basic frequency of timer */
  clks_t = timer->request_clks[timer->reg07_TAC & 3];

  while (timer->gb->cpu_clks_timer > clks_t) {
    /* update timer */
    if (timer->reg05_TIMA++ == 0xFF) {
      /* overflow, IRQ_3 interrupt will requested */
      timer->gb->cpu_clks_timerdbg = timer->gb->cpu_clks_total - timer->gb->cpu_clks_timerdbg;
      timer->reg05_TIMA = timer->reg06_TMA;
      timer->gb->reg0F_IF |= IRQ_3;   
      timer->gb->cpu_clks_timerdbg = timer->gb->cpu_clks_total;
      // _DEBUG_BREAK ();
    } 
    /* sub a freq block */
    timer->gb->cpu_clks_timer -= clks_t;
  }
}

void timer_uninit (struct timer **timer) {
  struct timer *timer_;
  assert (timer != null);
  timer_ = *timer;
  *timer = null;
  if (timer_ != null)
    free (timer_);
  else ;
}

int timer_init (struct timer **timer) {
  struct timer *timer_ = null;
  assert (timer != null);

  timer_ = (struct timer *)
     calloc (sizeof (struct timer), 1);
  timer_->clks = ticks;
  timer_->request_clks[0] = 1023.998;
  timer_->request_clks[1] = 15.998;
  timer_->request_clks[2] = 63.998;
  timer_->request_clks[3] = 255.998;
  assert (timer_ != null);
  * timer = timer_;
  return 0;
}