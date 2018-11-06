/* controller (PL0-PL5, with DAN215)
 * Game boy's joypad information read and write
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

/* for internal call */
#define joypad_hostdrv ubdata_user

/* Assert IRQ_5 Interrupt Gen MACRO */
#ifndef IRQ_5
# define IRQ_5 0x10
#endif 

#ifndef JOYPAD_INTERRUPT_GEN_ENABLE
# undef IRQ_5
# define IRQ_5 0
#endif 

static 
void default_update (struct controller *ctl, void *obj, struct controller_pad *gb_infos, struct controller_pad *host_infos) {
  DEBUG_OUT ("%s:%d please set joypad callback( controller_setupdate function)\n", __FILE__, __LINE__);
  DEBUG_BREAK();
}

void controller_write (struct controller *ctl, uint8_t value) {
  // ctl->reg00_P1 = value & 0x30;
  ctl->reg00_P1 = (ctl->reg00_P1 & ~0x30) | (value & 0x30);
}

/* Gameboy Joypad Infos:http://gbdev.gg8.se/files/docs/mirrors/pandocs.html#joypadinput */
uint8_t controller_read (struct controller *ctl) {
  uint8_t value =ctl->reg00_P1 & 0x30;
  switch (value) {
  case 0x00: /* maybe result direction | action ??*/
    value = ctl->reg00_P1 & 0xF0;
    value|= 0x0F;
#if (JOYPAD_LOAD_ACTION_IN_P1415_ALLOPEN == 0)
    if (ctl->gb_pad.right || ctl->gb_pad.a) value &= ~0x01; /*Right ||A*/
    if (ctl->gb_pad.left || ctl->gb_pad.b) value &= ~0x02; /*Left ||B */
    if (ctl->gb_pad.up || ctl->gb_pad.select) value &= ~0x04; /*Up ||Select */
    if (ctl->gb_pad.down || ctl->gb_pad.start) value &= ~0x08; /*Down || Start*/
#elif (JOYPAD_LOAD_ACTION_IN_P1415_ALLOPEN == 1)
    if (ctl->gb_pad.right && ctl->gb_pad.a) value &= ~0x01; /*Right &&A*/
    if (ctl->gb_pad.left && ctl->gb_pad.b) value &= ~0x02; /*Left &&B */
    if (ctl->gb_pad.up && ctl->gb_pad.select) value &= ~0x04; /*Up &&Select */
    if (ctl->gb_pad.down && ctl->gb_pad.start) value &= ~0x08; /*Down && Start*/
#elif (JOYPAD_LOAD_ACTION_IN_P1415_ALLOPEN == 2)
    if ((!!ctl->gb_pad.right) ^ !!ctl->gb_pad.a) value &= ~0x01; /*Right ^A*/
    if ((!!ctl->gb_pad.left) ^ !!ctl->gb_pad.b) value &= ~0x02; /*Left ^B */
    if ((!!ctl->gb_pad.up) ^ !!ctl->gb_pad.select) value &= ~0x04; /*Up ^Select */
    if ((!!ctl->gb_pad.down) ^ !!ctl->gb_pad.start) value &= ~0x08; /*Down ^ Start*/
#elif (JOYPAD_LOAD_ACTION_IN_P1415_ALLOPEN == 3)
    if (!(ctl->gb_pad.right && ctl->gb_pad.a)) value &= ~0x01; /*!(Right &&A)*/
    if (!(ctl->gb_pad.left && ctl->gb_pad.b)) value &= ~0x02; /*!(Left &&B)*/ 
    if (!(ctl->gb_pad.up && ctl->gb_pad.select)) value &= ~0x04; /*!(Up &&Select)*/ 
    if (!(ctl->gb_pad.down && ctl->gb_pad.start)) value &= ~0x08; /*!(Down &&Start)*/ 
#else 
# error "JOYPAD_LOAD_ACTION_IN_P1415_ALLOPEN define error."
#endif 
    break;
  case 0x30: /* reset device ?*/
    return 0xFF;
  case 0x20: /* p14 out, set button, active low*/
    value = ctl->reg00_P1 & 0xF0;
    value|= 0x0F;
    if (ctl->gb_pad.right) value &= ~0x01; /*Right */
    if (ctl->gb_pad.left) value &= ~0x02; /*Left */
    if (ctl->gb_pad.up) value &= ~0x04; /*Up */
    if (ctl->gb_pad.down) value &= ~0x08; /*Down */
    break;
  case 0x10: /* p15 out, set direction, active low */
    value = ctl->reg00_P1 & 0xF0;
    value|= 0x0F;
    if (ctl->gb_pad.a) value &= ~0x01; /*A */
    if (ctl->gb_pad.b) value &= ~0x02; /*B */
    if (ctl->gb_pad.select) value &= ~0x04; /*Select */
    if (ctl->gb_pad.start) value &= ~0x08; /*Start */
    break;
  }
  return value;
}

void controller_uninit (struct controller **ctl) {
  struct controller *ctl_;
  assert (ctl != null);
  ctl_ = *ctl;
  *ctl = null;
  if (ctl_ != null)
    free (ctl_);
  else ;
}

void controller_setupdate_ (struct controller *ctl, void (*update) 
             (struct controller *, 
             void *, 
                struct controller_pad *, /* self */ 
             struct controller_pad * /* host edge */), void *obj){
  assert (ctl != null);
  assert (update != null);
  ctl->joypad_hostdrv = update;
  ctl->obj = obj;
}

static
void ticks (struct controller *ctl) { 
  
  if (ctl->gb->cpu_clks_joypad > ctl->gb->mach_tools->joypad_gen_request_clks) {
    bool edge_gen =false;
    /* temrp cache for check edge (high to low*/
    struct controller_pad edge;
    assert (ctl != null);
    memset (& edge, 0, sizeof (edge));
    /*call hostdrv,  update joypad infos*/
    ((void (*)(struct controller *, void *, 
                  struct controller_pad *,
         struct controller_pad *))(ctl->joypad_hostdrv)) (ctl, ctl->obj, & ctl->gb_pad, & edge);
    ctl->gb->cpu_clks_joypad -= ctl->gb->mach_tools->joypad_gen_request_clks; /* sub a clk block */
    /* there is a bouncing phenomenon similar to electrical components flutter in real gameboy,
       making joypad interrupt very difficult to use. */
    /* check register */
    switch (ctl->reg00_P1 & 0x30) {
    case 0x30: /* close device*/
      return ;
    case 0x00: /* any button press will cause interrupt requested, resume stop command */
      if (ctl->gb_pad.right || ctl->gb_pad.a) edge_gen = true; /*Right ||A*/
      if (ctl->gb_pad.left || ctl->gb_pad.b) edge_gen = true; /*Left ||B */
      if (ctl->gb_pad.up || ctl->gb_pad.select) edge_gen = true; /*Up ||Select */
      if (ctl->gb_pad.down || ctl->gb_pad.start) edge_gen = true; /*Down || Start*/
    case 0x20: /* p14 out, button, */
      if (edge.right) edge_gen = true; /*Right */
      if (edge.left) edge_gen = true; /*Left */
      if (edge.up) edge_gen = true; /*Up */
      if (edge.down) edge_gen = true; /*Down */
      break;
    case 0x10: /* p15 out, direction */
      if (edge.a) edge_gen = true; /*A */
      if (edge.b) edge_gen = true; /*B */
      if (edge.select) edge_gen = true; /*Select */
      if (edge.start) edge_gen = true; /*Start */
      break;
    }
    if (edge_gen != false) {
      /* Set interrupt flags */
  #   if 0
      /* try resume gameboy from stop,  (if stop) */
      if (ctl->gb->lr35902->stop != false) {
        ctl->gb->lr35902->stop = false;
        ctl->gb->reg0F_IF |= IRQ_5;
      }
  #   else 
      /* IRQ_5 is the lowest priority interruption.*/
      ctl->gb->reg0F_IF |= IRQ_5;
      /* try resume gameboy from stop,  (if stop) */
      // gameboy_resume_ifstop (ctl->gb);
  #   endif 
    }
  }
}

int controller_init (struct controller **ctl) {
  struct controller *ctl_ =null;
  assert (ctl != null);

  ctl_ = (struct controller *)
     calloc (sizeof (struct controller), 1);
  ctl_->joypad_hostdrv = default_update;
  ctl_->obj = null;
  ctl_->clks = ticks;
  assert (ctl_ != null);
  // memset (& ctl_->gb_pad, 0x01, sizeof (ctl_->gb_pad));
  * ctl = ctl_;
  return 0;
}