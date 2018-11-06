/* CPU - LR35902
 * Sharp LR35902 Chip for GameBoy (base Z80)
 *
 * Sharp LR35902 Chip opcode mapper 
 * http://www.pastraiser.com/cpu/gameboy/gameboy_opcodes.html 
 * Z80 Chip opcode mapper 
 * http://clrhome.org/table/
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

int cpu_init (struct cpu **cpu) {
  struct cpu *cpu_ =null;
  assert (cpu != null);

  cpu_ = (struct cpu *)
     calloc (sizeof (struct cpu), 1);
  assert (cpu_ != null);
  * cpu = cpu_;
  return 0;
}

void cpu_uninit (struct cpu **cpu) {
  struct cpu *cpu_;
  assert (cpu != null);
  cpu_ = *cpu;
  *cpu = null;
  if (cpu_ != null)
    free (cpu_);
  else ;
}

void cpu_reset (struct cpu *cpu) {
  /*   AF=$01B0
  BC=$0013
  DE=$00D8
  HL=$014D
  Stack Pointer=$FFFE
  
  See. http://gbdev.gg8.se/wiki/articles/Power_Up_Sequence
  */
  /* this action is very important, 
        because some games will judge whether it is CGB 
     or not according to the initial flag bit.
     e.g konami::surival kids 1999
     */
  cpu->AF = cpu->gb->cart->header.title[15] & 0x80 ?  0x11B0:0x01B0;
  cpu->BC = 0x0013;
  cpu->HL = 0x014D;
  cpu->SP = 0xFFFE;
  /* When the GameBoy is powered up, 
    a 256 byte program starting at memory location 0 is executed. */
  cpu->PC = 0x0100; 
  cpu->IME = 0;
  cpu->halt = false;
  cpu->stop = false;
  cpu->_backup = 0;
}
