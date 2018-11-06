;; cpu_optick.s  (version2 for NASM Assembler)
;; Sharp LR35902 Chip Opcode for GameBoy 
;; 
;; Copyright (C) 2018 moecmks
;; This file is part of YuduliyaGB.
;; 
;; The contents of this file are subject to the Mozilla Public License Version
;; 1.1 (the "License"); you may not use this file except in compliance with
;; the License. You may obtain a copy of the License at
;; http://www.mozilla.org/MPL/
;; 

;; The Fleg Register consists of the following bits:
;; 7 6 5 4 3 2 1 0
;; Z N H C 0 0 0 0
Z_FLAG equ 0x80
N_FLAG equ 0x40
H_FLAG equ 0x20
C_FLAG equ 0x10

%if 1
extern _gameboy_read@8  ;; prototype uint8_t __stdcall _gameboy_mmu_read (void;;gameboy, ks_uint16 addresss)
extern _gameboy_write@12 ;; prototype void __stdcall _gameboy_mmu_write (void;;gameboy, ks_uint16 addresss, uint8_t value)
extern _gameboy_read2@8  ;; prototype uint8_t __stdcall _gameboy_mmu_read_w (void;;gameboy, ks_uint16 addresss)
extern _gameboy_write2@12 ;; prototype void __stdcall _gameboy_mmu_write_w (void;;gameboy, ks_uint16 addresss, uint8_t value)
%define MmuRead _gameboy_read@8
%define MmuReadWord _gameboy_read2@8
%define MmuWrite _gameboy_write@12
%define MmuWriteWord _gameboy_write2@12
%endif 

;;  define union .
%macro	defREG 2			; 1:low, 2:hi
  .%1: resb 1 
  .%2: resb 1
  .%2%1: equ .%1
%endmacro

%macro	defREG2 3			; 1:low, 2:hi 3:comb another.
  .%1: resb 1 
  .%2: resb 1
  .%3: equ .%1
%endmacro

struc lr35902                         ; structure definition       
  defREG F, A 
  defREG C, B
  defREG E, D
  defREG L, H
  defREG2 SP_LO, SP_HI, SP 
  defREG2 PC_LO, PC_HI, PC 
  
  .IME: resb 1
  .halt: resd 1 
  .stop: resd 1
  ._backup: resd 1
  .key1: resb 1
  .gameboy: resd 1 
endstruc

%macro xImm16_WriteReg 2 ;; eax:Imm16 %1 lr35902::reg %2:word or byte
  movzx ecx, %2 [YG_GP+lr35902.%1] 
  push ecx 
  push eax 
  push dword [YG_GP+lr35902.gameboy]
  call MmuWriteWord
%endmacro 
%macro xRegister_Read  1 
  mov ax, word [YG_GP+lr35902.%1]
  push eax
  push dword [YG_GP+lr35902.gameboy]
  call MmuRead
%endmacro  
%macro xImm16_WriteRegSP 0
  movzx ecx, word [YG_GP+lr35902.SP] 
  push ecx 
  push eax 
  push dword [YG_GP+lr35902.gameboy]
  call MmuWriteWord
%endmacro 
%macro xImm16_WriteRegA 0
  movzx ecx, byte [YG_GP+lr35902.A] 
  push ecx 
  push eax 
  push dword [YG_GP+lr35902.gameboy]
  call MmuWrite
%endmacro 
%macro xRegister_WriteReg 4 
  ;; eax:Register 
  ;; %1 lr35902::reg 
  ;; %2:word or byte
  ;; %3 Offset add 
  ;; %4 bit mask 0xFF or 0xFFFF 
  and eax, %4 
  add eax, %3 
  push eax 
  movzx eax, %2 [YG_GP+lr35902.%1] 
  push eax
  push dword [YG_GP+lr35902.gameboy]
  call MmuWrite
%endmacro 
%macro SetCyclesAndRet 1 ;; %1: burning cpu clk. 
  ;; write back PC 
  mov [YG_GP+lr35902.PC], si
  mov eax, %1
  jmp V_EXIT 
%endmacro         
%macro SetCyclesRetP 1  ;; %1: burning cpu clk. 
  ;; write back F 
  mov [YG_GP+lr35902.F], bl
  SetCyclesAndRet %1
%endmacro  
      
;;  Register access-read unwind 
%macro B_Read 0
  mov al, [YG_GP+lr35902.B]
%endmacro
%macro C_Read 0
  mov al, [YG_GP+lr35902.C]
%endmacro
%macro D_Read 0
  mov al, [YG_GP+lr35902.D]
%endmacro
%macro E_Read 0
  mov al, [YG_GP+lr35902.E]
%endmacro
%macro H_Read 0
  mov al, [YG_GP+lr35902.H]
%endmacro
%macro L_Read 0
  mov al, [YG_GP+lr35902.L]
%endmacro
%macro A_Read 0
  mov al, [YG_GP+lr35902.A]
%endmacro
%macro F_Read 0
  mov al, [YG_GP+lr35902.F]
%endmacro
%macro BC_Read 0
  mov ax, [YG_GP+lr35902.BC]
%endmacro
%macro DE_Read 0
  mov ax, [YG_GP+lr35902.DE]
%endmacro
%macro HL_Read 0
  mov ax, [YG_GP+lr35902.HL]
%endmacro
%macro AF_Read 0
  mov ax, [YG_GP+lr35902.AF]
%endmacro
%macro SP_Read 0
  mov ax, [YG_GP+lr35902.SP]
%endmacro
%macro xBC_Read 0
  xRegister_Read BC  
%endmacro
%macro xDE_Read 0
  xRegister_Read DE
%endmacro
%macro xHL_Read 0
  xRegister_Read HL
%endmacro
%macro xHL_Read_Inc 0
  xHL_Read
  inc word [YG_GP+lr35902.HL]
%endmacro  
%macro xHL_Read_Dec 0
  xHL_Read
  dec word [YG_GP+lr35902.HL]
%endmacro 
%macro xAF_Read 0
  xRegister_Read AF
%endmacro
%macro xPC_Read 0
  xRegister_Read PC 
%endmacro

;;  Register access-write unwind 
%macro B_Write 0
  mov [YG_GP+lr35902.B], al
%endmacro
%macro C_Write 0
  mov [YG_GP+lr35902.C], al
%endmacro
%macro D_Write 0
  mov [YG_GP+lr35902.D], al
%endmacro
%macro E_Write 0
  mov [YG_GP+lr35902.E], al
%endmacro
%macro H_Write 0
  mov [YG_GP+lr35902.H], al
%endmacro
%macro L_Write 0
  mov [YG_GP+lr35902.L], al
%endmacro
%macro A_Write 0
  mov [YG_GP+lr35902.A], al
%endmacro
%macro F_Write 0
  mov [YG_GP+lr35902.F], al
%endmacro
%macro BC_Write 0
  mov [YG_GP+lr35902.BC], ax
%endmacro
%macro DE_Write 0
  mov [YG_GP+lr35902.DE], ax
%endmacro
%macro HL_Write 0
  mov [YG_GP+lr35902.HL], ax
%endmacro
%macro AF_Write 0
  mov [YG_GP+lr35902.AF], ax
%endmacro
%macro SP_Write 0
  mov [YG_GP+lr35902.SP], ax
%endmacro
%macro  xBC_Write 0
  xRegister_WriteReg BC, word, 0, 0xFFFF  
%endmacro
%macro xDE_Write 0
  xRegister_WriteReg DE, word, 0, 0xFFFF
%endmacro
%macro xHL_Write 0
  xRegister_WriteReg HL, word, 0, 0xFFFF
%endmacro
%macro xHL_Write_Inc 0
  xRegister_WriteReg HL, word, 0, 0xFFFF
  inc word [YG_GP+lr35902.HL]
%endmacro
%macro xHL_Write_Dec 0
  xRegister_WriteReg HL, word, 0, 0xFFFF
  dec word [YG_GP+lr35902.HL]
%endmacro

%macro Imm8_Read 0
  push    esi 
  push    dword [YG_GP+lr35902.gameboy]
  call    MmuRead
  inc     esi 
%endmacro  
%macro Imm16_Read 0
  push    esi 
  push    dword [YG_GP+lr35902.gameboy]
  call    MmuReadWord
  add     esi, 2
%endmacro  
%macro Read_ByxX86SpecRegister  1
  ;;  %1:x86Register 
  push    %1 
  push    dword [YG_GP+lr35902.gameboy]
  call    MmuRead
%endmacro 
%macro Imm8Read_ExpandAddress16 0
  Imm8_Read 
  and eax, 000FFh
  add eax, 0FF00h 
%endmacro  
%macro Imm8_ExpandSignWord 0
  Imm8_Read 
  movsx eax, al 
%endmacro  
%macro C_ExpandAddress16 0
  movzx eax, byte [YG_GP+lr35902.C]
  add eax, 0FF00h 
%endmacro  
%macro Imm8Read_ExpandAddress16_Fetch 0
  Imm8Read_ExpandAddress16
  Read_ByxX86SpecRegister eax 
%endmacro  
%macro Imm16Read_Address_Fetch 0
  Imm16_Read
  Read_ByxX86SpecRegister eax 
%endmacro  
%macro C_ExpandAddress16_Fetch 0
  mov al, [YG_GP+lr35902.C]
  add eax, 0FF00h 
  Read_ByxX86SpecRegister eax 
%endmacro  
%macro EmptyMacro 0
%endmacro 

;; ld r(xHL), r(xHL)
%macro lr35902@LD@RxHLToRxHL  4 
 ;; %1 OpCase 
 ;; %2 Cycles_ 
 ;; %3 ReadOrExt
 ;; %4 WriteOrExt 
 %1:
    %3 
    %4 
    SetCyclesAndRet %2  
%endmacro

;; ld r, imm8/imm16
%macro lr35902@LD@Imm  4 
 ;; %1 OpCase 
 ;; %2 Immread 
 ;; %3 Cycles_
 ;; %4 WriteOrExt 
  %1: 
    %2 
    %4 
    SetCyclesAndRet %3 
%endmacro  

;; arith/logic main
%macro OP_Add$c_xHLrToA 1
;; %1: atomic_it 0(add) or C_FLAG(adc)
;; source value <- eax 
  and eax, 0xFF ;; Value &= 0xFF 
  and YG_PF, %1
  shr YG_PF, 5 ;; check c-flags 
  movzx edx, byte [YG_GP+lr35902.A]
  mov ecx, edx ;; temp WORD := A 
  adc ecx, eax ;; temp WORD := A + Value
  mov [YG_GP+lr35902.A], cl  ;; always write back A.
  test cl, cl 
  setz YG_PF_8 
  shl YG_PF_8, 7 ;; z flag set  
  xor dx, ax  
  mov ax, cx  
  xor ax, dx  ;; temp WORD:= temp WORD^(A ^Value)
  and ax, 0x10
  shl ax, 1 
  or YG_PF, eax   ;; h flag set  
  shl ch, 4 
  or YG_PF_8, ch  ;; c flag set  
%endmacro  

%macro AddWord_ 0
;; source <- eax 
;; target <- always register HL 

;; clear psb . save old Z 
  and YG_PF, Z_FLAG 
  movzx ecx, word [YG_GP+lr35902.HL]
  and eax, 0xFFFF 
  lea edx, [eax+ecx] 
  ;; always write back HL.
  mov word [YG_GP+lr35902.HL], dx 
  xor cx, ax 
  mov ax, dx 
  xor cx, ax 
  and cx, 01000h
  shl cx, 1
  or YG_PF_8, ch   ;; h flag set  
  and edx, 010000h
  shr edx, 12
  or YG_PF, edx ;; c flag set  
%endmacro  

%macro AddWord2_ 0
;; source <- eax 
;; target <- always register SP 
  movsx eax, al
  xor YG_PF, YG_PF 
  movzx ecx, word [YG_GP+lr35902.SP]
  and eax, 0xFFFF 
  lea edx, [eax+ecx] 
  ;; always write back HL.
  mov word [YG_GP+lr35902.SP], dx 
  xor cx, ax 
  mov ax, dx 
  xor cx, ax 
  and cx, 0x0110 ;; C|H 
  mov YG_PF_8, ch 
  shl YG_PF_8, 4
  shl cl, 1
  or YG_PF_8, cl 
  and YG_PF, (H_FLAG| C_FLAG)
%endmacro 

%macro OP_CmpSub$bc_xHLrToA 2
;; %1: atomic_it 0(sub) or C_FLAG(sbc)
;; %2: [YG_GP+lr35902.A] || cl for cmp opcode 
;; source value <- eax 
  and eax, 0xFF ;; Value &= 0xFF 
  and YG_PF, %1
  shr YG_PF, 5 ;; check c-flags 
  movzx edx, byte [YG_GP+lr35902.A]
  mov ecx, edx ;; temp WORD := A 
  sbb ecx, eax ;; temp WORD := A - Value
  mov %2, cl  ;; always write back A.
  test cl, cl
  setz YG_PF_8 
  shl YG_PF_8, 7 ;; z flag set  
  xor dx, ax  
  mov ax, cx  
  xor ax, dx  ;; temp WORD:= temp WORD^(A ^Value)
  and ax, 0x10
  shl ax, 1 
  or YG_PF, eax    ;; h flag set  
  and ecx, 0x8000 
  shr ecx, 11
  or YG_PF, ecx  ;; c flag set  
  or YG_PF, N_FLAG ;; n flag set  
%endmacro  
      
;; XOR | OR | AND do unwind base .
%macro OP_Logic_T 2
;; %1: initFlags
;; %2: LogicOp
;; source <- eax 
;; target <- always register A 
  mov YG_PF, %1
  movzx edx, byte [YG_GP+lr35902.A]
  %2 al, dl 
  ;; always write back A.
  mov [YG_GP+lr35902.A], al 
  setz al 
  shl eax, 7
  or YG_PF, eax ;; z flag set  
%endmacro   
     
;; unwind 
%macro Add_  0
  OP_Add$c_xHLrToA 0
%endmacro  

%macro Adc_ 0
  OP_Add$c_xHLrToA C_FLAG
%endmacro  
  
%macro Sub_ 0
  OP_CmpSub$bc_xHLrToA 0, [YG_GP+lr35902.A]
%endmacro    
      
%macro Sbc_ 0
  OP_CmpSub$bc_xHLrToA C_FLAG, [YG_GP+lr35902.A]
%endmacro        
 
%macro Cmp_ 0
  OP_CmpSub$bc_xHLrToA 0, cl
%endmacro   
      
%macro And_ 0
  OP_Logic_T H_FLAG, and
%endmacro    
      
%macro Xor_  0
  OP_Logic_T 0, xor
%endmacro        
 
%macro Or_ 0
  OP_Logic_T 0, or
%endmacro       

%macro DecWord_  0
  dec eax 
%endmacro  

%macro IncWord_  0
  inc eax 
%endmacro 

%macro Inc_  0 ;; ----------------------- DEC RxHL
;; source <- eax 
;; clear psb . save old C 
  and YG_PF, C_FLAG 
  add al, 1 
  setz dl 
  shl dl, 7 
  or YG_PF, edx ;; z flag set  
  test eax, 15 ;; 0xNF+1 := 0xC0 (C:= N+1)
  setz dl 
  shl dl, 5
  or YG_PF, edx ;; h flag set  
%endmacro  

%macro Dec_ 0  ;; ----------------------- INC RxHL
;; source <- eax 
;; clear psb . save old C 
  and YG_PF, C_FLAG 
  sub al, 1 
  setz dl 
  shl dl, 7 
  or YG_PF, edx ;; z flag set  
  lea ecx, [eax+1]
  test ecx, 15 ;; 0xN0-1 := 0xCF (C:= N-1)
  setz dl 
  shl dl, 5
  or YG_PF, edx ;; h flag set 
  or YG_PF, N_FLAG ;; n flag set 
%endmacro  

%macro lr35902@JP 3 ;; ------------------------- JMP/Jcc
 ;; %1 Opcode,
 ;; %2 Flags
 ;; %3 OpNOT 
   %1: 
      mov eax, YG_PF
      and eax, %2
      xor eax, %3 
      jne %%JP_skip 
      add esi, 2
      SetCyclesRetP 12
    %%JP_skip:
      Imm16_Read 
      mov esi, eax 
      SetCyclesRetP 16
%endmacro   

%macro lr35902@Rst 2 ;; ------------------------- Rst
;; %1 Opcode
;; %2 Vector
  %1:
    mov ax, si
    OP_PushWord
    mov si, %2
    SetCyclesAndRet 16
%endmacro   

%macro lr35902@JR  3 ;; -------------------------- JMP Short/Jcc Short 
;; %1 Opcode
;; %2 Flags
;; %3 OpNOT
   %1: 
      mov eax, YG_PF
      and eax, %2
      xor eax, %3 
      jne %%JR_skip 
      inc esi
      SetCyclesRetP 8
    %%JR_skip:
      Imm8_Read 
      movsx eax, al 
      add esi, eax 
      SetCyclesRetP 12
%endmacro 

%macro lr35902@CALL   3 ;; ------------------------ sub routine call 
;; %1 Opcode
;; %2 Flags
;; %3 OpNOT
   %1: 
      mov eax, YG_PF
      and eax, %2
      xor eax, %3 
      jne %%CALL_Skip 
      add esi, 2
      SetCyclesRetP 12
    %%CALL_Skip:
      lea eax, [YG_PC+2]
      OP_PushWord 
      Imm16_Read 
      mov esi, eax 
      SetCyclesRetP 24
%endmacro  


%macro lr35902@RET 4 ;; --------------------------- sub_call return 
;; %1 Opcode 
;; %2 Flags
;; %3 OpNOT
;; %4 RetHitCycles
   %1: 
      mov eax, YG_PF
      and eax, %2
      xor eax, %3 
      jne %%RET_skip 
      SetCyclesRetP 8
    %%RET_skip:
      OP_PopWord 
      mov YG_PC, eax 
      SetCyclesRetP %4
%endmacro    
      
%macro lr35902@RETI 4
 %1:
      OP_PopWord 
      mov YG_PC, eax 
      mov byte [YG_GP+lr35902.IME], 1
      SetCyclesAndRet 16    
%endmacro  

%macro OP_PushWord 0  ;; ----------------------- Stack Push Base 
;; source <- eax 
;; --SP Push High 
;; --SP Push Low
  mov cx, [YG_GP+lr35902.SP]
  sub cx, 2 
  mov [YG_GP+lr35902.SP], cx 
  push eax 
  push ecx 
  push dword [YG_GP+lr35902.gameboy]
  call MmuWriteWord
%endmacro  

%macro OP_PopWord  0  ;; ----------------------- Stack Pop Base 
;; target <- eax 
;; Pop Low ++SP
;; Pop High ++SP
  mov cx, [YG_GP+lr35902.SP]
  push ecx 
  add cx, 2 
  mov [YG_GP+lr35902.SP], cx 
  push dword [YG_GP+lr35902.gameboy]
  call MmuReadWord
%endmacro  

%macro AF_StackPopStuff  0  ;; -----------------------
  and eax, 0xFFF0
%endmacro  

%macro AF_StackPushStuff  0  ;; -----------------------
%endmacro  


%macro lr35902@MainALU 5
 ;; %1 OpCase 
 ;; %2 Cycles 
 ;; %3 ReadOrExt
 ;; %4 Op 
 ;; %5 WriteOrExt
  %1:
    %3
    %4
    %5 
    SetCyclesRetP %2
%endmacro 

%macro lr35902@MainALUExt  5
 ;; %1 OpCase 
 ;; %2 Cycles 
 ;; %3 ReadOrExt
 ;; %4 Op 
 ;; %5 WriteOrExt
  %1:
    %3
    %4
    %5 
    SetCyclesRetP %2
%endmacro 
        
;; rortoe shift with   
%macro RLC_ 0
  rol al, 1 
  setc YG_PF_8
  shl YG_PF_8, 4
  test al, al 
  setz dl 
  shl dl, 7
  or YG_PF_8, dl     
%endmacro  
;; rortoe shift with   
%macro RRC_ 0
  ror al, 1 
  setc YG_PF_8
  shl YG_PF_8, 4
  test al, al 
  setz dl 
  shl dl, 7 
  or YG_PF_8, dl     
%endmacro        
;; logic shift with carry
%macro RL_ 0
  shr YG_PF_8, 5
  rcl al, 1 
  setc YG_PF_8
  shl YG_PF_8, 4
  test al, al 
  setz dl 
  shl dl, 7
  or YG_PF_8, dl     
%endmacro  
;; logic shift with carry
%macro RR_ 0
  shr YG_PF_8, 5
  rcr al, 1 
  setc YG_PF_8
  shl YG_PF_8, 4
  test al, al 
  setz dl 
  shl dl, 7
  or YG_PF_8, dl     
%endmacro        
;; logic shift    
%macro RL_N_ 0
  shl al, 1 
  setc YG_PF_8
  shl YG_PF_8, 4
  test al, al 
  setz dl 
  shl dl, 7
  or YG_PF_8, dl     
%endmacro  
;; logic shift  
%macro RR_N_ 0
  shr al, 1 
  setc YG_PF_8
  shl YG_PF_8, 4
  test al, al 
  setz dl 
  shl dl, 7
  or YG_PF_8, dl     
%endmacro     
;; arith shift  save msb 
%macro RRS_N_ 0
  sar al, 1 
  setc YG_PF_8
  shl YG_PF_8, 4
  test al, al
  setz dl 
  shl dl, 7
  or YG_PF_8, dl     
%endmacro      
;; swap byte-lo 4bit and byte-hi 4bit
%macro SWAP_ 0
  ror al, 4 
  test al, al 
  setz cl 
  shl cl, 7
  mov YG_PF, ecx 
%endmacro          
  
;;  Set 
%macro lr35902@SetBit 5
 ;; %1 Opcode 
 ;; %2 Cycles 
 ;; %3 ReadOrExt 
 ;; %4 BitOrder
 ;; %5 WriteOrExt
  %1:
    %3
    mov ecx, 1 
    shl ecx, %4
    or eax, ecx 
    %5 
    SetCyclesAndRet %2
%endmacro  

%macro lr35902@ResBit 5
 ;; %1 Opcode 
 ;; %2 Cycles 
 ;; %3 ReadOrExt 
 ;; %4 BitOrder
 ;; %5 WriteOrExt
  %1:
    %3
    mov ecx, 1 
    shl ecx, %4
    not ecx 
    and al, cl 
    %5 
    SetCyclesRetP %2
%endmacro          
    
%macro lr35902@TestBit 4
 ;; %1 Opcode 
 ;; %2 Cycles 
 ;; %3 ReadOrExt 
 ;; %4 BitOrder
  %1:
    %3
    and YG_PF, C_FLAG
    or YG_PF, H_FLAG 
    mov ecx, 1 
    shl ecx, %4
    test al, cl 
    setz al 
    shl eax, 7
    or YG_PF, eax 
    SetCyclesRetP %2
%endmacro  
      
      
      
        ;; lr35902@MainStack  OPC1,12, PopWord_, EmptyMacro, BC_Write ;; POP BC  1 Cycles:12
%macro lr35902@MainStack    5 
 ;; %1 OpCase 
 ;; %2 Cycles 
 ;; %3 ReadOrExt
 ;; %4 Op 
 ;; %5 WriteOrExt
  %1:
    %3
    %4
    %5 
    SetCyclesAndRet %2
%endmacro 


section .text 
global _cpu_optick
_cpu_optick:
        push ebx ;U -  save old frame       
        push edi ;V -  save old frame 
        push esi 
        nop   
 
%define YG_PF_8 bl        
%define YG_PF ebx 
%define YG_PC esi 
%define YG_GP edi 

        ; ebx <- save now P (cpu's PSB reg)
        ; esi <- save now PC (cpu's EIP reg)
        ; edi <- save regs root
        ; eax <- calc temp or final calc out reslt 
        ; ecx <- calc temp 
        ; edx <- calc temp 
        
        mov YG_GP, [esp+4+12]   ;; fetch CPU struct 
        mov si, [YG_GP+lr35902.PC]
        mov bl, [YG_GP+lr35902.F]
        
        ; Fetch Opcode, PC++ 
        push YG_PC 
        push dword [YG_GP+lr35902.gameboy]
        call MmuRead
        inc YG_PC
        add YG_PC, [YG_GP+lr35902._backup]
        and eax, 255
        jmp dword [OPTAB+eax*4]
        
        lr35902@LD@Imm OP06, Imm8_Read, 8,  B_Write ;; LD B Imm8, 2, Cycles:8 
        lr35902@LD@Imm OP0E, Imm8_Read, 8,  C_Write ;; LD C Imm8, 2, Cycles:8 
        lr35902@LD@Imm OP16, Imm8_Read, 8,  D_Write ;; LD D Imm8, 2, Cycles:8 
        lr35902@LD@Imm OP1E, Imm8_Read, 8,  E_Write ;; LD E Imm8, 2, Cycles:8 
        lr35902@LD@Imm OP26, Imm8_Read, 8,  H_Write ;; LD H Imm8, 2, Cycles:8 
        lr35902@LD@Imm OP2E, Imm8_Read, 8,  L_Write ;; LD L Imm8, 2, Cycles:8 
        lr35902@LD@Imm OP36, Imm8_Read, 12, xHL_Write ;; LD xHL Imm8, 2, Cycles:8 
        lr35902@LD@Imm OP3E, Imm8_Read, 8,  A_Write ;; LD A Imm8, 2, Cycles:8 
     
        lr35902@LD@Imm OP01, Imm16_Read, 12, BC_Write ;; LD BC Imm16, 3 Cycles:12
        lr35902@LD@Imm OP11, Imm16_Read, 12, DE_Write ;; LD DE Imm16, 3 Cycles:12     
        lr35902@LD@Imm OP21, Imm16_Read, 12, HL_Write ;; LD HL Imm16, 3 Cycles:12     
        lr35902@LD@Imm OP31, Imm16_Read, 12, SP_Write ;; LD SP Imm16, 3 Cycles:12   
      
        lr35902@LD@RxHLToRxHL OP40, 4, B_Read, B_Write ;; LD B B, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP41, 4, C_Read, B_Write ;; LD B C, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP42, 4, D_Read, B_Write ;; LD B D, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP43, 4, E_Read, B_Write ;; LD B E, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP44, 4, H_Read, B_Write ;; LD B H, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP45, 4, L_Read, B_Write ;; LD B L, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP46, 8, xHL_Read, B_Write ;; LD B xHL, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP47, 4, A_Read, B_Write ;; LD B A, 1 Cycles:4   
        
        lr35902@LD@RxHLToRxHL OP48, 4, B_Read, C_Write ;; LD C B, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP49, 4, C_Read, C_Write ;; LD C C, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP4A, 4, D_Read, C_Write ;; LD C D, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP4B, 4, E_Read, C_Write ;; LD C E, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP4C, 4, H_Read, C_Write ;; LD C H, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP4D, 4, L_Read, C_Write ;; LD C L, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP4E, 8, xHL_Read, C_Write ;; LD C xHL, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP4F, 4, A_Read, C_Write ;; LD C A, 1 Cycles:4  

        lr35902@LD@RxHLToRxHL OP50, 4, B_Read, D_Write ;; LD D B, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP51, 4, C_Read, D_Write ;; LD D C, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP52, 4, D_Read, D_Write ;; LD D D, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP53, 4, E_Read, D_Write ;; LD D E, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP54, 4, H_Read, D_Write ;; LD D H, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP55, 4, L_Read, D_Write ;; LD D L, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP56, 8, xHL_Read, D_Write ;; LD D xHL, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP57, 4, A_Read, D_Write ;; LD D A, 1 Cycles:4   
        
        lr35902@LD@RxHLToRxHL OP58, 4, B_Read, E_Write ;; LD E B, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP59, 4, C_Read, E_Write ;; LD E C, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP5A, 4, D_Read, E_Write ;; LD E D, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP5B, 4, E_Read, E_Write ;; LD E E, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP5C, 4, H_Read, E_Write ;; LD E H, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP5D, 4, L_Read, E_Write ;; LD E L, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP5E, 8, xHL_Read, E_Write ;; LD E xHL, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP5F, 4, A_Read, E_Write ;; LD E A, 1 Cycles:4  

        lr35902@LD@RxHLToRxHL OP60, 4, B_Read, H_Write ;; LD H B, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP61, 4, C_Read, H_Write ;; LD H C, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP62, 4, D_Read, H_Write ;; LD H D, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP63, 4, E_Read, H_Write ;; LD H E, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP64, 4, H_Read, H_Write ;; LD H H, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP65, 4, L_Read, H_Write ;; LD H L, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP66, 8, xHL_Read, H_Write ;; LD H xHL, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP67, 4, A_Read, H_Write ;; LD H A, 1 Cycles:4   
        
        lr35902@LD@RxHLToRxHL OP68, 4, B_Read, L_Write ;; LD L B, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP69, 4, C_Read, L_Write ;; LD L C, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP6A, 4, D_Read, L_Write ;; LD L D, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP6B, 4, E_Read, L_Write ;; LD L E, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP6C, 4, H_Read, L_Write ;; LD L H, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP6D, 4, L_Read, L_Write ;; LD L L, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP6E, 8, xHL_Read, L_Write ;; LD L xHL, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP6F, 4, A_Read, L_Write ;; LD L A, 1 Cycles:4  
          
        lr35902@LD@RxHLToRxHL OP70, 8, B_Read, xHL_Write ;; LD xHL B, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP71, 8, C_Read, xHL_Write ;; LD xHL C, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP72, 8, D_Read, xHL_Write ;; LD xHL D, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP73, 8, E_Read, xHL_Write ;; LD xHL E, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP74, 8, H_Read, xHL_Write ;; LD xHL H, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP75, 8, L_Read, xHL_Write ;; LD xHL L, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP02, 8, A_Read, xBC_Write ;; LD xBC A, 1 Cycles:8     
        lr35902@LD@RxHLToRxHL OP12, 8, A_Read, xDE_Write ;; LD xDE A, 1 Cycles:8    
        lr35902@LD@RxHLToRxHL OP77, 8, A_Read, xHL_Write ;; LD xHL A, 1 Cycles:8    
        lr35902@LD@RxHLToRxHL OP22, 8, A_Read, xHL_Write_Inc;; LD xHL++ A, 1 Cycles:8  
        lr35902@LD@RxHLToRxHL OP32, 8, A_Read, xHL_Write_Dec ;; LD xHL-- A, 1 Cycles:8  
          
        lr35902@LD@RxHLToRxHL OP78, 4, B_Read, A_Write ;; LD A B, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP79, 4, C_Read, A_Write ;; LD A C, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP7A, 4, D_Read, A_Write ;; LD A D, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP7B, 4, E_Read, A_Write ;; LD A E, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP7C, 4, H_Read, A_Write ;; LD A H, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP7D, 4, L_Read, A_Write ;; LD A L, 1 Cycles:4
        lr35902@LD@RxHLToRxHL OP0A, 8, xBC_Read, A_Write ;; LD A xBC, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP1A, 8, xDE_Read, A_Write ;; LD A xDE, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP7E, 8, xHL_Read, A_Write ;; LD A xHL, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP2A, 8, xHL_Read_Inc, A_Write ;; LD A xHL++, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP3A, 8, xHL_Read_Dec, A_Write ;; LD A xHL--, 1 Cycles:8
        lr35902@LD@RxHLToRxHL OP7F, 4, A_Read, A_Write ;; LD A A, 1 Cycles:4      
  ;; MISC LD 
        lr35902@LD@RxHLToRxHL OP08,20, Imm16_Read, xImm16_WriteRegSP ;; LD (Imm16) SP, 3 Cycles:20
        lr35902@LD@RxHLToRxHL OPEA,16, Imm16_Read, xImm16_WriteRegA ;; LD (Imm16) A, 3 Cycles:16
        lr35902@LD@RxHLToRxHL OPE0,12, Imm8Read_ExpandAddress16, xImm16_WriteRegA ;; LD (Imm8+0FF00h) A, 2 Cycles:12
        lr35902@LD@RxHLToRxHL OPE2, 8, C_ExpandAddress16, xImm16_WriteRegA ;; LD (C+0FF00h) A, 2 Cycles:8  
        lr35902@LD@RxHLToRxHL OPF0,12, Imm8Read_ExpandAddress16_Fetch, A_Write ;; LD A, (Imm8+0FF00h)  2 Cycles:12
        lr35902@LD@RxHLToRxHL OPF2, 8, C_ExpandAddress16_Fetch, A_Write ;; LD A, (C+0FF00h) 2 Cycles:8    
        lr35902@LD@RxHLToRxHL OPFA,16, Imm16Read_Address_Fetch, A_Write ;; LD A, (Imm16) 3 Cycles:16
        lr35902@LD@RxHLToRxHL OPF9, 8, HL_Read, SP_Write ;; LD SP HL 1 Cycles:8 
        
        lr35902@MainALU  OP80, 4, B_Read, Add_, EmptyMacro ;; ADD A, B  1 Cycles:4
        lr35902@MainALU  OP81, 4, C_Read, Add_, EmptyMacro ;; ADD A, C  1 Cycles:4     
        lr35902@MainALU  OP82, 4, D_Read, Add_, EmptyMacro ;; ADD A, D  1 Cycles:4
        lr35902@MainALU  OP83, 4, E_Read, Add_, EmptyMacro ;; ADD A, E  1 Cycles:4      
        lr35902@MainALU  OP84, 4, H_Read, Add_, EmptyMacro ;; ADD A, H  1 Cycles:4
        lr35902@MainALU  OP85, 4, L_Read, Add_, EmptyMacro ;; ADD A, L  1 Cycles:4     
        lr35902@MainALU  OP86, 8, xHL_Read, Add_, EmptyMacro ;; ADD A, xHL  1 Cycles:8
        lr35902@MainALU  OP87, 4, A_Read, Add_, EmptyMacro ;; ADD A, A  1 Cycles:4  
        lr35902@MainALU  OPC6, 8, Imm8_Read, Add_, EmptyMacro ;; ADD A, Imm8  2 Cycles:8
        lr35902@MainALU  OP09, 8, BC_Read, AddWord_, HL_Write ;; ADD HL, BC  1 Cycles:8     
        lr35902@MainALU  OP19, 8, DE_Read, AddWord_, HL_Write ;; ADD HL, DE  1 Cycles:8     
        lr35902@MainALU  OP29, 8, HL_Read, AddWord_, HL_Write ;; ADD HL, HL  1 Cycles:8     
        lr35902@MainALU  OP39, 8, SP_Read, AddWord_, HL_Write ;; ADD HL, SP  1 Cycles:8     
        lr35902@MainALU  OPE8,16, Imm8_ExpandSignWord, AddWord2_, EmptyMacro ;; ADD SP, SignImm8  2 Cycles:16
        
        lr35902@MainALU  OP88, 4, B_Read, Adc_, EmptyMacro ;; ADC A, B  1 Cycles:4
        lr35902@MainALU  OP89, 4, C_Read, Adc_, EmptyMacro ;; ADC A, C  1 Cycles:4     
        lr35902@MainALU  OP8A, 4, D_Read, Adc_, EmptyMacro ;; ADC A, D  1 Cycles:4
        lr35902@MainALU  OP8B, 4, E_Read, Adc_, EmptyMacro ;; ADC A, E  1 Cycles:4      
        lr35902@MainALU  OP8C, 4, H_Read, Adc_, EmptyMacro ;; ADC A, H  1 Cycles:4
        lr35902@MainALU  OP8D, 4, L_Read, Adc_, EmptyMacro ;; ADC A, L  1 Cycles:4     
        lr35902@MainALU  OP8E, 8, xHL_Read, Adc_, EmptyMacro ;; ADC A, xHL  1 Cycles:8
        lr35902@MainALU  OP8F, 4, A_Read, Adc_, EmptyMacro ;; ADC A, A  1 Cycles:4  
        lr35902@MainALU  OPCE, 8, Imm8_Read, Adc_, EmptyMacro ;; ADC A, Imm8  2 Cycles:8     
        
        lr35902@MainALU  OP03, 8, BC_Read, IncWord_, BC_Write ;; INC BC  Cycles:8
        lr35902@MainALU  OP13, 8, DE_Read, IncWord_, DE_Write ;; INC DE  Cycles:8
        lr35902@MainALU  OP23, 8, HL_Read, IncWord_, HL_Write ;; INC HL  Cycles:8
        lr35902@MainALU  OP33, 8, SP_Read, IncWord_, SP_Write ;; INC SP  Cycles:8
        
        lr35902@MainALU  OP04, 4, B_Read, Inc_, B_Write ;; INC B  1 Cycles:4
        lr35902@MainALU  OP14, 4, D_Read, Inc_, D_Write ;; INC D  1 Cycles:4     
        lr35902@MainALU  OP24, 4, H_Read, Inc_, H_Write ;; INC H  1 Cycles:4
        lr35902@MainALU  OP34, 4, xHL_Read, Inc_, xHL_Write ;; INC xHL  1 Cycles:4      
        lr35902@MainALU  OP0C, 4, C_Read, Inc_, C_Write ;; INC C  1 Cycles:4
        lr35902@MainALU  OP1C, 4, E_Read, Inc_, E_Write ;; INC E  1 Cycles:4     
        lr35902@MainALU  OP2C,12, L_Read, Inc_, L_Write ;; INC L  1 Cycles:12
        lr35902@MainALU  OP3C, 4, A_Read, Inc_, A_Write ;; INC A  1 Cycles:4       
        
        lr35902@MainALU  OP90, 4, B_Read, Sub_, EmptyMacro ;; SUB A, B  1 Cycles:4
        lr35902@MainALU  OP91, 4, C_Read, Sub_, EmptyMacro ;; SUB A, C  1 Cycles:4     
        lr35902@MainALU  OP92, 4, D_Read, Sub_, EmptyMacro ;; SUB A, D  1 Cycles:4
        lr35902@MainALU  OP93, 4, E_Read, Sub_, EmptyMacro ;; SUB A, E  1 Cycles:4      
        lr35902@MainALU  OP94, 4, H_Read, Sub_, EmptyMacro ;; SUB A, H  1 Cycles:4
        lr35902@MainALU  OP95, 4, L_Read, Sub_, EmptyMacro ;; SUB A, L  1 Cycles:4     
        lr35902@MainALU  OP96, 8, xHL_Read, Sub_, EmptyMacro ;; SUB A, xHL  1 Cycles:8
        lr35902@MainALU  OP97, 4, A_Read, Sub_, EmptyMacro ;; SUB A, A  1 Cycles:4  
        lr35902@MainALU  OPD6, 8, Imm8_Read, Sub_, EmptyMacro ;; SUB A, Imm8  2 Cycles:8
        
        lr35902@MainALU  OP98, 4, B_Read, Sbc_, EmptyMacro ;; SBC A, B  1 Cycles:4
        lr35902@MainALU  OP99, 4, C_Read, Sbc_, EmptyMacro ;; SBC A, C  1 Cycles:4     
        lr35902@MainALU  OP9A, 4, D_Read, Sbc_, EmptyMacro ;; SBC A, D  1 Cycles:4
        lr35902@MainALU  OP9B, 4, E_Read, Sbc_, EmptyMacro ;; SBC A, E  1 Cycles:4      
        lr35902@MainALU  OP9C, 4, H_Read, Sbc_, EmptyMacro ;; SBC A, H  1 Cycles:4
        lr35902@MainALU  OP9D, 4, L_Read, Sbc_, EmptyMacro ;; SBC A, L  1 Cycles:4     
        lr35902@MainALU  OP9E, 8, xHL_Read, Sbc_, EmptyMacro ;; SBC A, xHL  1 Cycles:8
        lr35902@MainALU  OP9F, 4, A_Read, Sbc_, EmptyMacro ;; SBC A, A  1 Cycles:4  
        lr35902@MainALU  OPDE, 8, Imm8_Read, Sbc_, EmptyMacro ;; SBC A, Imm8  2 Cycles:8
     
        lr35902@MainALU  OP0B, 8, BC_Read, DecWord_, BC_Write ;; DEC BC  Cycles:8
        lr35902@MainALU  OP1B, 8, DE_Read, DecWord_, DE_Write ;; DEC DE  Cycles:8
        lr35902@MainALU  OP2B, 8, HL_Read, DecWord_, HL_Write ;; DEC HL  Cycles:8
        lr35902@MainALU  OP3B, 8, SP_Read, DecWord_, SP_Write ;; DEC SP  Cycles:8
        
        lr35902@MainALU  OP05, 4, B_Read, Dec_, B_Write ;; DEC B  1 Cycles:4
        lr35902@MainALU  OP15, 4, D_Read, Dec_, D_Write ;; DEC D  1 Cycles:4     
        lr35902@MainALU  OP25, 4, H_Read, Dec_, H_Write ;; DEC H  1 Cycles:4
        lr35902@MainALU  OP35, 4, xHL_Read, Dec_, xHL_Write ;; DEC xHL  1 Cycles:4      
        lr35902@MainALU  OP0D, 4, C_Read, Dec_, C_Write ;; DEC C  1 Cycles:4
        lr35902@MainALU  OP1D, 4, E_Read, Dec_, E_Write ;; DEC E  1 Cycles:4     
        lr35902@MainALU  OP2D,12, L_Read, Dec_, L_Write ;; DEC L  1 Cycles:12
        lr35902@MainALU  OP3D, 4, A_Read, Dec_, A_Write ;; DEC A  1 Cycles:4  
        
        lr35902@MainALU  OPA0, 4, B_Read, And_, EmptyMacro ;; AND A, B  1 Cycles:4
        lr35902@MainALU  OPA1, 4, C_Read, And_, EmptyMacro ;; AND A, C  1 Cycles:4     
        lr35902@MainALU  OPA2, 4, D_Read, And_, EmptyMacro ;; AND A, D  1 Cycles:4
        lr35902@MainALU  OPA3, 4, E_Read, And_, EmptyMacro ;; AND A, E  1 Cycles:4      
        lr35902@MainALU  OPA4, 4, H_Read, And_, EmptyMacro ;; AND A, H  1 Cycles:4
        lr35902@MainALU  OPA5, 4, L_Read, And_, EmptyMacro ;; AND A, L  1 Cycles:4     
        lr35902@MainALU  OPA6, 8, xHL_Read, And_, EmptyMacro ;; AND A, xHL  1 Cycles:8
        lr35902@MainALU  OPA7, 4, A_Read, And_, EmptyMacro ;; AND A, A  1 Cycles:4  
        lr35902@MainALU  OPE6, 8, Imm8_Read, And_, EmptyMacro ;; AND A, Imm8  2 Cycles:8
         
        lr35902@MainALU  OPA8, 4, B_Read, Xor_, EmptyMacro ;; XOR A, B  1 Cycles:4
        lr35902@MainALU  OPA9, 4, C_Read, Xor_, EmptyMacro ;; XOR A, C  1 Cycles:4     
        lr35902@MainALU  OPAA, 4, D_Read, Xor_, EmptyMacro ;; XOR A, D  1 Cycles:4
        lr35902@MainALU  OPAB, 4, E_Read, Xor_, EmptyMacro ;; XOR A, E  1 Cycles:4      
        lr35902@MainALU  OPAC, 4, H_Read, Xor_, EmptyMacro ;; XOR A, H  1 Cycles:4
        lr35902@MainALU  OPAD, 4, L_Read, Xor_, EmptyMacro ;; XOR A, L  1 Cycles:4     
        lr35902@MainALU  OPAE, 8, xHL_Read, Xor_, EmptyMacro ;; XOR A, xHL  1 Cycles:8
        lr35902@MainALU  OPAF, 4, A_Read, Xor_, EmptyMacro ;; XOR A, A  1 Cycles:4  
        lr35902@MainALU  OPEE, 8, Imm8_Read, Xor_, EmptyMacro ;; XOR A, Imm8  2 Cycles:8
        
        lr35902@MainALU  OPB0, 4, B_Read, Or_, EmptyMacro ;; OR A, B  1 Cycles:4
        lr35902@MainALU  OPB1, 4, C_Read, Or_, EmptyMacro ;; OR A, C  1 Cycles:4     
        lr35902@MainALU  OPB2, 4, D_Read, Or_, EmptyMacro ;; OR A, D  1 Cycles:4
        lr35902@MainALU  OPB3, 4, E_Read, Or_, EmptyMacro ;; OR A, E  1 Cycles:4      
        lr35902@MainALU  OPB4, 4, H_Read, Or_, EmptyMacro ;; OR A, H  1 Cycles:4
        lr35902@MainALU  OPB5, 4, L_Read, Or_, EmptyMacro ;; OR A, L  1 Cycles:4     
        lr35902@MainALU  OPB6, 8, xHL_Read, Or_, EmptyMacro ;; OR A, xHL  1 Cycles:8
        lr35902@MainALU  OPB7, 4, A_Read, Or_, EmptyMacro ;; OR A, A  1 Cycles:4  
        lr35902@MainALU  OPF6, 8, Imm8_Read, Or_, EmptyMacro ;; OR A, Imm8  2 Cycles:8
        
        lr35902@MainALU  OPB8, 4, B_Read, Cmp_, EmptyMacro ;; CP A, B  1 Cycles:4
        lr35902@MainALU  OPB9, 4, C_Read, Cmp_, EmptyMacro ;; CP A, C  1 Cycles:4     
        lr35902@MainALU  OPBA, 4, D_Read, Cmp_, EmptyMacro ;; CP A, D  1 Cycles:4
        lr35902@MainALU  OPBB, 4, E_Read, Cmp_, EmptyMacro ;; CP A, E  1 Cycles:4      
        lr35902@MainALU  OPBC, 4, H_Read, Cmp_, EmptyMacro ;; CP A, H  1 Cycles:4
        lr35902@MainALU  OPBD, 4, L_Read, Cmp_, EmptyMacro ;; CP A, L  1 Cycles:4     
        lr35902@MainALU  OPBE, 8, xHL_Read, Cmp_, EmptyMacro ;; CP A, xHL  1 Cycles:8
        lr35902@MainALU  OPBF, 4, A_Read, Cmp_, EmptyMacro ;; CP A, A  1 Cycles:4  
        lr35902@MainALU  OPFE, 8, Imm8_Read, Cmp_, EmptyMacro ;; CP A, Imm8  2 Cycles:8 

        lr35902@MainStack  OPC1,12, OP_PopWord, EmptyMacro, BC_Write ;; POP BC  1 Cycles:12
        lr35902@MainStack  OPD1,12, OP_PopWord, EmptyMacro, DE_Write ;; POP DE  1 Cycles:12     
        lr35902@MainStack  OPE1,12, OP_PopWord, EmptyMacro, HL_Write ;; POP HL  1 Cycles:12
        lr35902@MainStack  OPF1,12, OP_PopWord, AF_StackPopStuff, AF_Write ;; POP AF  1 Cycles:12  
        
        lr35902@MainStack  OPC5,16, BC_Read, EmptyMacro, OP_PushWord ;; PUSH BC  1 Cycles:16
        lr35902@MainStack  OPD5,16, DE_Read, EmptyMacro, OP_PushWord ;; PUSH DE  1 Cycles:16     
        lr35902@MainStack  OPE5,16, HL_Read, EmptyMacro, OP_PushWord ;; PUSH HL  1 Cycles:16
        lr35902@MainStack  OPF5,16, AF_Read, AF_StackPushStuff, OP_PushWord ;; PUSH AF  1 Cycles:16  
    
        lr35902@JP  OPC2,Z_FLAG, Z_FLAG ;; JP NZ 
        lr35902@JP  OPD2,C_FLAG, C_FLAG ;; JP NC 
        lr35902@JP  OPCA,Z_FLAG, 0 ;; JP Z 
        lr35902@JP  OPDA,C_FLAG, 0 ;; JP C   
        lr35902@JP  OPC3,0, 1 ;; JP A16    
        
        lr35902@Rst  OPC7, 000H ;; RST 00H  1 Cycles:16
        lr35902@Rst  OPD7, 010H ;; RST 10H  1 Cycles:16     
        lr35902@Rst  OPE7, 020H ;; RST 20H  1 Cycles:16
        lr35902@Rst  OPF7, 030H ;; RST 30H  1 Cycles:16  
        lr35902@Rst  OPCF, 008H ;; RST 08H  1 Cycles:16
        lr35902@Rst  OPDF, 018H ;; RST 18H  1 Cycles:16     
        lr35902@Rst  OPEF, 028H ;; RST 28H  1 Cycles:16
        lr35902@Rst  OPFF, 038H ;; RST 38H  1 Cycles:16     
        
        lr35902@CALL  OPC4,Z_FLAG, Z_FLAG ;; CALL NZ 
        lr35902@CALL  OPD4,C_FLAG, C_FLAG ;; CALL NC 
        lr35902@CALL  OPCC,Z_FLAG, 0 ;; CALL Z 
        lr35902@CALL  OPDC,C_FLAG, 0 ;; CALL C   
        lr35902@CALL  OPCD,0, 1 ;; CALL  
      
        lr35902@JR  OP20,Z_FLAG, Z_FLAG ;; JR NZ 
        lr35902@JR  OP30,C_FLAG, C_FLAG ;; JR NC 
        lr35902@JR  OP28,Z_FLAG, 0 ;; JR Z 
        lr35902@JR  OP38,C_FLAG, 0 ;; JR C   
        lr35902@JR  OP18,0, 1 ;; JR R8  

        lr35902@RET  OPC0,Z_FLAG, Z_FLAG, 20 ;; RET NZ 
        lr35902@RET  OPD0,C_FLAG, C_FLAG, 20 ;; RET NC 
        lr35902@RET  OPC8,Z_FLAG, 0, 20 ;; RET Z 
        lr35902@RET  OPD8,C_FLAG, 0, 20 ;; RET C   
        lr35902@RET  OPC9,0, 1, 16 ;; RET  
        lr35902@RETI OPD9,0, 1, 16 ;; RETI 
;;  MISC 
      OPF8: ;; -------------------------------------------  LD HL SP+Imm8(sign8) 2 Cycles:12 
        Imm8_Read
        ;; ext sign 
        movsx eax, al 
        movzx ecx, word[YG_GP+lr35902.SP]
        and eax, 0xFFFF
        lea edx, [ecx+eax]
        mov [YG_GP+lr35902.HL], dx ;; write back HL 
        xor ecx, eax 
        xor ecx, edx 
        and cx, 0x0110 ;; C|H 
        mov YG_PF_8, ch 
        shl YG_PF_8, 4
        shl cl, 1
        or YG_PF_8, cl 
        and YG_PF, (H_FLAG| C_FLAG)
        SetCyclesRetP 12
      OP76:   ; Halt,  not backup PC in my source code ^_^
        mov dword [YG_GP+lr35902.halt], 1
        SetCyclesAndRet 4    
      OP10:   ; Stop, Check CGB speed mode 
        movzx eax, byte [YG_GP+lr35902.key1]
        test eax, 1 
        je Stop_Skip
        xor eax, 0x80 ;; switch to "other" speed 
        and eax, 0xFE ;; reset LSB  see gb-programming-manual.pdf::2.6.2 CPU Operating Speed
                      ;; for simplicity, I will not simulate the huge waste of time brought by handover.
        mov [YG_GP+lr35902.key1], al 
        add YG_PC, 1 ;; skip one byte (should is 00)      
        SetCyclesAndRet 0x80000004
     Stop_Skip:
        mov dword [YG_GP+lr35902.stop], 1
        add YG_PC, 1 ;; skip one byte (should is 00)    
        SetCyclesAndRet 4        
      OPF3:   ; DI 
        mov byte [YG_GP+lr35902.IME], 0 
        SetCyclesAndRet 4 
      OPFB:   ; EI 
        mov byte [YG_GP+lr35902.IME], 1
        SetCyclesAndRet 4   
      OP07:   ; RLCA 
        rol byte [YG_GP+lr35902.A], 1 
        setc YG_PF_8
        shl YG_PF_8, 4
        SetCyclesRetP 4 
      OP17:   ; RLA 
        shr YG_PF_8, 5
        rcl byte [YG_GP+lr35902.A], 1 
        setc YG_PF_8
        shl YG_PF_8, 4
        SetCyclesRetP 4    
      OP0F:   ; RRCA 
        ror byte [YG_GP+lr35902.A], 1 
        setc YG_PF_8
        shl YG_PF_8, 4
        SetCyclesRetP 4 
      OP1F:   ; RRA 
        shr YG_PF_8, 5
        rcr byte [YG_GP+lr35902.A], 1 
        setc YG_PF_8   
        shl YG_PF_8, 4
        SetCyclesRetP 4
      OP27:   ; BCD Adjust   
        movzx eax, byte [YG_GP+lr35902.A]
        test YG_PF_8, N_FLAG 
        jne DAS_Proc 
        ;;  DAA. 
        ;;  Check DAA-low 
        test YG_PF, H_FLAG
        jne DAA_Low 
        mov ecx, eax 
        and ecx, 0x0F 
        cmp ecx, 9
        jbe DAA_LowSkip
        DAA_Low:
          add eax, 6 
          DAA_LowSkip:
            ;; Check DAA-High 
            test YG_PF, C_FLAG
            jne DAA_High
            mov ecx, eax  
            cmp ecx, 0x9F
            jbe DAA_HighSkip
            DAA_High:
              add eax, 0x60 
              DAA_HighSkip:
                ;; Check Z, C 
                and YG_PF, C_FLAG
                mov ecx, eax 
                and ecx, 0x100 
                shl ecx, 4 
                or YG_PF_8, ch 
                test al, al 
                setz cl 
                shl cl, 7
                or YG_PF_8, cl ;; z_flag done 
                mov [YG_GP+lr35902.A], al
                SetCyclesRetP 4       
    DAS_Proc:
        ;;  DAS 
        ;;  Check DAS-low 
        test YG_PF, H_FLAG
        je DAS_LowSkip 
        sub eax, 6
        and eax, 0xFF
          DAS_LowSkip:
            ;; Check DAS-High 
            test YG_PF, C_FLAG
            je DAS_HighSkip
            sub eax, 0x60
              DAS_HighSkip:
                ;; Check Z, C 
                and YG_PF, C_FLAG
                mov ecx, eax 
                and ecx, 0x100 
                shl ecx, 4 
                or YG_PF_8, ch 
                test al, al 
                setz cl
                shl cl, 7
                or YG_PF_8, cl ;; z_flag done 
                or YG_PF_8, N_FLAG ;; n_flag done 
                mov [YG_GP+lr35902.A], al
                SetCyclesRetP 4               
      OP37:   ; SCF 
        and YG_PF, Z_FLAG
        or YG_PF, C_FLAG  
        SetCyclesRetP 4    
      OP2F:   ; CPL  
        not byte [YG_GP+lr35902.A] 
        or YG_PF, N_FLAG
        or YG_PF, H_FLAG
        SetCyclesRetP 4   
      OP3F:   ; CCF 
        and YG_PF, (Z_FLAG |C_FLAG)
        xor YG_PF, C_FLAG  
        SetCyclesRetP 4   
      OPE9:
        mov si, [YG_GP+lr35902.HL]
        SetCyclesAndRet 4    
      OPCB: ;; DD Perfix(BITS) for Z80/lr35902 
        Imm8_Read 
        and eax, 255 
        jmp dword [CBTAB+eax*4]
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; Bit Opcode DOne 
        ;; --------------------------------------------------------------
            lr35902@MainALUExt  CB00, 8, B_Read, RLC_, B_Write      ;; RLC B 2 Cycles:8
            lr35902@MainALUExt  CB01, 8, C_Read, RLC_, C_Write      ;; RLC C 2 Cycles:8
            lr35902@MainALUExt  CB02, 8, D_Read, RLC_, D_Write      ;; RLC D 2 Cycles:8
            lr35902@MainALUExt  CB03, 8, E_Read, RLC_, E_Write      ;; RLC E 2 Cycles:8     
            lr35902@MainALUExt  CB04, 8, H_Read, RLC_, H_Write      ;; RLC H 2 Cycles:8
            lr35902@MainALUExt  CB05, 8, L_Read, RLC_, L_Write      ;; RLC L 2 Cycles:8        
            lr35902@MainALUExt  CB06,16, xHL_Read, RLC_, xHL_Write      ;; RLC xHL 2 Cycles:16
            lr35902@MainALUExt  CB07, 8, A_Read, RLC_, A_Write      ;; RLC A 2 Cycles:8        
            
            lr35902@MainALUExt  CB08, 8, B_Read, RRC_, B_Write      ;; RRC B 2 Cycles:8
            lr35902@MainALUExt  CB09, 8, C_Read, RRC_, C_Write      ;; RRC C 2 Cycles:8
            lr35902@MainALUExt  CB0A, 8, D_Read, RRC_, D_Write      ;; RRC D 2 Cycles:8
            lr35902@MainALUExt  CB0B, 8, E_Read, RRC_, E_Write      ;; RRC E 2 Cycles:8     
            lr35902@MainALUExt  CB0C, 8, H_Read, RRC_, H_Write      ;; RRC H 2 Cycles:8
            lr35902@MainALUExt  CB0D, 8, L_Read, RRC_, L_Write      ;; RRC L 2 Cycles:8        
            lr35902@MainALUExt  CB0E,16, xHL_Read, RRC_, xHL_Write      ;; RRC xHL 2 Cycles:16
            lr35902@MainALUExt  CB0F, 8, A_Read, RRC_, A_Write      ;; RRC A 2 Cycles:8     
            
            lr35902@MainALUExt  CB10, 8, B_Read, RL_, B_Write      ;; RL B 2 Cycles:8
            lr35902@MainALUExt  CB11, 8, C_Read, RL_, C_Write      ;; RL C 2 Cycles:8
            lr35902@MainALUExt  CB12, 8, D_Read, RL_, D_Write      ;; RL D 2 Cycles:8
            lr35902@MainALUExt  CB13, 8, E_Read, RL_, E_Write      ;; RL E 2 Cycles:8     
            lr35902@MainALUExt  CB14, 8, H_Read, RL_, H_Write      ;; RL H 2 Cycles:8
            lr35902@MainALUExt  CB15, 8, L_Read, RL_, L_Write      ;; RL L 2 Cycles:8        
            lr35902@MainALUExt  CB16,16, xHL_Read, RL_, xHL_Write      ;; RL xHL 2 Cycles:16
            lr35902@MainALUExt  CB17, 8, A_Read, RL_, A_Write      ;; RL A 2 Cycles:8        
            
            lr35902@MainALUExt  CB18, 8, B_Read, RR_, B_Write      ;; RR B 2 Cycles:8
            lr35902@MainALUExt  CB19, 8, C_Read, RR_, C_Write      ;; RR C 2 Cycles:8
            lr35902@MainALUExt  CB1A, 8, D_Read, RR_, D_Write      ;; RR D 2 Cycles:8
            lr35902@MainALUExt  CB1B, 8, E_Read, RR_, E_Write      ;; RR E 2 Cycles:8     
            lr35902@MainALUExt  CB1C, 8, H_Read, RR_, H_Write      ;; RR H 2 Cycles:8
            lr35902@MainALUExt  CB1D, 8, L_Read, RR_, L_Write      ;; RR L 2 Cycles:8        
            lr35902@MainALUExt  CB1E,16, xHL_Read, RR_, xHL_Write      ;; RR xHL 2 Cycles:16
            lr35902@MainALUExt  CB1F, 8, A_Read, RR_, A_Write      ;; RR A 2 Cycles:8            
            
            lr35902@MainALUExt  CB20, 8, B_Read, RL_N_, B_Write      ;; SLA B 2 Cycles:8
            lr35902@MainALUExt  CB21, 8, C_Read, RL_N_, C_Write      ;; SLA C 2 Cycles:8
            lr35902@MainALUExt  CB22, 8, D_Read, RL_N_, D_Write      ;; SLA D 2 Cycles:8
            lr35902@MainALUExt  CB23, 8, E_Read, RL_N_, E_Write      ;; SLA E 2 Cycles:8     
            lr35902@MainALUExt  CB24, 8, H_Read, RL_N_, H_Write      ;; SLA H 2 Cycles:8
            lr35902@MainALUExt  CB25, 8, L_Read, RL_N_, L_Write      ;; SLA L 2 Cycles:8        
            lr35902@MainALUExt  CB26,16, xHL_Read, RL_N_, xHL_Write      ;; SLA xHL 2 Cycles:16
            lr35902@MainALUExt  CB27, 8, A_Read, RL_N_, A_Write      ;; SLA A 2 Cycles:8        
            
            lr35902@MainALUExt  CB28, 8, B_Read, RRS_N_, B_Write      ;; SRA B 2 Cycles:8
            lr35902@MainALUExt  CB29, 8, C_Read, RRS_N_, C_Write      ;; SRA C 2 Cycles:8
            lr35902@MainALUExt  CB2A, 8, D_Read, RRS_N_, D_Write      ;; SRA D 2 Cycles:8
            lr35902@MainALUExt  CB2B, 8, E_Read, RRS_N_, E_Write      ;; SRA E 2 Cycles:8     
            lr35902@MainALUExt  CB2C, 8, H_Read, RRS_N_, H_Write      ;; SRA H 2 Cycles:8
            lr35902@MainALUExt  CB2D, 8, L_Read, RRS_N_, L_Write      ;; SRA L 2 Cycles:8        
            lr35902@MainALUExt  CB2E,16, xHL_Read, RRS_N_, xHL_Write      ;; SRA xHL 2 Cycles:16
            lr35902@MainALUExt  CB2F, 8, A_Read, RRS_N_, A_Write      ;; SRA A 2 Cycles:8      
            
            lr35902@MainALUExt  CB30, 8, B_Read, SWAP_, B_Write      ;; SWAP B 2 Cycles:8
            lr35902@MainALUExt  CB31, 8, C_Read, SWAP_, C_Write      ;; SWAP C 2 Cycles:8
            lr35902@MainALUExt  CB32, 8, D_Read, SWAP_, D_Write      ;; SWAP D 2 Cycles:8
            lr35902@MainALUExt  CB33, 8, E_Read, SWAP_, E_Write      ;; SWAP E 2 Cycles:8     
            lr35902@MainALUExt  CB34, 8, H_Read, SWAP_, H_Write      ;; SWAP H 2 Cycles:8
            lr35902@MainALUExt  CB35, 8, L_Read, SWAP_, L_Write      ;; SWAP L 2 Cycles:8        
            lr35902@MainALUExt  CB36,16, xHL_Read, SWAP_, xHL_Write      ;; SWAP xHL 2 Cycles:16
            lr35902@MainALUExt  CB37, 8, A_Read, SWAP_, A_Write      ;; SWAP A 2 Cycles:8        
            
            lr35902@MainALUExt  CB38, 8, B_Read, RR_N_, B_Write      ;; SRL B 2 Cycles:8
            lr35902@MainALUExt  CB39, 8, C_Read, RR_N_, C_Write      ;; SRL C 2 Cycles:8
            lr35902@MainALUExt  CB3A, 8, D_Read, RR_N_, D_Write      ;; SRL D 2 Cycles:8
            lr35902@MainALUExt  CB3B, 8, E_Read, RR_N_, E_Write      ;; SRL E 2 Cycles:8     
            lr35902@MainALUExt  CB3C, 8, H_Read, RR_N_, H_Write      ;; SRL H 2 Cycles:8
            lr35902@MainALUExt  CB3D, 8, L_Read, RR_N_, L_Write      ;; SRL L 2 Cycles:8        
            lr35902@MainALUExt  CB3E,16, xHL_Read, RR_N_, xHL_Write      ;; SRL xHL 2 Cycles:16
            lr35902@MainALUExt  CB3F, 8, A_Read, RR_N_, A_Write      ;; SRL A 2 Cycles:8        
            
            lr35902@TestBit CB40, 8,  B_Read, 0 ;; BIT B, 0 Cycles:8 
            lr35902@TestBit CB41, 8,  C_Read, 0 ;; BIT C, 0 Cycles:8        
            lr35902@TestBit CB42, 8,  D_Read, 0 ;; BIT D, 0 Cycles:8        
            lr35902@TestBit CB43, 8,  E_Read, 0 ;; BIT E, 0 Cycles:8     
            lr35902@TestBit CB44, 8,  H_Read, 0 ;; BIT H, 0 Cycles:8 
            lr35902@TestBit CB45, 8,  L_Read, 0 ;; BIT L, 0 Cycles:8        
            lr35902@TestBit CB46,16,  xHL_Read, 0 ;; BIT xHL, 0 Cycles:16       
            lr35902@TestBit CB47, 8,  A_Read, 0 ;; BIT A, 0 Cycles:8         
          
            lr35902@TestBit CB48, 8,  B_Read, 1 ;; BIT B, 1 Cycles:8 
            lr35902@TestBit CB49, 8,  C_Read, 1 ;; BIT C, 1 Cycles:8        
            lr35902@TestBit CB4A, 8,  D_Read, 1 ;; BIT D, 1 Cycles:8        
            lr35902@TestBit CB4B, 8,  E_Read, 1 ;; BIT E, 1 Cycles:8     
            lr35902@TestBit CB4C, 8,  H_Read, 1 ;; BIT H, 1 Cycles:8 
            lr35902@TestBit CB4D, 8,  L_Read, 1 ;; BIT L, 1 Cycles:8        
            lr35902@TestBit CB4E,16,  xHL_Read, 1 ;; BIT xHL, 1 Cycles:16       
            lr35902@TestBit CB4F, 8,  A_Read, 1 ;; BIT A, 1 Cycles:8 

            lr35902@TestBit CB50, 8,  B_Read, 2 ;; BIT B, 2 Cycles:8 
            lr35902@TestBit CB51, 8,  C_Read, 2 ;; BIT C, 2 Cycles:8        
            lr35902@TestBit CB52, 8,  D_Read, 2 ;; BIT D, 2 Cycles:8        
            lr35902@TestBit CB53, 8,  E_Read, 2 ;; BIT E, 2 Cycles:8     
            lr35902@TestBit CB54, 8,  H_Read, 2 ;; BIT H, 2 Cycles:8 
            lr35902@TestBit CB55, 8,  L_Read, 2 ;; BIT L, 2 Cycles:8        
            lr35902@TestBit CB56,16,  xHL_Read, 2 ;; BIT xHL, 2 Cycles:16       
            lr35902@TestBit CB57, 8,  A_Read, 2 ;; BIT A, 2 Cycles:8         
          
            lr35902@TestBit CB58, 8,  B_Read, 3 ;; BIT B, 3 Cycles:8 
            lr35902@TestBit CB59, 8,  C_Read, 3 ;; BIT C, 3 Cycles:8        
            lr35902@TestBit CB5A, 8,  D_Read, 3 ;; BIT D, 3 Cycles:8        
            lr35902@TestBit CB5B, 8,  E_Read, 3 ;; BIT E, 3 Cycles:8     
            lr35902@TestBit CB5C, 8,  H_Read, 3 ;; BIT H, 3 Cycles:8 
            lr35902@TestBit CB5D, 8,  L_Read, 3 ;; BIT L, 3 Cycles:8        
            lr35902@TestBit CB5E,16,  xHL_Read, 3 ;; BIT xHL, 3 Cycles:16       
            lr35902@TestBit CB5F, 8,  A_Read, 3 ;; BIT A, 3 Cycles:8 

            lr35902@TestBit CB60, 8,  B_Read, 4 ;; BIT B, 4 Cycles:8 
            lr35902@TestBit CB61, 8,  C_Read, 4 ;; BIT C, 4 Cycles:8        
            lr35902@TestBit CB62, 8,  D_Read, 4 ;; BIT D, 4 Cycles:8        
            lr35902@TestBit CB63, 8,  E_Read, 4 ;; BIT E, 4 Cycles:8     
            lr35902@TestBit CB64, 8,  H_Read, 4 ;; BIT H, 4 Cycles:8 
            lr35902@TestBit CB65, 8,  L_Read, 4 ;; BIT L, 4 Cycles:8        
            lr35902@TestBit CB66,16,  xHL_Read, 4 ;; BITxHLD, 4 Cycles:16       
            lr35902@TestBit CB67, 8,  A_Read, 4 ;; BIT A, 4 Cycles:8         
          
            lr35902@TestBit CB68, 8,  B_Read, 5 ;; BIT B, 5 Cycles:8 
            lr35902@TestBit CB69, 8,  C_Read, 5 ;; BIT C, 5 Cycles:8        
            lr35902@TestBit CB6A, 8,  D_Read, 5 ;; BIT D, 5 Cycles:8        
            lr35902@TestBit CB6B, 8,  E_Read, 5 ;; BIT E, 5 Cycles:8     
            lr35902@TestBit CB6C, 8,  H_Read, 5 ;; BIT H, 5 Cycles:8 
            lr35902@TestBit CB6D, 8,  L_Read, 5 ;; BIT L, 5 Cycles:8        
            lr35902@TestBit CB6E,16,  xHL_Read, 5 ;; BIT xHL, 5 Cycles:16       
            lr35902@TestBit CB6F, 8,  A_Read, 5 ;; BIT A, 5 Cycles:8 
            
            lr35902@TestBit CB70, 8,  B_Read, 6 ;; BIT B, 6 Cycles:8 
            lr35902@TestBit CB71, 8,  C_Read, 6 ;; BIT C, 6 Cycles:8        
            lr35902@TestBit CB72, 8,  D_Read, 6 ;; BIT D, 6 Cycles:8        
            lr35902@TestBit CB73, 8,  E_Read, 6 ;; BIT E, 6 Cycles:8     
            lr35902@TestBit CB74, 8,  H_Read, 6 ;; BIT H, 6 Cycles:8 
            lr35902@TestBit CB75, 8,  L_Read, 6 ;; BIT L, 6 Cycles:8        
            lr35902@TestBit CB76,16,  xHL_Read, 6 ;; BIT xHL, 6 Cycles:16       
            lr35902@TestBit CB77, 8,  A_Read, 6 ;; BIT A, 6 Cycles:8         
          
            lr35902@TestBit CB78, 8,  B_Read, 7 ;; BIT B, 7 Cycles:8 
            lr35902@TestBit CB79, 8,  C_Read, 7 ;; BIT C, 7 Cycles:8        
            lr35902@TestBit CB7A, 8,  D_Read, 7 ;; BIT D, 7 Cycles:8        
            lr35902@TestBit CB7B, 8,  E_Read, 7 ;; BIT E, 7 Cycles:8     
            lr35902@TestBit CB7C, 8,  H_Read, 7 ;; BIT H, 7 Cycles:8 
            lr35902@TestBit CB7D, 8,  L_Read, 7 ;; BIT L, 7 Cycles:8        
            lr35902@TestBit CB7E,16,  xHL_Read, 7 ;; BIT xHL, 7 Cycles:16       
            lr35902@TestBit CB7F, 8,  A_Read, 7 ;; BIT A, 7 Cycles:8 
            
            lr35902@ResBit CB80, 8,  B_Read, 0, B_Write ;; RES B, 0 Cycles:8 
            lr35902@ResBit CB81, 8,  C_Read, 0, C_Write ;; RES C, 0 Cycles:8        
            lr35902@ResBit CB82, 8,  D_Read, 0, D_Write ;; RES D, 0 Cycles:8        
            lr35902@ResBit CB83, 8,  E_Read, 0, E_Write ;; RES E, 0 Cycles:8     
            lr35902@ResBit CB84, 8,  H_Read, 0, H_Write ;; RES H, 0 Cycles:8 
            lr35902@ResBit CB85, 8,  L_Read, 0, L_Write ;; RES L, 0 Cycles:8        
            lr35902@ResBit CB86,16,  xHL_Read, 0, xHL_Write ;; RES xHL, 0 Cycles:16       
            lr35902@ResBit CB87, 8,  A_Read, 0, A_Write ;; RES A, 0 Cycles:8         
          
            lr35902@ResBit CB88, 8,  B_Read, 1, B_Write ;; RES B, 1 Cycles:8 
            lr35902@ResBit CB89, 8,  C_Read, 1, C_Write ;; RES C, 1 Cycles:8        
            lr35902@ResBit CB8A, 8,  D_Read, 1, D_Write ;; RES D, 1 Cycles:8        
            lr35902@ResBit CB8B, 8,  E_Read, 1, E_Write ;; RES E, 1 Cycles:8     
            lr35902@ResBit CB8C, 8,  H_Read, 1, H_Write ;; RES H, 1 Cycles:8 
            lr35902@ResBit CB8D, 8,  L_Read, 1, L_Write ;; RES L, 1 Cycles:8        
            lr35902@ResBit CB8E,16,  xHL_Read, 1, xHL_Write ;; RES xHL, 1 Cycles:16       
            lr35902@ResBit CB8F, 8,  A_Read, 1, A_Write ;; RES A, 1 Cycles:8 

            lr35902@ResBit CB90, 8,  B_Read, 2, B_Write ;; RES B, 2 Cycles:8 
            lr35902@ResBit CB91, 8,  C_Read, 2, C_Write ;; RES C, 2 Cycles:8        
            lr35902@ResBit CB92, 8,  D_Read, 2, D_Write ;; RES D, 2 Cycles:8        
            lr35902@ResBit CB93, 8,  E_Read, 2, E_Write ;; RES E, 2 Cycles:8     
            lr35902@ResBit CB94, 8,  H_Read, 2, H_Write ;; RES H, 2 Cycles:8 
            lr35902@ResBit CB95, 8,  L_Read, 2, L_Write ;; RES L, 2 Cycles:8        
            lr35902@ResBit CB96,16,  xHL_Read, 2, xHL_Write ;; RES xHL, 2 Cycles:16       
            lr35902@ResBit CB97, 8,  A_Read, 2, A_Write ;; RES A, 2 Cycles:8         
          
            lr35902@ResBit CB98, 8,  B_Read, 3, B_Write ;; RES B, 3 Cycles:8 
            lr35902@ResBit CB99, 8,  C_Read, 3, C_Write ;; RES C, 3 Cycles:8        
            lr35902@ResBit CB9A, 8,  D_Read, 3, D_Write ;; RES D, 3 Cycles:8        
            lr35902@ResBit CB9B, 8,  E_Read, 3, E_Write ;; RES E, 3 Cycles:8     
            lr35902@ResBit CB9C, 8,  H_Read, 3, H_Write ;; RES H, 3 Cycles:8 
            lr35902@ResBit CB9D, 8,  L_Read, 3, L_Write ;; RES L, 3 Cycles:8        
            lr35902@ResBit CB9E,16,  xHL_Read, 3, xHL_Write ;; RES xHL, 3 Cycles:16       
            lr35902@ResBit CB9F, 8,  A_Read, 3, A_Write ;; RES A, 3 Cycles:8 

            lr35902@ResBit CBA0, 8,  B_Read, 4, B_Write ;; RES B, 4 Cycles:8 
            lr35902@ResBit CBA1, 8,  C_Read, 4, C_Write ;; RES C, 4 Cycles:8        
            lr35902@ResBit CBA2, 8,  D_Read, 4, D_Write ;; RES D, 4 Cycles:8        
            lr35902@ResBit CBA3, 8,  E_Read, 4, E_Write ;; RES E, 4 Cycles:8     
            lr35902@ResBit CBA4, 8,  H_Read, 4, H_Write ;; RES H, 4 Cycles:8 
            lr35902@ResBit CBA5, 8,  L_Read, 4, L_Write ;; RES L, 4 Cycles:8        
            lr35902@ResBit CBA6,16,  xHL_Read, 4, xHL_Write ;; RES xHL, 4 Cycles:16       
            lr35902@ResBit CBA7, 8,  A_Read, 4, A_Write ;; RES A, 4 Cycles:8         
          
            lr35902@ResBit CBA8, 8,  B_Read, 5, B_Write ;; RES B, 5 Cycles:8 
            lr35902@ResBit CBA9, 8,  C_Read, 5, C_Write ;; RES C, 5 Cycles:8        
            lr35902@ResBit CBAA, 8,  D_Read, 5, D_Write ;; RES D, 5 Cycles:8        
            lr35902@ResBit CBAB, 8,  E_Read, 5, E_Write ;; RES E, 5 Cycles:8     
            lr35902@ResBit CBAC, 8,  H_Read, 5, H_Write ;; RES H, 5 Cycles:8 
            lr35902@ResBit CBAD, 8,  L_Read, 5, L_Write ;; RES L, 5 Cycles:8        
            lr35902@ResBit CBAE,16,  xHL_Read, 5, xHL_Write ;; RES xHL, 5 Cycles:16       
            lr35902@ResBit CBAF, 8,  A_Read, 5, A_Write ;; RES A, 5 Cycles:8 
            
            lr35902@ResBit CBB0, 8,  B_Read, 6, B_Write ;; RES B, 6 Cycles:8 
            lr35902@ResBit CBB1, 8,  C_Read, 6, C_Write ;; RES C, 6 Cycles:8        
            lr35902@ResBit CBB2, 8,  D_Read, 6, D_Write ;; RES D, 6 Cycles:8        
            lr35902@ResBit CBB3, 8,  E_Read, 6, E_Write ;; RES E, 6 Cycles:8     
            lr35902@ResBit CBB4, 8,  H_Read, 6, H_Write ;; RES H, 6 Cycles:8 
            lr35902@ResBit CBB5, 8,  L_Read, 6, L_Write ;; RES L, 6 Cycles:8        
            lr35902@ResBit CBB6,16,  xHL_Read, 6, xHL_Write ;; RES xHL, 6 Cycles:16       
            lr35902@ResBit CBB7, 8,  A_Read, 6, A_Write ;; RES A, 6 Cycles:8         
          
            lr35902@ResBit CBB8, 8,  B_Read, 7, B_Write ;; RES B, 7 Cycles:8 
            lr35902@ResBit CBB9, 8,  C_Read, 7, C_Write ;; RES C, 7 Cycles:8        
            lr35902@ResBit CBBA, 8,  D_Read, 7, D_Write ;; RES D, 7 Cycles:8        
            lr35902@ResBit CBBB, 8,  E_Read, 7, E_Write ;; RES E, 7 Cycles:8     
            lr35902@ResBit CBBC, 8,  H_Read, 7, H_Write ;; RES H, 7 Cycles:8 
            lr35902@ResBit CBBD, 8,  L_Read, 7, L_Write ;; RES L, 7 Cycles:8        
            lr35902@ResBit CBBE,16,  xHL_Read, 7, xHL_Write ;; RES xHL, 7 Cycles:16       
            lr35902@ResBit CBBF, 8,  A_Read, 7, A_Write ;; RES A, 7 Cycles:8 

            lr35902@SetBit CBC0, 8,  B_Read, 0, B_Write ;; SET B, 0 Cycles:8 
            lr35902@SetBit CBC1, 8,  C_Read, 0, C_Write ;; SET C, 0 Cycles:8        
            lr35902@SetBit CBC2, 8,  D_Read, 0, D_Write ;; SET D, 0 Cycles:8        
            lr35902@SetBit CBC3, 8,  E_Read, 0, E_Write ;; SET E, 0 Cycles:8     
            lr35902@SetBit CBC4, 8,  H_Read, 0, H_Write ;; SET H, 0 Cycles:8 
            lr35902@SetBit CBC5, 8,  L_Read, 0, L_Write ;; SET L, 0 Cycles:8        
            lr35902@SetBit CBC6,16,  xHL_Read, 0, xHL_Write ;; SET xHL, 0 Cycles:16       
            lr35902@SetBit CBC7, 8,  A_Read, 0, A_Write ;; SET A, 0 Cycles:8         
          
            lr35902@SetBit CBC8, 8,  B_Read, 1, B_Write ;; SET B, 1 Cycles:8 
            lr35902@SetBit CBC9, 8,  C_Read, 1, C_Write ;; SET C, 1 Cycles:8        
            lr35902@SetBit CBCA, 8,  D_Read, 1, D_Write ;; SET D, 1 Cycles:8        
            lr35902@SetBit CBCB, 8,  E_Read, 1, E_Write ;; SET E, 1 Cycles:8     
            lr35902@SetBit CBCC, 8,  H_Read, 1, H_Write ;; SET H, 1 Cycles:8 
            lr35902@SetBit CBCD, 8,  L_Read, 1, L_Write ;; SET L, 1 Cycles:8        
            lr35902@SetBit CBCE,16,  xHL_Read, 1, xHL_Write ;; SET xHL, 1 Cycles:16       
            lr35902@SetBit CBCF, 8,  A_Read, 1, A_Write ;; SET A, 1 Cycles:8 

            lr35902@SetBit CBD0, 8,  B_Read, 2, B_Write ;; SET B, 2 Cycles:8 
            lr35902@SetBit CBD1, 8,  C_Read, 2, C_Write ;; SET C, 2 Cycles:8        
            lr35902@SetBit CBD2, 8,  D_Read, 2, D_Write ;; SET D, 2 Cycles:8        
            lr35902@SetBit CBD3, 8,  E_Read, 2, E_Write ;; SET E, 2 Cycles:8     
            lr35902@SetBit CBD4, 8,  H_Read, 2, H_Write ;; SET H, 2 Cycles:8 
            lr35902@SetBit CBD5, 8,  L_Read, 2, L_Write ;; SET L, 2 Cycles:8        
            lr35902@SetBit CBD6,16,  xHL_Read, 2, xHL_Write ;; SET xHL, 2 Cycles:16       
            lr35902@SetBit CBD7, 8,  A_Read, 2, A_Write ;; SET A, 2 Cycles:8         
          
            lr35902@SetBit CBD8, 8,  B_Read, 3, B_Write ;; SET B, 3 Cycles:8 
            lr35902@SetBit CBD9, 8,  C_Read, 3, C_Write ;; SET C, 3 Cycles:8        
            lr35902@SetBit CBDA, 8,  D_Read, 3, D_Write ;; SET D, 3 Cycles:8        
            lr35902@SetBit CBDB, 8,  E_Read, 3, E_Write ;; SET E, 3 Cycles:8     
            lr35902@SetBit CBDC, 8,  H_Read, 3, H_Write ;; SET H, 3 Cycles:8 
            lr35902@SetBit CBDD, 8,  L_Read, 3, L_Write ;; SET L, 3 Cycles:8        
            lr35902@SetBit CBDE,16,  xHL_Read, 3, xHL_Write ;; SET xHL, 3 Cycles:16       
            lr35902@SetBit CBDF, 8,  A_Read, 3, A_Write ;; SET A, 3 Cycles:8 

            lr35902@SetBit CBE0, 8,  B_Read, 4, B_Write ;; SET B, 4 Cycles:8 
            lr35902@SetBit CBE1, 8,  C_Read, 4, C_Write ;; SET C, 4 Cycles:8        
            lr35902@SetBit CBE2, 8,  D_Read, 4, D_Write ;; SET D, 4 Cycles:8        
            lr35902@SetBit CBE3, 8,  E_Read, 4, E_Write ;; SET E, 4 Cycles:8     
            lr35902@SetBit CBE4, 8,  H_Read, 4, H_Write ;; SET H, 4 Cycles:8 
            lr35902@SetBit CBE5, 8,  L_Read, 4, L_Write ;; SET L, 4 Cycles:8        
            lr35902@SetBit CBE6,16,  xHL_Read, 4, xHL_Write ;; SET xHL, 4 Cycles:16       
            lr35902@SetBit CBE7, 8,  A_Read, 4, A_Write ;; SET A, 4 Cycles:8         
          
            lr35902@SetBit CBE8, 8,  B_Read, 5, B_Write ;; SET B, 5 Cycles:8 
            lr35902@SetBit CBE9, 8,  C_Read, 5, C_Write ;; SET C, 5 Cycles:8        
            lr35902@SetBit CBEA, 8,  D_Read, 5, D_Write ;; SET D, 5 Cycles:8        
            lr35902@SetBit CBEB, 8,  E_Read, 5, E_Write ;; SET E, 5 Cycles:8     
            lr35902@SetBit CBEC, 8,  H_Read, 5, H_Write ;; SET H, 5 Cycles:8 
            lr35902@SetBit CBED, 8,  L_Read, 5, L_Write ;; SET L, 5 Cycles:8        
            lr35902@SetBit CBEE,16,  xHL_Read, 5, xHL_Write ;; SET xHL, 5 Cycles:16       
            lr35902@SetBit CBEF, 8,  A_Read, 5, A_Write ;; SET A, 5 Cycles:8 
            
            lr35902@SetBit CBF0, 8,  B_Read, 6, B_Write ;; SET B, 6 Cycles:8 
            lr35902@SetBit CBF1, 8,  C_Read, 6, C_Write ;; SET C, 6 Cycles:8        
            lr35902@SetBit CBF2, 8,  D_Read, 6, D_Write ;; SET D, 6 Cycles:8        
            lr35902@SetBit CBF3, 8,  E_Read, 6, E_Write ;; SET E, 6 Cycles:8     
            lr35902@SetBit CBF4, 8,  H_Read, 6, H_Write ;; SET H, 6 Cycles:8 
            lr35902@SetBit CBF5, 8,  L_Read, 6, L_Write ;; SET L, 6 Cycles:8        
            lr35902@SetBit CBF6,16,  xHL_Read, 6, xHL_Write ;; SET xHL, 6 Cycles:16       
            lr35902@SetBit CBF7, 8,  A_Read, 6, A_Write ;; SET A, 6 Cycles:8         
          
            lr35902@SetBit CBF8, 8,  B_Read, 7, B_Write ;; SET B, 7 Cycles:8 
            lr35902@SetBit CBF9, 8,  C_Read, 7, C_Write ;; SET C, 7 Cycles:8        
            lr35902@SetBit CBFA, 8,  D_Read, 7, D_Write ;; SET D, 7 Cycles:8        
            lr35902@SetBit CBFB, 8,  E_Read, 7, E_Write ;; SET E, 7 Cycles:8     
            lr35902@SetBit CBFC, 8,  H_Read, 7, H_Write ;; SET H, 7 Cycles:8 
            lr35902@SetBit CBFD, 8,  L_Read, 7, L_Write ;; SET L, 7 Cycles:8        
            lr35902@SetBit CBFE,16,  xHL_Read, 7, xHL_Write ;; SET xHL, 7 Cycles:16       
            lr35902@SetBit CBFF, 8,  A_Read, 7, A_Write ;; SET A, 7 Cycles:8    
        
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; --------------------------------------------------------------
        ;; Ub Code for lr35902 
        ;; --------------------------------------------------------------
        
      OPDD: ;; DD Perfix(IX) for Z80 
      OPFD: ;; FD Perfix(IY) for Z80 
      OPED: ;; ED Perfix(EXTD) for Z80     
      OPD3: ;; out (*),a for Z80 
      OPDB: ;; in a,(*) for Z80 
      OPE3: ;; ex (sp),hl for Z80
      OPE4: ;; call po,** for Z80 
      OPEB: ;; ex de,hl for Z80 
      OPEC: ;; call pe,** for Z80 
      OPF4: ;; call p,** for Z80 
      OPFC: ;; call m,** for Z80 
        int 3
      OP00: ;; NOP 
        mov [YG_GP+lr35902.PC], si
        mov eax, 4 
V_EXIT:
        mov dword [YG_GP+lr35902._backup], 0
        pop esi 
        pop edi 
        pop ebx 
        ret 
        
        
        align 16      
OPTAB   dd  OP00, OP01, OP02, OP03, OP04, OP05, OP06, OP07, OP08, OP09, OP0A, OP0B, OP0C, OP0D, OP0E, OP0F,\
            OP10, OP11, OP12, OP13, OP14, OP15, OP16, OP17, OP18, OP19, OP1A, OP1B, OP1C, OP1D, OP1E, OP1F,\
            OP20, OP21, OP22, OP23, OP24, OP25, OP26, OP27, OP28, OP29, OP2A, OP2B, OP2C, OP2D, OP2E, OP2F,\
            OP30, OP31, OP32, OP33, OP34, OP35, OP36, OP37, OP38, OP39, OP3A, OP3B, OP3C, OP3D, OP3E, OP3F,\
            OP40, OP41, OP42, OP43, OP44, OP45, OP46, OP47, OP48, OP49, OP4A, OP4B, OP4C, OP4D, OP4E, OP4F,\
            OP50, OP51, OP52, OP53, OP54, OP55, OP56, OP57, OP58, OP59, OP5A, OP5B, OP5C, OP5D, OP5E, OP5F,\
            OP60, OP61, OP62, OP63, OP64, OP65, OP66, OP67, OP68, OP69, OP6A, OP6B, OP6C, OP6D, OP6E, OP6F,\
            OP70, OP71, OP72, OP73, OP74, OP75, OP76, OP77, OP78, OP79, OP7A, OP7B, OP7C, OP7D, OP7E, OP7F,\
            OP80, OP81, OP82, OP83, OP84, OP85, OP86, OP87, OP88, OP89, OP8A, OP8B, OP8C, OP8D, OP8E, OP8F,\
            OP90, OP91, OP92, OP93, OP94, OP95, OP96, OP97, OP98, OP99, OP9A, OP9B, OP9C, OP9D, OP9E, OP9F,\
            OPA0, OPA1, OPA2, OPA3, OPA4, OPA5, OPA6, OPA7, OPA8, OPA9, OPAA, OPAB, OPAC, OPAD, OPAE, OPAF,\
            OPB0, OPB1, OPB2, OPB3, OPB4, OPB5, OPB6, OPB7, OPB8, OPB9, OPBA, OPBB, OPBC, OPBD, OPBE, OPBF,\
            OPC0, OPC1, OPC2, OPC3, OPC4, OPC5, OPC6, OPC7, OPC8, OPC9, OPCA, OPCB, OPCC, OPCD, OPCE, OPCF,\
            OPD0, OPD1, OPD2, OPD3, OPD4, OPD5, OPD6, OPD7, OPD8, OPD9, OPDA, OPDB, OPDC, OPDD, OPDE, OPDF,\
            OPE0, OPE1, OPE2, OPE3, OPE4, OPE5, OPE6, OPE7, OPE8, OPE9, OPEA, OPEB, OPEC, OPED, OPEE, OPEF,\
            OPF0, OPF1, OPF2, OPF3, OPF4, OPF5, OPF6, OPF7, OPF8, OPF9, OPFA, OPFB, OPFC, OPFD, OPFE, OPFF
CBTAB   dd  CB00, CB01, CB02, CB03, CB04, CB05, CB06, CB07, CB08, CB09, CB0A, CB0B, CB0C, CB0D, CB0E, CB0F,\
            CB10, CB11, CB12, CB13, CB14, CB15, CB16, CB17, CB18, CB19, CB1A, CB1B, CB1C, CB1D, CB1E, CB1F,\
            CB20, CB21, CB22, CB23, CB24, CB25, CB26, CB27, CB28, CB29, CB2A, CB2B, CB2C, CB2D, CB2E, CB2F,\
            CB30, CB31, CB32, CB33, CB34, CB35, CB36, CB37, CB38, CB39, CB3A, CB3B, CB3C, CB3D, CB3E, CB3F,\
            CB40, CB41, CB42, CB43, CB44, CB45, CB46, CB47, CB48, CB49, CB4A, CB4B, CB4C, CB4D, CB4E, CB4F,\
            CB50, CB51, CB52, CB53, CB54, CB55, CB56, CB57, CB58, CB59, CB5A, CB5B, CB5C, CB5D, CB5E, CB5F,\
            CB60, CB61, CB62, CB63, CB64, CB65, CB66, CB67, CB68, CB69, CB6A, CB6B, CB6C, CB6D, CB6E, CB6F,\
            CB70, CB71, CB72, CB73, CB74, CB75, CB76, CB77, CB78, CB79, CB7A, CB7B, CB7C, CB7D, CB7E, CB7F,\
            CB80, CB81, CB82, CB83, CB84, CB85, CB86, CB87, CB88, CB89, CB8A, CB8B, CB8C, CB8D, CB8E, CB8F,\
            CB90, CB91, CB92, CB93, CB94, CB95, CB96, CB97, CB98, CB99, CB9A, CB9B, CB9C, CB9D, CB9E, CB9F,\
            CBA0, CBA1, CBA2, CBA3, CBA4, CBA5, CBA6, CBA7, CBA8, CBA9, CBAA, CBAB, CBAC, CBAD, CBAE, CBAF,\
            CBB0, CBB1, CBB2, CBB3, CBB4, CBB5, CBB6, CBB7, CBB8, CBB9, CBBA, CBBB, CBBC, CBBD, CBBE, CBBF,\
            CBC0, CBC1, CBC2, CBC3, CBC4, CBC5, CBC6, CBC7, CBC8, CBC9, CBCA, CBCB, CBCC, CBCD, CBCE, CBCF,\
            CBD0, CBD1, CBD2, CBD3, CBD4, CBD5, CBD6, CBD7, CBD8, CBD9, CBDA, CBDB, CBDC, CBDD, CBDE, CBDF,\
            CBE0, CBE1, CBE2, CBE3, CBE4, CBE5, CBE6, CBE7, CBE8, CBE9, CBEA, CBEB, CBEC, CBED, CBEE, CBEF,\
            CBF0, CBF1, CBF2, CBF3, CBF4, CBF5, CBF6, CBF7, CBF8, CBF9, CBFA, CBFB, CBFC, CBFD, CBFE, CBFF       