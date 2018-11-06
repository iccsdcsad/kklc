;; cpu_optick.asm  (old version for MASM Assembler)
;; Sharp LR35902 Chip Opcode for GameBoy 
;; 
;; Copyright (C) 2018 moecmks
;; This file is part of KS3578.
;; 
;; do What The Fuck you want to Public License
;; 
;; Version 1.0, March 2000
;; Copyright (C) 2000 Banlu Kemiyatorn (]d).
;; 136 Nives 7 Jangwattana 14 Laksi Bangkok
;; Everyone is permitted to copy and distribute verbatim copies
;; of this license document, but changing it is not allowed.
;; 
;; Ok, the purpose of this license is simple
;; and you just
;; 
;; DO WHAT THE FUCK YOU WANT TO.
;;

  .686                      ; create 32 bit code           
  .model flat, stdcall      ; 32 bit memory model
  option casemap :none      ; case sensitive

;; Sharp LR35902 Chip opcode mapper 
;; http://www.pastraiser.com/cpu/gameboy/gameboy_opcodes.html 
;; Z80 Chip opcode mapper 
;; http://clrhome.org/table/

;; Register F Mask
;; The register flag field is different from the standard, in order to optimize instruction operation.
Z_FLAG equ 040H
H_FLAG equ 010H
N_FLAG equ 002H
C_FLAG equ 001H
ZC_FLAG equ 041H

;; extern memory/IO read/write
extrn gameboy_mmu_read@8:proc  ;; prototype ks_uint8 __stdcall gameboy_mmu_read (void;;gameboy, ks_uint16 addresss)
extrn gameboy_mmu_write@12:proc ;; prototype void __stdcall gameboy_mmu_write (void;;gameboy, ks_uint16 addresss, ks_uint8 value)
extrn gameboy_mmu_read_w@8:proc ;; prototype ks_uint16 __stdcall gameboy_mmu_read_w (void;;gameboy, ks_uint16 addresss)
extrn gameboy_mmu_write_w@12:proc ;; prototype void __stdcall gameboy_mmu_write_w (void;;gameboy, ks_uint16 addresss, ks_uint16 value)

;;  define union .
defREG macro lo, hi 
  union 
    struct 
      lo db ?
      hi db ?
    ends 
    hi&lo dw ?
  ends 
endm 
  
defREG2 macro lo, hi, blk 
  union 
    struct 
      lo db ?
      hi db ?
    ends 
    blk dw ?
  ends 
endm 

;;  XXX:memory order dep. 
cpu struct
  defREG F, A  ;; A (Accumulator)
               ;; probably the most commonly used register. 
               ;; Many instructions have special extended operation codes for accumulators.
               ;; F (Program status byte) 
               
  ;; defREG C, B ;; A2008: syntax error : C
  union 
    struct 
      C_ db ?
      B db ?
    ends 
    BC dw ?
  ends           ;; Register BC, without special explanation.
  defREG E, D    ;; Register DE, without special explanation.
  defREG L, H    ;; Register HL, which is mostly used to address 16-bit address data, 
                 ;;                              also has the basic properties of BC, DE registers
  defREG2 SP_LO, SP_HI, SP_ ;; Stack Pointer. 
  defREG2 PC_LO, PC_HI, PC ;; Program Pointer. 
  
  IME db ?        ;; Interrupt Master Enable
  
  halt dd ?     ;; for Halt 
  
  stop dd ?     ;; for stop 
  _backup dd ?     ;; for halt bug 
  key1 db ?  
  
  gameboy dd ?
cpu ends

    .code

;;  prototype ks_int32 cpu_optick (struct cpu;;cpu_);
cpu_optick proc C 
             option prologue:none, epilogue:none

        push ebx ;U -  save old frame       
        push edi ;V -  save old frame 
        push esi 
        nop   
        
YG_PF_8 equ bl        
YG_PF equ ebx 
YG_PC equ esi 
YG_GP equ edi 

_GB_INT3_ASSERT macro address 
  .if [YG_GP].PC == address
    int 3
  .endif 
endm  
        ; ebx <- save now P (cpu's PSB reg)
        ; esi <- save now PC (cpu's EIP reg)
        ; edi <- save regs root
        ; eax <- calc temp or final calc out reslt 
        ; ecx <- calc temp 
        ; edx <- calc temp 
        
        mov YG_GP, [esp+4+12]   ;; fetch CPU struct 
        mov si, [YG_GP].PC  
        mov bl, [YG_GP].F
        assume edi:ptr cpu
;;_GB_INT3_ASSERT 0C000H
        ; Fetch Opcode, PC++ 
        push YG_PC 
        push [YG_GP].gameboy
        call gameboy_mmu_read@8
        inc YG_PC
        and eax, 255
        add esi, [YG_GP]._backup 
        jmp dword ptr[OPTAB+eax*4]
        
EmptyMacro macro 
endm  

SetCyclesAndRet macro Cycles 
  ;; write back PC 
  mov [YG_GP].PC, si
  mov eax, Cycles
  jmp V_EXIT 
endm 

SetCyclesRetP macro Cycles 
  ;; write back PC 
  mov [YG_GP].F, bl
  SetCyclesAndRet Cycles
endm 

;;  Register access-write unwind 
B_Write equ mov [YG_GP].B, al
C_Write equ mov [YG_GP].C_, al
D_Write equ mov [YG_GP].D, al
E_Write equ mov [YG_GP].E, al
H_Write equ mov [YG_GP].H, al
L_Write equ mov [YG_GP].L, al
A_Write equ mov [YG_GP].A, al
F_Write equ mov [YG_GP].F, al
SP_Write equ mov [YG_GP].SP_, ax
DE_Write equ mov [YG_GP].DE, ax
BC_Write equ mov [YG_GP].BC, ax
HL_Write equ mov [YG_GP].HL, ax
AF_Write equ mov [YG_GP].AF, ax
PC_Write equ mov [YG_GP].PC, ax

;; (Imm16) := Z80Register:BYTE 
xImm16_WriteReg8 macro _Reg 
  mov cl, [YG_GP]._Reg 
  push ecx 
  push eax 
  push [YG_GP].gameboy
  call gameboy_mmu_write@12  
endm

;; (Imm16) := Z80Register:WORD 
xImm16_WriteReg16 macro _Reg 
  mov cx, [YG_GP]._Reg 
  push ecx 
  push eax 
  push [YG_GP].gameboy
  call gameboy_mmu_write_w@12  
endm

;; (Imm16) := Z80Register-SP 
xImm16_WriteRegSP macro
  xImm16_WriteReg16 SP_ 
endm

;; (Imm16) := Z80Register-A 
xImm16_WriteRegA macro
  xImm16_WriteReg8 A
endm

;; (Z80Register:WORD ) :=  Value8:eax::al
xRegister_WriteReg8 macro _Reg 
  push eax 
  mov ax, [YG_GP]._Reg 
  push eax
  push [YG_GP].gameboy
  call gameboy_mmu_write@12 
endm

;; Unwind4
xBC_Write equ xRegister_WriteReg8 BC  
xDE_Write equ xRegister_WriteReg8 DE
xHL_Write equ xRegister_WriteReg8 HL
xAF_Write equ xRegister_WriteReg8 AF
xPC_Write equ xRegister_WriteReg8 PC

xHL_Write_Inc macro 
  xHL_Write
  inc [YG_GP].HL
endm 

xHL_Write_Dec macro 
  xHL_Write
  dec [YG_GP].HL
endm 

;;  Register access-read unwind 
B_Read equ  mov al, [YG_GP].B
C_Read equ  mov al, [YG_GP].C_
D_Read equ  mov al, [YG_GP].D
E_Read equ  mov al, [YG_GP].E
H_Read equ  mov al, [YG_GP].H
L_Read equ  mov al, [YG_GP].L
A_Read equ  mov al, [YG_GP].A
BC_Read equ  mov ax, [YG_GP].BC
DE_Read equ  mov ax, [YG_GP].DE
HL_Read equ  mov ax, [YG_GP].HL
SP_Read equ  mov ax, [YG_GP].SP_
AF_Read equ  mov ax, [YG_GP].AF

;; Imm8:eax::al := FetchPC++
Imm8_Read macro 
  push    esi 
  push   [YG_GP].gameboy
  call    gameboy_mmu_read@8
  inc     esi 
endm 

;; Imm16:eax::ax := FetchPC, FetchPC +=2
Imm16_Read macro 
  push    esi 
  push   [YG_GP].gameboy
  call    gameboy_mmu_read_w@8
  add     esi, 2
endm 

Read_ByxX86SpecRegister macro _Reg
  ;;  _Reg:x86Register 
  push    _Reg 
  push   [YG_GP].gameboy
  call    gameboy_mmu_read@8
endm

Imm8Read_ExpandAddress16 macro 
  Imm8_Read 
  and eax, 000FFh
  add eax, 0FF00h 
endm 

Imm8_ExpandSignWord macro 
  Imm8_Read 
  movsx eax, al 
endm 

C_ExpandAddress16 macro 
  movzx eax, [YG_GP].C_
  add eax, 0FF00h 
endm 

Imm8Read_ExpandAddress16_Fetch macro 
  Imm8Read_ExpandAddress16
  Read_ByxX86SpecRegister eax 
endm 

Imm16Read_Address_Fetch macro 
  Imm16_Read
  Read_ByxX86SpecRegister eax 
endm 

C_ExpandAddress16_Fetch macro 
  mov al, [YG_GP].C_
  add eax, 0FF00h 
  Read_ByxX86SpecRegister eax 
endm 

xRegister_Read macro _Reg 
  mov ax, [YG_GP].&_Reg
  push eax
  push [YG_GP].gameboy
  call gameboy_mmu_read@8
endm 

;; Unwind5
xBC_Read equ xRegister_Read BC  
xDE_Read equ xRegister_Read DE
xHL_Read equ xRegister_Read HL
xAF_Read equ xRegister_Read AF
xPC_Read equ xRegister_Read PC 

xHL_Read_Inc macro 
  xHL_Read
  inc [YG_GP].HL
endm 

xHL_Read_Dec macro 
  xHL_Read
  dec [YG_GP].HL
endm 

LD@Imm8 macro OpCase, Cycles_, WriteOrExt
 OpCase&:
    Imm8_Read
    WriteOrExt
    SetCyclesAndRet Cycles_ 
endm 
      LD@Imm8 OP06, 8,  B_Write ;; LD B Imm8, 2, Cycles:8 
      LD@Imm8 OP0E, 8,  C_Write ;; LD C Imm8, 2, Cycles:8 
      LD@Imm8 OP16, 8,  D_Write ;; LD D Imm8, 2, Cycles:8 
      LD@Imm8 OP1E, 8,  E_Write ;; LD E Imm8, 2, Cycles:8 
      LD@Imm8 OP26, 8,  H_Write ;; LD H Imm8, 2, Cycles:8 
      LD@Imm8 OP2E, 8,  L_Write ;; LD L Imm8, 2, Cycles:8 
      LD@Imm8 OP36,12,  xHL_Write ;; LD xHL Imm8, 2, Cycles:8 
      LD@Imm8 OP3E, 8,  A_Write ;; LD A Imm8, 2, Cycles:8 
   
LD@Imm16 macro OpCase, Cycles_, lrReg 
 OpCase&:
    Imm16_Read
    mov [YG_GP].&lrReg, ax 
    SetCyclesAndRet Cycles_ 
endm     
      LD@Imm16 OP01, 12, BC ;; LD BC Imm16, 3 Cycles:12
      LD@Imm16 OP11, 12, DE ;; LD DE Imm16, 3 Cycles:12     
      LD@Imm16 OP21, 12, HL ;; LD HL Imm16, 3 Cycles:12     
      LD@Imm16 OP31, 12, SP_ ;; LD SP Imm16, 3 Cycles:12      
  
LD@RxHLToRxHL macro OpCase, Cycles_, ReadOrExt, WriteOrExt
 OpCase&:
    ReadOrExt
    WriteOrExt
    SetCyclesAndRet Cycles_ 
endm      
      LD@RxHLToRxHL OP40, 4, B_Read, B_Write ;; LD B B, 1 Cycles:4
      LD@RxHLToRxHL OP41, 4, C_Read, B_Write ;; LD B C, 1 Cycles:4
      LD@RxHLToRxHL OP42, 4, D_Read, B_Write ;; LD B D, 1 Cycles:4
      LD@RxHLToRxHL OP43, 4, E_Read, B_Write ;; LD B E, 1 Cycles:4
      LD@RxHLToRxHL OP44, 4, H_Read, B_Write ;; LD B H, 1 Cycles:4
      LD@RxHLToRxHL OP45, 4, L_Read, B_Write ;; LD B L, 1 Cycles:4
      LD@RxHLToRxHL OP46, 8, xHL_Read, B_Write ;; LD B xHL, 1 Cycles:8
      LD@RxHLToRxHL OP47, 4, A_Read, B_Write ;; LD B A, 1 Cycles:4   
      
      LD@RxHLToRxHL OP48, 4, B_Read, C_Write ;; LD C B, 1 Cycles:4
      LD@RxHLToRxHL OP49, 4, C_Read, C_Write ;; LD C C, 1 Cycles:4
      LD@RxHLToRxHL OP4A, 4, D_Read, C_Write ;; LD C D, 1 Cycles:4
      LD@RxHLToRxHL OP4B, 4, E_Read, C_Write ;; LD C E, 1 Cycles:4
      LD@RxHLToRxHL OP4C, 4, H_Read, C_Write ;; LD C H, 1 Cycles:4
      LD@RxHLToRxHL OP4D, 4, L_Read, C_Write ;; LD C L, 1 Cycles:4
      LD@RxHLToRxHL OP4E, 8, xHL_Read, C_Write ;; LD C xHL, 1 Cycles:8
      LD@RxHLToRxHL OP4F, 4, A_Read, C_Write ;; LD C A, 1 Cycles:4  

      LD@RxHLToRxHL OP50, 4, B_Read, D_Write ;; LD D B, 1 Cycles:4
      LD@RxHLToRxHL OP51, 4, C_Read, D_Write ;; LD D C, 1 Cycles:4
      LD@RxHLToRxHL OP52, 4, D_Read, D_Write ;; LD D D, 1 Cycles:4
      LD@RxHLToRxHL OP53, 4, E_Read, D_Write ;; LD D E, 1 Cycles:4
      LD@RxHLToRxHL OP54, 4, H_Read, D_Write ;; LD D H, 1 Cycles:4
      LD@RxHLToRxHL OP55, 4, L_Read, D_Write ;; LD D L, 1 Cycles:4
      LD@RxHLToRxHL OP56, 8, xHL_Read, D_Write ;; LD D xHL, 1 Cycles:8
      LD@RxHLToRxHL OP57, 4, A_Read, D_Write ;; LD D A, 1 Cycles:4   
      
      LD@RxHLToRxHL OP58, 4, B_Read, E_Write ;; LD E B, 1 Cycles:4
      LD@RxHLToRxHL OP59, 4, C_Read, E_Write ;; LD E C, 1 Cycles:4
      LD@RxHLToRxHL OP5A, 4, D_Read, E_Write ;; LD E D, 1 Cycles:4
      LD@RxHLToRxHL OP5B, 4, E_Read, E_Write ;; LD E E, 1 Cycles:4
      LD@RxHLToRxHL OP5C, 4, H_Read, E_Write ;; LD E H, 1 Cycles:4
      LD@RxHLToRxHL OP5D, 4, L_Read, E_Write ;; LD E L, 1 Cycles:4
      LD@RxHLToRxHL OP5E, 8, xHL_Read, E_Write ;; LD E xHL, 1 Cycles:8
      LD@RxHLToRxHL OP5F, 4, A_Read, E_Write ;; LD E A, 1 Cycles:4  

      LD@RxHLToRxHL OP60, 4, B_Read, H_Write ;; LD H B, 1 Cycles:4
      LD@RxHLToRxHL OP61, 4, C_Read, H_Write ;; LD H C, 1 Cycles:4
      LD@RxHLToRxHL OP62, 4, D_Read, H_Write ;; LD H D, 1 Cycles:4
      LD@RxHLToRxHL OP63, 4, E_Read, H_Write ;; LD H E, 1 Cycles:4
      LD@RxHLToRxHL OP64, 4, H_Read, H_Write ;; LD H H, 1 Cycles:4
      LD@RxHLToRxHL OP65, 4, L_Read, H_Write ;; LD H L, 1 Cycles:4
      LD@RxHLToRxHL OP66, 8, xHL_Read, H_Write ;; LD H xHL, 1 Cycles:8
      LD@RxHLToRxHL OP67, 4, A_Read, H_Write ;; LD H A, 1 Cycles:4   
      
      LD@RxHLToRxHL OP68, 4, B_Read, L_Write ;; LD L B, 1 Cycles:4
      LD@RxHLToRxHL OP69, 4, C_Read, L_Write ;; LD L C, 1 Cycles:4
      LD@RxHLToRxHL OP6A, 4, D_Read, L_Write ;; LD L D, 1 Cycles:4
      LD@RxHLToRxHL OP6B, 4, E_Read, L_Write ;; LD L E, 1 Cycles:4
      LD@RxHLToRxHL OP6C, 4, H_Read, L_Write ;; LD L H, 1 Cycles:4
      LD@RxHLToRxHL OP6D, 4, L_Read, L_Write ;; LD L L, 1 Cycles:4
      LD@RxHLToRxHL OP6E, 8, xHL_Read, L_Write ;; LD L xHL, 1 Cycles:8
      LD@RxHLToRxHL OP6F, 4, A_Read, L_Write ;; LD L A, 1 Cycles:4  
        
      LD@RxHLToRxHL OP70, 8, B_Read, xHL_Write ;; LD xHL B, 1 Cycles:8
      LD@RxHLToRxHL OP71, 8, C_Read, xHL_Write ;; LD xHL C, 1 Cycles:8
      LD@RxHLToRxHL OP72, 8, D_Read, xHL_Write ;; LD xHL D, 1 Cycles:8
      LD@RxHLToRxHL OP73, 8, E_Read, xHL_Write ;; LD xHL E, 1 Cycles:8
      LD@RxHLToRxHL OP74, 8, H_Read, xHL_Write ;; LD xHL H, 1 Cycles:8
      LD@RxHLToRxHL OP75, 8, L_Read, xHL_Write ;; LD xHL L, 1 Cycles:8
      ;; LD@RxHLToRxHL OP76, 8, xHL_Read, xHL_Write ;; LD xHL xHL, 1 Cycles:8 
      LD@RxHLToRxHL OP02, 8, A_Read, xBC_Write ;; LD xBC A, 1 Cycles:8     
      LD@RxHLToRxHL OP12, 8, A_Read, xDE_Write ;; LD xDE A, 1 Cycles:8    
      LD@RxHLToRxHL OP77, 8, A_Read, xHL_Write ;; LD xHL A, 1 Cycles:8    
      LD@RxHLToRxHL OP22, 8, A_Read, xHL_Write_Inc;; LD xHL++ A, 1 Cycles:8  
      LD@RxHLToRxHL OP32, 8, A_Read, xHL_Write_Dec ;; LD xHL-- A, 1 Cycles:8  
        
      LD@RxHLToRxHL OP78, 4, B_Read, A_Write ;; LD A B, 1 Cycles:4
      LD@RxHLToRxHL OP79, 4, C_Read, A_Write ;; LD A C, 1 Cycles:4
      LD@RxHLToRxHL OP7A, 4, D_Read, A_Write ;; LD A D, 1 Cycles:4
      LD@RxHLToRxHL OP7B, 4, E_Read, A_Write ;; LD A E, 1 Cycles:4
      LD@RxHLToRxHL OP7C, 4, H_Read, A_Write ;; LD A H, 1 Cycles:4
      LD@RxHLToRxHL OP7D, 4, L_Read, A_Write ;; LD A L, 1 Cycles:4
      LD@RxHLToRxHL OP0A, 8, xBC_Read, A_Write ;; LD A xBC, 1 Cycles:8
      LD@RxHLToRxHL OP1A, 8, xDE_Read, A_Write ;; LD A xDE, 1 Cycles:8
      LD@RxHLToRxHL OP7E, 8, xHL_Read, A_Write ;; LD A xHL, 1 Cycles:8
      LD@RxHLToRxHL OP2A, 8, xHL_Read_Inc, A_Write ;; LD A xHL++, 1 Cycles:8
      LD@RxHLToRxHL OP3A, 8, xHL_Read_Dec, A_Write ;; LD A xHL--, 1 Cycles:8
      LD@RxHLToRxHL OP7F, 4, A_Read, A_Write ;; LD A A, 1 Cycles:4      
;; MISC LD 
      LD@RxHLToRxHL OP08,20, Imm16_Read, xImm16_WriteRegSP ;; LD (Imm16) SP, 3 Cycles:20
      LD@RxHLToRxHL OPEA,16, Imm16_Read, xImm16_WriteRegA ;; LD (Imm16) A, 3 Cycles:16
      LD@RxHLToRxHL OPE0,12, Imm8Read_ExpandAddress16, xImm16_WriteRegA ;; LD (Imm8+0FF00h) A, 2 Cycles:12
      LD@RxHLToRxHL OPE2, 8, C_ExpandAddress16, xImm16_WriteRegA ;; LD (C+0FF00h) A, 2 Cycles:8  
      LD@RxHLToRxHL OPF0,12, Imm8Read_ExpandAddress16_Fetch, A_Write ;; LD A, (Imm8+0FF00h)  2 Cycles:12
      LD@RxHLToRxHL OPF2, 8, C_ExpandAddress16_Fetch, A_Write ;; LD A, (C+0FF00h) 2 Cycles:8    
      LD@RxHLToRxHL OPFA,16, Imm16Read_Address_Fetch, A_Write ;; LD A, (Imm16) 3 Cycles:16
      LD@RxHLToRxHL OPF9, 8, HL_Read, SP_Write ;; LD SP HL 1 Cycles:8
      
      OPF8: ;; LD HL SP+Imm8(sign8) 2 Cycles:12 
      Imm8_Read
      
      ;; Clear Reg-f
      ;;xor YG_PF, YG_PF
      xor edx, edx 
      ;; ext sign 
      movsx ax, al 
      mov cx, [YG_GP].SP_
      and ecx, 0FFFFh
      and eax, 0FFFFh 
      lea edx, [ecx+eax]
      ;; SetH
      mov [YG_GP].HL, dx 
      xor cx, ax 
      mov ax, dx 
      xor cx, ax 
      and cx, 01000h
      shr cx, 8 
      ;;or YG_PF, ecx 
      ;; SetC 
      and dx, 010000h
      shr dx, 16 
      ;;or YG_PF, edx 
      SetCyclesRetP 12
      
      ;; ALU, LOGIC 0x8x- 0xBx---------------------------------------------------------------------------------------
  
Add$c_xHLrToA macro atomic_it, Cycles
;; atomic_it 0(add) or 1(adc)
;; source value <- eax 
;; target <- always register A 
  and eax, 0FFh ;; Value &= 0xFF 
  and YG_PF, atomic_it
  shr YG_PF, 1 ;; check c-flags 
  movzx edx, [YG_GP].A 
  mov ecx, edx ;; temp WORD := A 
  adc ecx, eax ;; temp WORD := A + Value
  mov [YG_GP].A, cl  ;; always write back A.
  xor dx, ax  
  mov ax, cx  
  xor ax, dx  ;; temp WORD:= temp WORD^(A ^Value)
  and ax, 010h
  or YG_PF, eax   ;; SetH 
  or YG_PF_8, ch  ;; SetC 
  test cl, cl 
  setz cl 
  shl ecx, 6 
  or YG_PF, ecx ;; SetZ  XXX:ZTable 
endm 
      
CmpSub$bc_xHLrToA macro atomic_it, Cycles, Register ;; [YG_GP].A || cl for cmp opcode 
;; source <- eax 
;; target <- always register A  or nodone
  and eax, 0FFh ;; Value &= 0xFF 
  and YG_PF, atomic_it
  shr YG_PF, 1 ;; check c-flags 
  movzx edx, [YG_GP].A 
  mov ecx, edx ;; temp WORD := A 
  sbb ecx, eax ;; temp WORD := A - Value
  mov Register, cl  ;; always write back A.
  xor dx, ax  
  mov ax, cx  
  xor ax, dx  ;; temp WORD:= temp WORD^(A ^Value)
  and ax, 010h
  or YG_PF, eax   ;; SetH 
  mov eax, ecx 
  shr ax, 15
  or YG_PF_8, al  ;; SetC 
  test cl, cl 
  setz cl 
  shl ecx, 6 
  or YG_PF, ecx ;; SetZ  XXX:ZTable 
  or YG_PF, N_FLAG ;; SetN 
endm 
      
;; XOR | OR | AND do unwind base .
Logic_T macro   Cycles, initFlags, LogicOp
;; source <- eax 
;; target <- always register A 

;; clear psb . 
  mov YG_PF, initFlags
  movzx edx, [YG_GP].A 
  LogicOp eax, edx 
  ;; always write back A.
  mov [YG_GP].A, al 
  
  ;; SetZ  XXX:ZTable 
  test al, al 
  setz al 
  shl eax, 6 
  or YG_PF, eax 
endm  
     
;; unwind 
Add_ macro Cycles
  Add$c_xHLrToA 0, Cycles
endm 

Adc_ macro Cycles
  Add$c_xHLrToA 1, Cycles
endm 
  
Sub_ macro Cycles
  CmpSub$bc_xHLrToA 0, Cycles, [YG_GP].A
endm   
      
Sbc_ macro Cycles
  CmpSub$bc_xHLrToA 1, Cycles, [YG_GP].A
endm       
 
Cmp_ macro Cycles
  CmpSub$bc_xHLrToA 0, Cycles, cl
endm  
      
And_ macro Cycles
  Logic_T Cycles, H_FLAG, and
endm   
      
Xor_ macro Cycles
  Logic_T Cycles, 0, xor
endm       
 
Or_ macro Cycles
  Logic_T Cycles, 0, or
endm      
 
AddWord_  macro 
;; source <- eax 
;; target <- always register HL 

;; clear psb . save old Z 
  and YG_PF, Z_FLAG 
  movzx ecx, [YG_GP].HL 
  and eax, 0FFFFh 
  lea edx, [eax+ecx] 
  ;; always write back HL.
  mov [YG_GP].HL, dx 
  ;; SetH
  xor cx, ax 
  mov ax, dx 
  xor cx, ax 
  and cx, 01000h
  shr cx, 8 
  or YG_PF, ecx 
  ;; SetC 
  and edx, 010000h
  shr edx, 16 
  or YG_PF, edx 
endm 

;; TO SP
AddWord2_  macro 
;; source <- eax 
;; target <- always register SP 

;; clear psb . save old Z 
  ;; xor YG_PF, YG_PF 
  movzx ecx, [YG_GP].SP_ 
  and eax, 0FFFFh 
  lea edx, [eax+ecx] 
  ;; always write back SP.
  mov [YG_GP].SP_, dx 
  ;; SetH
  xor cx, ax 
  mov ax, dx 
  xor cx, ax 
  and cx, 01000h
  shr cx, 8 
  ;;or YG_PF, ecx 
  ;; SetC 
  and dx, 010000h
  shr dx, 16 
  ;;or YG_PF, edx 
endm 


DecWord_  macro 
  dec eax 
endm 

IncWord_  macro 
  inc eax 
endm 
 
Inc_  macro  ;; -----------------------
;; source <- eax 
;; clear psb . save old Z 
  and YG_PF, C_FLAG 
  and eax, 0FFh 
  lea edx, [eax+1] 
  mov ecx, edx 
  
  ;; SetH
  xor cx, ax 
  and cx, 010h 
  or YG_PF, ecx 
 
  mov eax, edx 
  test dl, dl 
  setz dl  
  shl dl, 6 
  or YG_PF, edx 
endm 

Dec_  macro  ;; -----------------------
;; source <- eax 
;; clear psb . save old Z 
  and YG_PF, C_FLAG 
  or YG_PF, N_FLAG 
  and eax, 0FFh 
  lea edx, [eax-1] 
  mov ecx, edx 
  
  ;; SetH
  xor cx, ax 
  and cx, 010h 
  or YG_PF, ecx 
 
  mov eax, edx 
  test dl, dl 
  setz dl 
  shl dl, 6 
  or YG_PF, edx 
endm 

;; --- Include OP, imm8 and ADD Word Register. 
Opcode@MainALU  macro  Opcode, Cycles, ReadOrExt, Op, WriteOrExt
  Opcode&:
    ReadOrExt
    Op
    WriteOrExt 
    SetCyclesRetP Cycles
endm 
      Opcode@MainALU  OP80, 4, B_Read, Add_, EmptyMacro ;; ADD A, B  1 Cycles:4
      Opcode@MainALU  OP81, 4, C_Read, Add_, EmptyMacro ;; ADD A, C  1 Cycles:4     
      Opcode@MainALU  OP82, 4, D_Read, Add_, EmptyMacro ;; ADD A, D  1 Cycles:4
      Opcode@MainALU  OP83, 4, E_Read, Add_, EmptyMacro ;; ADD A, E  1 Cycles:4      
      Opcode@MainALU  OP84, 4, H_Read, Add_, EmptyMacro ;; ADD A, H  1 Cycles:4
      Opcode@MainALU  OP85, 4, L_Read, Add_, EmptyMacro ;; ADD A, L  1 Cycles:4     
      Opcode@MainALU  OP86, 8, xHL_Read, Add_, EmptyMacro ;; ADD A, xHL  1 Cycles:8
      Opcode@MainALU  OP87, 4, A_Read, Add_, EmptyMacro ;; ADD A, A  1 Cycles:4  
      Opcode@MainALU  OPC6, 8, Imm8_Read, Add_, EmptyMacro ;; ADD A, Imm8  2 Cycles:8
      Opcode@MainALU  OP09, 8, BC_Read, AddWord_, HL_Write ;; ADD HL, BC  1 Cycles:8     
      Opcode@MainALU  OP19, 8, DE_Read, AddWord_, HL_Write ;; ADD HL, DE  1 Cycles:8     
      Opcode@MainALU  OP29, 8, HL_Read, AddWord_, HL_Write ;; ADD HL, HL  1 Cycles:8     
      Opcode@MainALU  OP39, 8, SP_Read, AddWord_, HL_Write ;; ADD HL, SP  1 Cycles:8     
      Opcode@MainALU  OPE8,16, Imm8_ExpandSignWord, AddWord2_, EmptyMacro ;; ADD SP, SignImm8  2 Cycles:16
      
      Opcode@MainALU  OP88, 4, B_Read, Adc_, EmptyMacro ;; ADC A, B  1 Cycles:4
      Opcode@MainALU  OP89, 4, C_Read, Adc_, EmptyMacro ;; ADC A, C  1 Cycles:4     
      Opcode@MainALU  OP8A, 4, D_Read, Adc_, EmptyMacro ;; ADC A, D  1 Cycles:4
      Opcode@MainALU  OP8B, 4, E_Read, Adc_, EmptyMacro ;; ADC A, E  1 Cycles:4      
      Opcode@MainALU  OP8C, 4, H_Read, Adc_, EmptyMacro ;; ADC A, H  1 Cycles:4
      Opcode@MainALU  OP8D, 4, L_Read, Adc_, EmptyMacro ;; ADC A, L  1 Cycles:4     
      Opcode@MainALU  OP8E, 8, xHL_Read, Adc_, EmptyMacro ;; ADC A, xHL  1 Cycles:8
      Opcode@MainALU  OP8F, 4, A_Read, Adc_, EmptyMacro ;; ADC A, A  1 Cycles:4  
      Opcode@MainALU  OPCE, 8, Imm8_Read, Adc_, EmptyMacro ;; ADC A, Imm8  2 Cycles:8     
      
      Opcode@MainALU  OP03, 8, BC_Read, IncWord_, BC_Write ;; INC BC  Cycles:8
      Opcode@MainALU  OP13, 8, DE_Read, IncWord_, DE_Write ;; INC DE  Cycles:8
      Opcode@MainALU  OP23, 8, HL_Read, IncWord_, HL_Write ;; INC HL  Cycles:8
      Opcode@MainALU  OP33, 8, SP_Read, IncWord_, SP_Write ;; INC SP  Cycles:8
      
      Opcode@MainALU  OP04, 4, B_Read, Inc_, B_Write ;; INC B  1 Cycles:4
      Opcode@MainALU  OP14, 4, D_Read, Inc_, D_Write ;; INC D  1 Cycles:4     
      Opcode@MainALU  OP24, 4, H_Read, Inc_, H_Write ;; INC H  1 Cycles:4
      Opcode@MainALU  OP34, 4, xHL_Read, Inc_, xHL_Write ;; INC xHL  1 Cycles:4      
      Opcode@MainALU  OP0C, 4, C_Read, Inc_, C_Write ;; INC C  1 Cycles:4
      Opcode@MainALU  OP1C, 4, E_Read, Inc_, E_Write ;; INC E  1 Cycles:4     
      Opcode@MainALU  OP2C,12, L_Read, Inc_, L_Write ;; INC L  1 Cycles:12
      Opcode@MainALU  OP3C, 4, A_Read, Inc_, A_Write ;; INC A  1 Cycles:4       
      
      Opcode@MainALU  OP90, 4, B_Read, Sub_, EmptyMacro ;; SUB A, B  1 Cycles:4
      Opcode@MainALU  OP91, 4, C_Read, Sub_, EmptyMacro ;; SUB A, C  1 Cycles:4     
      Opcode@MainALU  OP92, 4, D_Read, Sub_, EmptyMacro ;; SUB A, D  1 Cycles:4
      Opcode@MainALU  OP93, 4, E_Read, Sub_, EmptyMacro ;; SUB A, E  1 Cycles:4      
      Opcode@MainALU  OP94, 4, H_Read, Sub_, EmptyMacro ;; SUB A, H  1 Cycles:4
      Opcode@MainALU  OP95, 4, L_Read, Sub_, EmptyMacro ;; SUB A, L  1 Cycles:4     
      Opcode@MainALU  OP96, 8, xHL_Read, Sub_, EmptyMacro ;; SUB A, xHL  1 Cycles:8
      Opcode@MainALU  OP97, 4, A_Read, Sub_, EmptyMacro ;; SUB A, A  1 Cycles:4  
      Opcode@MainALU  OPD6, 8, Imm8_Read, Sub_, EmptyMacro ;; SUB A, Imm8  2 Cycles:8
      
      Opcode@MainALU  OP98, 4, B_Read, Sbc_, EmptyMacro ;; SBC A, B  1 Cycles:4
      Opcode@MainALU  OP99, 4, C_Read, Sbc_, EmptyMacro ;; SBC A, C  1 Cycles:4     
      Opcode@MainALU  OP9A, 4, D_Read, Sbc_, EmptyMacro ;; SBC A, D  1 Cycles:4
      Opcode@MainALU  OP9B, 4, E_Read, Sbc_, EmptyMacro ;; SBC A, E  1 Cycles:4      
      Opcode@MainALU  OP9C, 4, H_Read, Sbc_, EmptyMacro ;; SBC A, H  1 Cycles:4
      Opcode@MainALU  OP9D, 4, L_Read, Sbc_, EmptyMacro ;; SBC A, L  1 Cycles:4     
      Opcode@MainALU  OP9E, 8, xHL_Read, Sbc_, EmptyMacro ;; SBC A, xHL  1 Cycles:8
      Opcode@MainALU  OP9F, 4, A_Read, Sbc_, EmptyMacro ;; SBC A, A  1 Cycles:4  
      Opcode@MainALU  OPDE, 8, Imm8_Read, Sbc_, EmptyMacro ;; SBC A, Imm8  2 Cycles:8
   
      Opcode@MainALU  OP0B, 8, BC_Read, DecWord_, BC_Write ;; DEC BC  Cycles:8
      Opcode@MainALU  OP1B, 8, DE_Read, DecWord_, DE_Write ;; DEC DE  Cycles:8
      Opcode@MainALU  OP2B, 8, HL_Read, DecWord_, HL_Write ;; DEC HL  Cycles:8
      Opcode@MainALU  OP3B, 8, SP_Read, DecWord_, SP_Write ;; DEC SP  Cycles:8
      
      Opcode@MainALU  OP05, 4, B_Read, Dec_, B_Write ;; DEC B  1 Cycles:4
      Opcode@MainALU  OP15, 4, D_Read, Dec_, D_Write ;; DEC D  1 Cycles:4     
      Opcode@MainALU  OP25, 4, H_Read, Dec_, H_Write ;; DEC H  1 Cycles:4
      Opcode@MainALU  OP35, 4, xHL_Read, Dec_, xHL_Write ;; DEC xHL  1 Cycles:4      
      Opcode@MainALU  OP0D, 4, C_Read, Dec_, C_Write ;; DEC C  1 Cycles:4
      Opcode@MainALU  OP1D, 4, E_Read, Dec_, E_Write ;; DEC E  1 Cycles:4     
      Opcode@MainALU  OP2D,12, L_Read, Dec_, L_Write ;; DEC L  1 Cycles:12
      Opcode@MainALU  OP3D, 4, A_Read, Dec_, A_Write ;; DEC A  1 Cycles:4  
      
      Opcode@MainALU  OPA0, 4, B_Read, And_, EmptyMacro ;; AND A, B  1 Cycles:4
      Opcode@MainALU  OPA1, 4, C_Read, And_, EmptyMacro ;; AND A, C  1 Cycles:4     
      Opcode@MainALU  OPA2, 4, D_Read, And_, EmptyMacro ;; AND A, D  1 Cycles:4
      Opcode@MainALU  OPA3, 4, E_Read, And_, EmptyMacro ;; AND A, E  1 Cycles:4      
      Opcode@MainALU  OPA4, 4, H_Read, And_, EmptyMacro ;; AND A, H  1 Cycles:4
      Opcode@MainALU  OPA5, 4, L_Read, And_, EmptyMacro ;; AND A, L  1 Cycles:4     
      Opcode@MainALU  OPA6, 8, xHL_Read, And_, EmptyMacro ;; AND A, xHL  1 Cycles:8
      Opcode@MainALU  OPA7, 4, A_Read, And_, EmptyMacro ;; AND A, A  1 Cycles:4  
      Opcode@MainALU  OPE6, 8, Imm8_Read, And_, EmptyMacro ;; AND A, Imm8  2 Cycles:8
       
      Opcode@MainALU  OPA8, 4, B_Read, Xor_, EmptyMacro ;; XOR A, B  1 Cycles:4
      Opcode@MainALU  OPA9, 4, C_Read, Xor_, EmptyMacro ;; XOR A, C  1 Cycles:4     
      Opcode@MainALU  OPAA, 4, D_Read, Xor_, EmptyMacro ;; XOR A, D  1 Cycles:4
      Opcode@MainALU  OPAB, 4, E_Read, Xor_, EmptyMacro ;; XOR A, E  1 Cycles:4      
      Opcode@MainALU  OPAC, 4, H_Read, Xor_, EmptyMacro ;; XOR A, H  1 Cycles:4
      Opcode@MainALU  OPAD, 4, L_Read, Xor_, EmptyMacro ;; XOR A, L  1 Cycles:4     
      Opcode@MainALU  OPAE, 8, xHL_Read, Xor_, EmptyMacro ;; XOR A, xHL  1 Cycles:8
      Opcode@MainALU  OPAF, 4, A_Read, Xor_, EmptyMacro ;; XOR A, A  1 Cycles:4  
      Opcode@MainALU  OPEE, 8, Imm8_Read, Xor_, EmptyMacro ;; XOR A, Imm8  2 Cycles:8
      
      Opcode@MainALU  OPB0, 4, B_Read, Or_, EmptyMacro ;; OR A, B  1 Cycles:4
      Opcode@MainALU  OPB1, 4, C_Read, Or_, EmptyMacro ;; OR A, C  1 Cycles:4     
      Opcode@MainALU  OPB2, 4, D_Read, Or_, EmptyMacro ;; OR A, D  1 Cycles:4
      Opcode@MainALU  OPB3, 4, E_Read, Or_, EmptyMacro ;; OR A, E  1 Cycles:4      
      Opcode@MainALU  OPB4, 4, H_Read, Or_, EmptyMacro ;; OR A, H  1 Cycles:4
      Opcode@MainALU  OPB5, 4, L_Read, Or_, EmptyMacro ;; OR A, L  1 Cycles:4     
      Opcode@MainALU  OPB6, 8, xHL_Read, Or_, EmptyMacro ;; OR A, xHL  1 Cycles:8
      Opcode@MainALU  OPB7, 4, A_Read, Or_, EmptyMacro ;; OR A, A  1 Cycles:4  
      Opcode@MainALU  OPF6, 8, Imm8_Read, Or_, EmptyMacro ;; OR A, Imm8  2 Cycles:8
      
      Opcode@MainALU  OPB8, 4, B_Read, Cmp_, EmptyMacro ;; CP A, B  1 Cycles:4
      Opcode@MainALU  OPB9, 4, C_Read, Cmp_, EmptyMacro ;; CP A, C  1 Cycles:4     
      Opcode@MainALU  OPBA, 4, D_Read, Cmp_, EmptyMacro ;; CP A, D  1 Cycles:4
      Opcode@MainALU  OPBB, 4, E_Read, Cmp_, EmptyMacro ;; CP A, E  1 Cycles:4      
      Opcode@MainALU  OPBC, 4, H_Read, Cmp_, EmptyMacro ;; CP A, H  1 Cycles:4
      Opcode@MainALU  OPBD, 4, L_Read, Cmp_, EmptyMacro ;; CP A, L  1 Cycles:4     
      Opcode@MainALU  OPBE, 8, xHL_Read, Cmp_, EmptyMacro ;; CP A, xHL  1 Cycles:8
      Opcode@MainALU  OPBF, 4, A_Read, Cmp_, EmptyMacro ;; CP A, A  1 Cycles:4  
      Opcode@MainALU  OPFE, 8, Imm8_Read, Cmp_, EmptyMacro ;; CP A, Imm8  2 Cycles:8
      
PushWord_  macro  ;; -----------------------
;; source <- eax 
  mov cx, [YG_GP].SP_ 
  sub cx, 2 
  mov [YG_GP].SP_, cx 
  push eax 
  push ecx 
  push [YG_GP].gameboy
  call gameboy_mmu_write_w@12
endm 

PopWord_  macro  ;; -----------------------
  mov cx, [YG_GP].SP_ 
  push ecx 
  add cx, 2 
  mov [YG_GP].SP_, cx 
  push [YG_GP].gameboy
  call gameboy_mmu_read_w@8
endm 

;; --- 
Opcode@MainStackOperate  macro  Opcode, Cycles, ReadOrExt, Op, WriteOrExt
  Opcode&:
    ReadOrExt
    Op
    WriteOrExt 
    SetCyclesAndRet Cycles
endm   
 
      Opcode@MainStackOperate  OPC1,12, PopWord_, EmptyMacro, BC_Write ;; POP BC  1 Cycles:12
      Opcode@MainStackOperate  OPD1,12, PopWord_, EmptyMacro, DE_Write ;; POP DE  1 Cycles:12     
      Opcode@MainStackOperate  OPE1,12, PopWord_, EmptyMacro, HL_Write ;; POP HL  1 Cycles:12
      Opcode@MainStackOperate  OPF1,12, PopWord_, EmptyMacro, AF_Write ;; POP AF  1 Cycles:12  
      
      Opcode@MainStackOperate  OPC5,16, BC_Read, EmptyMacro, PushWord_ ;; PUSH BC  1 Cycles:16
      Opcode@MainStackOperate  OPD5,16, DE_Read, EmptyMacro, PushWord_ ;; PUSH DE  1 Cycles:16     
      Opcode@MainStackOperate  OPE5,16, HL_Read, EmptyMacro, PushWord_ ;; PUSH HL  1 Cycles:16
      Opcode@MainStackOperate  OPF5,16, AF_Read, EmptyMacro, PushWord_ ;; PUSH AF  1 Cycles:16  

Opcode@Rst  macro  Opcode, Cycles, Vector
  Opcode&:
    mov ax, si
    PushWord_ 
    mov si, Vector
    SetCyclesRetP Cycles
endm  

      Opcode@Rst  OPC7,16, 000H ;; RST 00H  1 Cycles:16
      Opcode@Rst  OPD7,16, 010H ;; RST 10H  1 Cycles:16     
      Opcode@Rst  OPE7,16, 020H ;; RST 20H  1 Cycles:16
      Opcode@Rst  OPF7,16, 030H ;; RST 30H  1 Cycles:16  
      Opcode@Rst  OPCF,16, 008H ;; RST 08H  1 Cycles:16
      Opcode@Rst  OPDF,16, 018H ;; RST 18H  1 Cycles:16     
      Opcode@Rst  OPEF,16, 028H ;; RST 28H  1 Cycles:16
      Opcode@Rst  OPFF,16, 038H ;; RST 38H  1 Cycles:16     
      
Opcode@JR    macro  Opcode, Flags, OpNOT 
   Opcode&: 
      mov eax, YG_PF
      and eax, Flags
      xor eax, OpNOT 
      jne @F 
      inc esi
      SetCyclesRetP 8
    @@:
      Imm8_Read 
      movsx eax, al 
      add esi, eax 
      SetCyclesRetP 12
endm
      Opcode@JR  OP20,Z_FLAG, Z_FLAG ;; JR NZ 
      Opcode@JR  OP30,C_FLAG, C_FLAG ;; JR NC 
      Opcode@JR  OP28,Z_FLAG, 0 ;; JR Z 
      Opcode@JR  OP38,C_FLAG, 0 ;; JR C   
      Opcode@JR  OP18,0, 1 ;; JR R8
      
Opcode@JP    macro  Opcode, Flags, OpNOT 
   Opcode&: 
      mov eax, YG_PF
      and eax, Flags
      xor eax, OpNOT 
      jne @F 
      add esi, 2
      SetCyclesRetP 12
    @@:
      Imm16_Read 
      mov esi, eax 
      SetCyclesRetP 16
endm      
      Opcode@JP  OPC2,Z_FLAG, Z_FLAG ;; JP NZ 
      Opcode@JP  OPD2,C_FLAG, C_FLAG ;; JP NC 
      Opcode@JP  OPCA,Z_FLAG, 0 ;; JP Z 
      Opcode@JP  OPDA,C_FLAG, 0 ;; JP C   
      Opcode@JP  OPC3,0, 1 ;; JP A16
    OPE9:
      mov si, [YG_GP].HL 
      mov [YG_GP].PC, ax 
      SetCyclesAndRet 4
      ;; LD@RxHLToRxHL OPE9, 4, HL_Read, PC_Write ;; JP (HL), Same as LD PC HL, 1 Cycles:4  
  
Opcode@CALL    macro  Opcode, Flags, OpNOT 
   Opcode&: 
      mov eax, YG_PF
      and eax, Flags
      xor eax, OpNOT 
      jne @F 
      add esi, 2
      SetCyclesRetP 12
    @@:
      lea eax, [YG_PC+2]
      PushWord_ 
      Imm16_Read 
      mov esi, eax 
      SetCyclesRetP 24
endm 
      Opcode@CALL  OPC4,Z_FLAG, Z_FLAG ;; CALL NZ 
      Opcode@CALL  OPD4,C_FLAG, C_FLAG ;; CALL NC 
      Opcode@CALL  OPCC,Z_FLAG, 0 ;; CALL Z 
      Opcode@CALL  OPDC,C_FLAG, 0 ;; CALL C   
      Opcode@CALL  OPCD,0, 1 ;; CALL  
      
Opcode@RET    macro  Opcode, Flags, OpNOT, RetHitCycles
   Opcode&: 
      mov eax, YG_PF
      and eax, Flags
      xor eax, OpNOT 
      jne @F 
      SetCyclesRetP 8
    @@:
      PopWord_ 
      mov YG_PC, eax 
      SetCyclesRetP RetHitCycles
endm      
 
Opcode@RETI    macro  Opcode, Flags, OpNOT, RetHitCycles
   Opcode&: 
      mov eax, YG_PF
      and eax, Flags
      xor eax, OpNOT 
      jne @F 
      SetCyclesRetP 8
    @@:
      PopWord_ 
      mov YG_PC, eax 
      mov [YG_GP].IME, 1
      SetCyclesRetP RetHitCycles
endm 

      Opcode@RET  OPC0,Z_FLAG, Z_FLAG, 20 ;; RET NZ 
      Opcode@RET  OPD0,C_FLAG, C_FLAG, 20 ;; RET NC 
      Opcode@RET  OPC8,Z_FLAG, 0, 20 ;; RET Z 
      Opcode@RET  OPD8,C_FLAG, 0, 20 ;; RET C   
      Opcode@RET  OPC9,0, 1, 16 ;; RET  
      Opcode@RETI OPD9,0, 1, 16 ;; RETI 
      
;;  MISC 8. 
      OP76:   ; Halt,  not backup PC in my source code ^_^
        mov [YG_GP].halt, 1
        SetCyclesAndRet 4    
      OP10:   ; Stop, Check CGB speed mode 
        movzx eax, [YG_GP].key1
        test eax, 1 
        je @F
        xor eax, 080H ;; switch to "other" speed 
        and eax, 0FEH ;; reset LSB  see gb-programming-manual.pdf::2.6.2 CPU Operating Speed
                      ;; for simplicity, I will not simulate the huge waste of time brought by handover.
        mov [YG_GP].key1, al 
        SetCyclesAndRet 080000004H
     @@:mov [YG_GP].stop, 1
        add YG_PC, 1 ;; skip one byte (should is 00)
      OP00:   ; NOP 
        SetCyclesAndRet 4           
      OPF3:   ; DI 
        mov [YG_GP].IME, 0 
        SetCyclesAndRet 4 
      OPFB:   ; EI 
        mov [YG_GP].IME, 1
        SetCyclesAndRet 4   
      OP07:   ; RLCA 
        rol [YG_GP].A, 1 
        setc YG_PF_8
        SetCyclesRetP 4 
      OP17:   ; RLA 
        shr YG_PF_8, 1
        rcl [YG_GP].A, 1 
        setc YG_PF_8
        SetCyclesRetP 4    
      OP0F:   ; RRCA 
        ror [YG_GP].A, 1 
        setc YG_PF_8
        SetCyclesRetP 4 
      OP1F:   ; RRA 
        shr YG_PF_8, 1
        rcr [YG_GP].A, 1 
        setc YG_PF_8
        SetCyclesRetP 4     
      OP27:   ; DAA  
        SetCyclesRetP 4     
        mov al, [YG_GP].A
        and YG_PF_8, N_FLAG 
        jne DAS_Proc 
        ;;  DAA. 
        daa 
        mov [YG_GP].A, al
        setc YG_PF_8  ;; SETC 
        setz al 
        shl al, 6 
        or YG_PF_8, al
        SetCyclesRetP 4         
    DAS_Proc:
        ;;  DAS 
        das     
        mov [YG_GP].A, al
        setc YG_PF_8  ;; SETC 
        setz al 
        shl al, 6 
        or YG_PF_8, al
        or YG_PF_8, N_FLAG
        SetCyclesRetP 4    
      OP37:   ; SCF 
        and YG_PF_8, Z_FLAG
        or YG_PF_8, C_FLAG  
        SetCyclesRetP 4    
      OP2F:   ; CPL  
        not [YG_GP].A 
        or YG_PF_8, N_FLAG
        or YG_PF_8, H_FLAG
        SetCyclesRetP 4   
      OP3F:   ; CCF 
        and YG_PF_8, ZC_FLAG
        xor YG_PF_8, C_FLAG  
        SetCyclesRetP 4 
         
      ;; rortoe shift with   
      RLC_ macro Cycles
        rol al, 1 
        setc YG_PF_8
        test al, al 
        setz dl 
        shl dl, 6 
        or YG_PF_8, dl     
      endm 
      ;; rortoe shift with   
      RRC_ macro Cycles
        ror al, 1 
        setc YG_PF_8
        test al, al 
        setz dl 
        shl dl, 6 
        or YG_PF_8, dl     
      endm       
      ;; logic shift with carry
      RL_ macro Cycles
        shr YG_PF_8, 1
        rcl al, 1 
        setc YG_PF_8
        test al, al 
        setz dl 
        shl dl, 6 
        or YG_PF_8, dl     
      endm 
      ;; logic shift with carry
      RR_ macro Cycles
        shr YG_PF_8, 1
        rcr al, 1 
        setc YG_PF_8
        test al, al 
        setz dl 
        shl dl, 6 
        or YG_PF_8, dl     
      endm       
      ;; logic shift    
      RL_N_ macro Cycles
        shl al, 1 
        setc YG_PF_8
        test al, al 
        setz dl 
        shl dl, 6 
        or YG_PF_8, dl     
      endm 
      ;; logic shift  
      RR_N_ macro Cycles
        shr al, 1 
        setc YG_PF_8
        test al, al 
        setz dl 
        shl dl, 6 
        or YG_PF_8, dl     
      endm    
      ;; arith shift  save msb 
      RRS_N_ macro Cycles
        sar al, 1 
        setc YG_PF_8
        setz dl 
        shl dl, 6 
        or YG_PF_8, dl     
      endm     
      ;; swap byte-lo 4bit and byte-hi 4bit
      SWAP_ macro Cycles
        ror al, 4 
        test al, al 
        setz cl 
        shl cl, 6 
        mov YG_PF, ecx 
      endm         
        
      ;;  Set 
      Opcode@SetBit macro Opcode, Cycles,  ReadOrExt, BitOrder, WriteOrExt
        Opcode&:
          ReadOrExt
          mov ecx, 1 
          shl ecx, BitOrder
          or eax, ecx 
          WriteOrExt 
          SetCyclesAndRet Cycles
      endm 
      
      Opcode@ResBit macro Opcode, Cycles,  ReadOrExt, BitOrder, WriteOrExt
        Opcode&:
          ReadOrExt
          mov ecx, 1 
          shl ecx, BitOrder
          not ecx 
          and eax, ecx 
          WriteOrExt 
          SetCyclesAndRet Cycles
      endm         
          
      Opcode@TestBit macro Opcode, Cycles,  ReadOrExt, BitOrder
        Opcode&:
          ReadOrExt
          and YG_PF, C_FLAG
          or YG_PF, H_FLAG 
          mov ecx, 1 
          shl ecx, BitOrder
          test al, cl 
          setz al 
          shl eax, 6 
          or YG_PF, eax 
          SetCyclesRetP Cycles
      endm       
        
      OPCB: ;; ---------------------------------- Perfix CB -----------------------------------------------------------------------------
        Imm8_Read 
        and eax, 255 
        jmp dword ptr[CBTAB+eax*4]
        
        ;; --- Include OP, imm8 and ADD Word Register. 
        Opcode@MainALUExt  macro  Opcode, Cycles, ReadOrExt, Op, WriteOrExt
          Opcode&:
            ReadOrExt
            Op
            WriteOrExt 
            SetCyclesRetP Cycles
        endm  
        
        Opcode@MainALUExt  CB00, 8, B_Read, RLC_, B_Write      ;; RLC B 2 Cycles:8
        Opcode@MainALUExt  CB01, 8, C_Read, RLC_, C_Write      ;; RLC C 2 Cycles:8
        Opcode@MainALUExt  CB02, 8, D_Read, RLC_, D_Write      ;; RLC D 2 Cycles:8
        Opcode@MainALUExt  CB03, 8, E_Read, RLC_, E_Write      ;; RLC E 2 Cycles:8     
        Opcode@MainALUExt  CB04, 8, H_Read, RLC_, H_Write      ;; RLC H 2 Cycles:8
        Opcode@MainALUExt  CB05, 8, L_Read, RLC_, L_Write      ;; RLC L 2 Cycles:8        
        Opcode@MainALUExt  CB06,16, xHL_Read, RLC_, xHL_Write      ;; RLC xHL 2 Cycles:16
        Opcode@MainALUExt  CB07, 8, A_Read, RLC_, A_Write      ;; RLC A 2 Cycles:8        
        
        Opcode@MainALUExt  CB08, 8, B_Read, RRC_, B_Write      ;; RRC B 2 Cycles:8
        Opcode@MainALUExt  CB09, 8, C_Read, RRC_, C_Write      ;; RRC C 2 Cycles:8
        Opcode@MainALUExt  CB0A, 8, D_Read, RRC_, D_Write      ;; RRC D 2 Cycles:8
        Opcode@MainALUExt  CB0B, 8, E_Read, RRC_, E_Write      ;; RRC E 2 Cycles:8     
        Opcode@MainALUExt  CB0C, 8, H_Read, RRC_, H_Write      ;; RRC H 2 Cycles:8
        Opcode@MainALUExt  CB0D, 8, L_Read, RRC_, L_Write      ;; RRC L 2 Cycles:8        
        Opcode@MainALUExt  CB0E,16, xHL_Read, RRC_, xHL_Write      ;; RRC xHL 2 Cycles:16
        Opcode@MainALUExt  CB0F, 8, A_Read, RRC_, A_Write      ;; RRC A 2 Cycles:8     
        
        Opcode@MainALUExt  CB10, 8, B_Read, RL_, B_Write      ;; RL B 2 Cycles:8
        Opcode@MainALUExt  CB11, 8, C_Read, RL_, C_Write      ;; RL C 2 Cycles:8
        Opcode@MainALUExt  CB12, 8, D_Read, RL_, D_Write      ;; RL D 2 Cycles:8
        Opcode@MainALUExt  CB13, 8, E_Read, RL_, E_Write      ;; RL E 2 Cycles:8     
        Opcode@MainALUExt  CB14, 8, H_Read, RL_, H_Write      ;; RL H 2 Cycles:8
        Opcode@MainALUExt  CB15, 8, L_Read, RL_, L_Write      ;; RL L 2 Cycles:8        
        Opcode@MainALUExt  CB16,16, xHL_Read, RL_, xHL_Write      ;; RL xHL 2 Cycles:16
        Opcode@MainALUExt  CB17, 8, A_Read, RL_, A_Write      ;; RL A 2 Cycles:8        
        
        Opcode@MainALUExt  CB18, 8, B_Read, RR_, B_Write      ;; RR B 2 Cycles:8
        Opcode@MainALUExt  CB19, 8, C_Read, RR_, C_Write      ;; RR C 2 Cycles:8
        Opcode@MainALUExt  CB1A, 8, D_Read, RR_, D_Write      ;; RR D 2 Cycles:8
        Opcode@MainALUExt  CB1B, 8, E_Read, RR_, E_Write      ;; RR E 2 Cycles:8     
        Opcode@MainALUExt  CB1C, 8, H_Read, RR_, H_Write      ;; RR H 2 Cycles:8
        Opcode@MainALUExt  CB1D, 8, L_Read, RR_, L_Write      ;; RR L 2 Cycles:8        
        Opcode@MainALUExt  CB1E,16, xHL_Read, RR_, xHL_Write      ;; RR xHL 2 Cycles:16
        Opcode@MainALUExt  CB1F, 8, A_Read, RR_, A_Write      ;; RR A 2 Cycles:8            
        
        Opcode@MainALUExt  CB20, 8, B_Read, RL_N_, B_Write      ;; SLA B 2 Cycles:8
        Opcode@MainALUExt  CB21, 8, C_Read, RL_N_, C_Write      ;; SLA C 2 Cycles:8
        Opcode@MainALUExt  CB22, 8, D_Read, RL_N_, D_Write      ;; SLA D 2 Cycles:8
        Opcode@MainALUExt  CB23, 8, E_Read, RL_N_, E_Write      ;; SLA E 2 Cycles:8     
        Opcode@MainALUExt  CB24, 8, H_Read, RL_N_, H_Write      ;; SLA H 2 Cycles:8
        Opcode@MainALUExt  CB25, 8, L_Read, RL_N_, L_Write      ;; SLA L 2 Cycles:8        
        Opcode@MainALUExt  CB26,16, xHL_Read, RL_N_, xHL_Write      ;; SLA xHL 2 Cycles:16
        Opcode@MainALUExt  CB27, 8, A_Read, RL_N_, A_Write      ;; SLA A 2 Cycles:8        
        
        Opcode@MainALUExt  CB28, 8, B_Read, RRS_N_, B_Write      ;; SRA B 2 Cycles:8
        Opcode@MainALUExt  CB29, 8, C_Read, RRS_N_, C_Write      ;; SRA C 2 Cycles:8
        Opcode@MainALUExt  CB2A, 8, D_Read, RRS_N_, D_Write      ;; SRA D 2 Cycles:8
        Opcode@MainALUExt  CB2B, 8, E_Read, RRS_N_, E_Write      ;; SRA E 2 Cycles:8     
        Opcode@MainALUExt  CB2C, 8, H_Read, RRS_N_, H_Write      ;; SRA H 2 Cycles:8
        Opcode@MainALUExt  CB2D, 8, L_Read, RRS_N_, L_Write      ;; SRA L 2 Cycles:8        
        Opcode@MainALUExt  CB2E,16, xHL_Read, RRS_N_, xHL_Write      ;; SRA xHL 2 Cycles:16
        Opcode@MainALUExt  CB2F, 8, A_Read, RRS_N_, A_Write      ;; SRA A 2 Cycles:8      
        
        Opcode@MainALUExt  CB30, 8, B_Read, SWAP_, B_Write      ;; SWAP B 2 Cycles:8
        Opcode@MainALUExt  CB31, 8, C_Read, SWAP_, C_Write      ;; SWAP C 2 Cycles:8
        Opcode@MainALUExt  CB32, 8, D_Read, SWAP_, D_Write      ;; SWAP D 2 Cycles:8
        Opcode@MainALUExt  CB33, 8, E_Read, SWAP_, E_Write      ;; SWAP E 2 Cycles:8     
        Opcode@MainALUExt  CB34, 8, H_Read, SWAP_, H_Write      ;; SWAP H 2 Cycles:8
        Opcode@MainALUExt  CB35, 8, L_Read, SWAP_, L_Write      ;; SWAP L 2 Cycles:8        
        Opcode@MainALUExt  CB36,16, xHL_Read, SWAP_, xHL_Write      ;; SWAP xHL 2 Cycles:16
        Opcode@MainALUExt  CB37, 8, A_Read, SWAP_, A_Write      ;; SWAP A 2 Cycles:8        
        
        Opcode@MainALUExt  CB38, 8, B_Read, RR_N_, B_Write      ;; SRL B 2 Cycles:8
        Opcode@MainALUExt  CB39, 8, C_Read, RR_N_, C_Write      ;; SRL C 2 Cycles:8
        Opcode@MainALUExt  CB3A, 8, D_Read, RR_N_, D_Write      ;; SRL D 2 Cycles:8
        Opcode@MainALUExt  CB3B, 8, E_Read, RR_N_, E_Write      ;; SRL E 2 Cycles:8     
        Opcode@MainALUExt  CB3C, 8, H_Read, RR_N_, H_Write      ;; SRL H 2 Cycles:8
        Opcode@MainALUExt  CB3D, 8, L_Read, RR_N_, L_Write      ;; SRL L 2 Cycles:8        
        Opcode@MainALUExt  CB3E,16, xHL_Read, RR_N_, xHL_Write      ;; SRL xHL 2 Cycles:16
        Opcode@MainALUExt  CB3F, 8, A_Read, RR_N_, A_Write      ;; SRL A 2 Cycles:8        
        
        Opcode@TestBit CB40, 8,  B_Read, 0 ;; BIT B, 0 Cycles:8 
        Opcode@TestBit CB41, 8,  C_Read, 0 ;; BIT C, 0 Cycles:8        
        Opcode@TestBit CB42, 8,  D_Read, 0 ;; BIT D, 0 Cycles:8        
        Opcode@TestBit CB43, 8,  E_Read, 0 ;; BIT E, 0 Cycles:8     
        Opcode@TestBit CB44, 8,  H_Read, 0 ;; BIT H, 0 Cycles:8 
        Opcode@TestBit CB45, 8,  L_Read, 0 ;; BIT L, 0 Cycles:8        
        Opcode@TestBit CB46,16,  xHL_Read, 0 ;; BIT xHL, 0 Cycles:16       
        Opcode@TestBit CB47, 8,  A_Read, 0 ;; BIT A, 0 Cycles:8         
      
        Opcode@TestBit CB48, 8,  B_Read, 1 ;; BIT B, 1 Cycles:8 
        Opcode@TestBit CB49, 8,  C_Read, 1 ;; BIT C, 1 Cycles:8        
        Opcode@TestBit CB4A, 8,  D_Read, 1 ;; BIT D, 1 Cycles:8        
        Opcode@TestBit CB4B, 8,  E_Read, 1 ;; BIT E, 1 Cycles:8     
        Opcode@TestBit CB4C, 8,  H_Read, 1 ;; BIT H, 1 Cycles:8 
        Opcode@TestBit CB4D, 8,  L_Read, 1 ;; BIT L, 1 Cycles:8        
        Opcode@TestBit CB4E,16,  xHL_Read, 1 ;; BIT xHL, 1 Cycles:16       
        Opcode@TestBit CB4F, 8,  A_Read, 1 ;; BIT A, 1 Cycles:8 

        Opcode@TestBit CB50, 8,  B_Read, 2 ;; BIT B, 2 Cycles:8 
        Opcode@TestBit CB51, 8,  C_Read, 2 ;; BIT C, 2 Cycles:8        
        Opcode@TestBit CB52, 8,  D_Read, 2 ;; BIT D, 2 Cycles:8        
        Opcode@TestBit CB53, 8,  E_Read, 2 ;; BIT E, 2 Cycles:8     
        Opcode@TestBit CB54, 8,  H_Read, 2 ;; BIT H, 2 Cycles:8 
        Opcode@TestBit CB55, 8,  L_Read, 2 ;; BIT L, 2 Cycles:8        
        Opcode@TestBit CB56,16,  xHL_Read, 2 ;; BIT xHL, 2 Cycles:16       
        Opcode@TestBit CB57, 8,  A_Read, 2 ;; BIT A, 2 Cycles:8         
      
        Opcode@TestBit CB58, 8,  B_Read, 3 ;; BIT B, 3 Cycles:8 
        Opcode@TestBit CB59, 8,  C_Read, 3 ;; BIT C, 3 Cycles:8        
        Opcode@TestBit CB5A, 8,  D_Read, 3 ;; BIT D, 3 Cycles:8        
        Opcode@TestBit CB5B, 8,  E_Read, 3 ;; BIT E, 3 Cycles:8     
        Opcode@TestBit CB5C, 8,  H_Read, 3 ;; BIT H, 3 Cycles:8 
        Opcode@TestBit CB5D, 8,  L_Read, 3 ;; BIT L, 3 Cycles:8        
        Opcode@TestBit CB5E,16,  xHL_Read, 3 ;; BIT xHL, 3 Cycles:16       
        Opcode@TestBit CB5F, 8,  A_Read, 3 ;; BIT A, 3 Cycles:8 

        Opcode@TestBit CB60, 8,  B_Read, 4 ;; BIT B, 4 Cycles:8 
        Opcode@TestBit CB61, 8,  C_Read, 4 ;; BIT C, 4 Cycles:8        
        Opcode@TestBit CB62, 8,  D_Read, 4 ;; BIT D, 4 Cycles:8        
        Opcode@TestBit CB63, 8,  E_Read, 4 ;; BIT E, 4 Cycles:8     
        Opcode@TestBit CB64, 8,  H_Read, 4 ;; BIT H, 4 Cycles:8 
        Opcode@TestBit CB65, 8,  L_Read, 4 ;; BIT L, 4 Cycles:8        
        Opcode@TestBit CB66,16,  xHL_Read, 4 ;; BITxHLD, 4 Cycles:16       
        Opcode@TestBit CB67, 8,  A_Read, 4 ;; BIT A, 4 Cycles:8         
      
        Opcode@TestBit CB68, 8,  B_Read, 5 ;; BIT B, 5 Cycles:8 
        Opcode@TestBit CB69, 8,  C_Read, 5 ;; BIT C, 5 Cycles:8        
        Opcode@TestBit CB6A, 8,  D_Read, 5 ;; BIT D, 5 Cycles:8        
        Opcode@TestBit CB6B, 8,  E_Read, 5 ;; BIT E, 5 Cycles:8     
        Opcode@TestBit CB6C, 8,  H_Read, 5 ;; BIT H, 5 Cycles:8 
        Opcode@TestBit CB6D, 8,  L_Read, 5 ;; BIT L, 5 Cycles:8        
        Opcode@TestBit CB6E,16,  xHL_Read, 5 ;; BIT xHL, 5 Cycles:16       
        Opcode@TestBit CB6F, 8,  A_Read, 5 ;; BIT A, 5 Cycles:8 
        
        Opcode@TestBit CB70, 8,  B_Read, 6 ;; BIT B, 6 Cycles:8 
        Opcode@TestBit CB71, 8,  C_Read, 6 ;; BIT C, 6 Cycles:8        
        Opcode@TestBit CB72, 8,  D_Read, 6 ;; BIT D, 6 Cycles:8        
        Opcode@TestBit CB73, 8,  E_Read, 6 ;; BIT E, 6 Cycles:8     
        Opcode@TestBit CB74, 8,  H_Read, 6 ;; BIT H, 6 Cycles:8 
        Opcode@TestBit CB75, 8,  L_Read, 6 ;; BIT L, 6 Cycles:8        
        Opcode@TestBit CB76,16,  xHL_Read, 6 ;; BIT xHL, 6 Cycles:16       
        Opcode@TestBit CB77, 8,  A_Read, 6 ;; BIT A, 6 Cycles:8         
      
        Opcode@TestBit CB78, 8,  B_Read, 7 ;; BIT B, 7 Cycles:8 
        Opcode@TestBit CB79, 8,  C_Read, 7 ;; BIT C, 7 Cycles:8        
        Opcode@TestBit CB7A, 8,  D_Read, 7 ;; BIT D, 7 Cycles:8        
        Opcode@TestBit CB7B, 8,  E_Read, 7 ;; BIT E, 7 Cycles:8     
        Opcode@TestBit CB7C, 8,  H_Read, 7 ;; BIT H, 7 Cycles:8 
        Opcode@TestBit CB7D, 8,  L_Read, 7 ;; BIT L, 7 Cycles:8        
        Opcode@TestBit CB7E,16,  xHL_Read, 7 ;; BIT xHL, 7 Cycles:16       
        Opcode@TestBit CB7F, 8,  A_Read, 7 ;; BIT A, 7 Cycles:8 
        
        Opcode@ResBit CB80, 8,  B_Read, 0, B_Write ;; RES B, 0 Cycles:8 
        Opcode@ResBit CB81, 8,  C_Read, 0, C_Write ;; RES C, 0 Cycles:8        
        Opcode@ResBit CB82, 8,  D_Read, 0, D_Write ;; RES D, 0 Cycles:8        
        Opcode@ResBit CB83, 8,  E_Read, 0, E_Write ;; RES E, 0 Cycles:8     
        Opcode@ResBit CB84, 8,  H_Read, 0, H_Write ;; RES H, 0 Cycles:8 
        Opcode@ResBit CB85, 8,  L_Read, 0, L_Write ;; RES L, 0 Cycles:8        
        Opcode@ResBit CB86,16,  xHL_Read, 0, xHL_Write ;; RES xHL, 0 Cycles:16       
        Opcode@ResBit CB87, 8,  A_Read, 0, A_Write ;; RES A, 0 Cycles:8         
      
        Opcode@ResBit CB88, 8,  B_Read, 1, B_Write ;; RES B, 1 Cycles:8 
        Opcode@ResBit CB89, 8,  C_Read, 1, C_Write ;; RES C, 1 Cycles:8        
        Opcode@ResBit CB8A, 8,  D_Read, 1, D_Write ;; RES D, 1 Cycles:8        
        Opcode@ResBit CB8B, 8,  E_Read, 1, E_Write ;; RES E, 1 Cycles:8     
        Opcode@ResBit CB8C, 8,  H_Read, 1, H_Write ;; RES H, 1 Cycles:8 
        Opcode@ResBit CB8D, 8,  L_Read, 1, L_Write ;; RES L, 1 Cycles:8        
        Opcode@ResBit CB8E,16,  xHL_Read, 1, xHL_Write ;; RES xHL, 1 Cycles:16       
        Opcode@ResBit CB8F, 8,  A_Read, 1, A_Write ;; RES A, 1 Cycles:8 

        Opcode@ResBit CB90, 8,  B_Read, 2, B_Write ;; RES B, 2 Cycles:8 
        Opcode@ResBit CB91, 8,  C_Read, 2, C_Write ;; RES C, 2 Cycles:8        
        Opcode@ResBit CB92, 8,  D_Read, 2, D_Write ;; RES D, 2 Cycles:8        
        Opcode@ResBit CB93, 8,  E_Read, 2, E_Write ;; RES E, 2 Cycles:8     
        Opcode@ResBit CB94, 8,  H_Read, 2, H_Write ;; RES H, 2 Cycles:8 
        Opcode@ResBit CB95, 8,  L_Read, 2, L_Write ;; RES L, 2 Cycles:8        
        Opcode@ResBit CB96,16,  xHL_Read, 2, xHL_Write ;; RES xHL, 2 Cycles:16       
        Opcode@ResBit CB97, 8,  A_Read, 2, A_Write ;; RES A, 2 Cycles:8         
      
        Opcode@ResBit CB98, 8,  B_Read, 3, B_Write ;; RES B, 3 Cycles:8 
        Opcode@ResBit CB99, 8,  C_Read, 3, C_Write ;; RES C, 3 Cycles:8        
        Opcode@ResBit CB9A, 8,  D_Read, 3, D_Write ;; RES D, 3 Cycles:8        
        Opcode@ResBit CB9B, 8,  E_Read, 3, E_Write ;; RES E, 3 Cycles:8     
        Opcode@ResBit CB9C, 8,  H_Read, 3, H_Write ;; RES H, 3 Cycles:8 
        Opcode@ResBit CB9D, 8,  L_Read, 3, L_Write ;; RES L, 3 Cycles:8        
        Opcode@ResBit CB9E,16,  xHL_Read, 3, xHL_Write ;; RES xHL, 3 Cycles:16       
        Opcode@ResBit CB9F, 8,  A_Read, 3, A_Write ;; RES A, 3 Cycles:8 

        Opcode@ResBit CBA0, 8,  B_Read, 4, B_Write ;; RES B, 4 Cycles:8 
        Opcode@ResBit CBA1, 8,  C_Read, 4, C_Write ;; RES C, 4 Cycles:8        
        Opcode@ResBit CBA2, 8,  D_Read, 4, D_Write ;; RES D, 4 Cycles:8        
        Opcode@ResBit CBA3, 8,  E_Read, 4, E_Write ;; RES E, 4 Cycles:8     
        Opcode@ResBit CBA4, 8,  H_Read, 4, H_Write ;; RES H, 4 Cycles:8 
        Opcode@ResBit CBA5, 8,  L_Read, 4, L_Write ;; RES L, 4 Cycles:8        
        Opcode@ResBit CBA6,16,  xHL_Read, 4, xHL_Write ;; RES xHL, 4 Cycles:16       
        Opcode@ResBit CBA7, 8,  A_Read, 4, A_Write ;; RES A, 4 Cycles:8         
      
        Opcode@ResBit CBA8, 8,  B_Read, 5, B_Write ;; RES B, 5 Cycles:8 
        Opcode@ResBit CBA9, 8,  C_Read, 5, C_Write ;; RES C, 5 Cycles:8        
        Opcode@ResBit CBAA, 8,  D_Read, 5, D_Write ;; RES D, 5 Cycles:8        
        Opcode@ResBit CBAB, 8,  E_Read, 5, E_Write ;; RES E, 5 Cycles:8     
        Opcode@ResBit CBAC, 8,  H_Read, 5, H_Write ;; RES H, 5 Cycles:8 
        Opcode@ResBit CBAD, 8,  L_Read, 5, L_Write ;; RES L, 5 Cycles:8        
        Opcode@ResBit CBAE,16,  xHL_Read, 5, xHL_Write ;; RES xHL, 5 Cycles:16       
        Opcode@ResBit CBAF, 8,  A_Read, 5, A_Write ;; RES A, 5 Cycles:8 
        
        Opcode@ResBit CBB0, 8,  B_Read, 6, B_Write ;; RES B, 6 Cycles:8 
        Opcode@ResBit CBB1, 8,  C_Read, 6, C_Write ;; RES C, 6 Cycles:8        
        Opcode@ResBit CBB2, 8,  D_Read, 6, D_Write ;; RES D, 6 Cycles:8        
        Opcode@ResBit CBB3, 8,  E_Read, 6, E_Write ;; RES E, 6 Cycles:8     
        Opcode@ResBit CBB4, 8,  H_Read, 6, H_Write ;; RES H, 6 Cycles:8 
        Opcode@ResBit CBB5, 8,  L_Read, 6, L_Write ;; RES L, 6 Cycles:8        
        Opcode@ResBit CBB6,16,  xHL_Read, 6, xHL_Write ;; RES xHL, 6 Cycles:16       
        Opcode@ResBit CBB7, 8,  A_Read, 6, A_Write ;; RES A, 6 Cycles:8         
      
        Opcode@ResBit CBB8, 8,  B_Read, 7, B_Write ;; RES B, 7 Cycles:8 
        Opcode@ResBit CBB9, 8,  C_Read, 7, C_Write ;; RES C, 7 Cycles:8        
        Opcode@ResBit CBBA, 8,  D_Read, 7, D_Write ;; RES D, 7 Cycles:8        
        Opcode@ResBit CBBB, 8,  E_Read, 7, E_Write ;; RES E, 7 Cycles:8     
        Opcode@ResBit CBBC, 8,  H_Read, 7, H_Write ;; RES H, 7 Cycles:8 
        Opcode@ResBit CBBD, 8,  L_Read, 7, L_Write ;; RES L, 7 Cycles:8        
        Opcode@ResBit CBBE,16,  xHL_Read, 7, xHL_Write ;; RES xHL, 7 Cycles:16       
        Opcode@ResBit CBBF, 8,  A_Read, 7, A_Write ;; RES A, 7 Cycles:8 

        Opcode@SetBit CBC0, 8,  B_Read, 0, B_Write ;; SET B, 0 Cycles:8 
        Opcode@SetBit CBC1, 8,  C_Read, 0, C_Write ;; SET C, 0 Cycles:8        
        Opcode@SetBit CBC2, 8,  D_Read, 0, D_Write ;; SET D, 0 Cycles:8        
        Opcode@SetBit CBC3, 8,  E_Read, 0, E_Write ;; SET E, 0 Cycles:8     
        Opcode@SetBit CBC4, 8,  H_Read, 0, H_Write ;; SET H, 0 Cycles:8 
        Opcode@SetBit CBC5, 8,  L_Read, 0, L_Write ;; SET L, 0 Cycles:8        
        Opcode@SetBit CBC6,16,  xHL_Read, 0, xHL_Write ;; SET xHL, 0 Cycles:16       
        Opcode@SetBit CBC7, 8,  A_Read, 0, A_Write ;; SET A, 0 Cycles:8         
      
        Opcode@SetBit CBC8, 8,  B_Read, 1, B_Write ;; SET B, 1 Cycles:8 
        Opcode@SetBit CBC9, 8,  C_Read, 1, C_Write ;; SET C, 1 Cycles:8        
        Opcode@SetBit CBCA, 8,  D_Read, 1, D_Write ;; SET D, 1 Cycles:8        
        Opcode@SetBit CBCB, 8,  E_Read, 1, E_Write ;; SET E, 1 Cycles:8     
        Opcode@SetBit CBCC, 8,  H_Read, 1, H_Write ;; SET H, 1 Cycles:8 
        Opcode@SetBit CBCD, 8,  L_Read, 1, L_Write ;; SET L, 1 Cycles:8        
        Opcode@SetBit CBCE,16,  xHL_Read, 1, xHL_Write ;; SET xHL, 1 Cycles:16       
        Opcode@SetBit CBCF, 8,  A_Read, 1, A_Write ;; SET A, 1 Cycles:8 

        Opcode@SetBit CBD0, 8,  B_Read, 2, B_Write ;; SET B, 2 Cycles:8 
        Opcode@SetBit CBD1, 8,  C_Read, 2, C_Write ;; SET C, 2 Cycles:8        
        Opcode@SetBit CBD2, 8,  D_Read, 2, D_Write ;; SET D, 2 Cycles:8        
        Opcode@SetBit CBD3, 8,  E_Read, 2, E_Write ;; SET E, 2 Cycles:8     
        Opcode@SetBit CBD4, 8,  H_Read, 2, H_Write ;; SET H, 2 Cycles:8 
        Opcode@SetBit CBD5, 8,  L_Read, 2, L_Write ;; SET L, 2 Cycles:8        
        Opcode@SetBit CBD6,16,  xHL_Read, 2, xHL_Write ;; SET xHL, 2 Cycles:16       
        Opcode@SetBit CBD7, 8,  A_Read, 2, A_Write ;; SET A, 2 Cycles:8         
      
        Opcode@SetBit CBD8, 8,  B_Read, 3, B_Write ;; SET B, 3 Cycles:8 
        Opcode@SetBit CBD9, 8,  C_Read, 3, C_Write ;; SET C, 3 Cycles:8        
        Opcode@SetBit CBDA, 8,  D_Read, 3, D_Write ;; SET D, 3 Cycles:8        
        Opcode@SetBit CBDB, 8,  E_Read, 3, E_Write ;; SET E, 3 Cycles:8     
        Opcode@SetBit CBDC, 8,  H_Read, 3, H_Write ;; SET H, 3 Cycles:8 
        Opcode@SetBit CBDD, 8,  L_Read, 3, L_Write ;; SET L, 3 Cycles:8        
        Opcode@SetBit CBDE,16,  xHL_Read, 3, xHL_Write ;; SET xHL, 3 Cycles:16       
        Opcode@SetBit CBDF, 8,  A_Read, 3, A_Write ;; SET A, 3 Cycles:8 

        Opcode@SetBit CBE0, 8,  B_Read, 4, B_Write ;; SET B, 4 Cycles:8 
        Opcode@SetBit CBE1, 8,  C_Read, 4, C_Write ;; SET C, 4 Cycles:8        
        Opcode@SetBit CBE2, 8,  D_Read, 4, D_Write ;; SET D, 4 Cycles:8        
        Opcode@SetBit CBE3, 8,  E_Read, 4, E_Write ;; SET E, 4 Cycles:8     
        Opcode@SetBit CBE4, 8,  H_Read, 4, H_Write ;; SET H, 4 Cycles:8 
        Opcode@SetBit CBE5, 8,  L_Read, 4, L_Write ;; SET L, 4 Cycles:8        
        Opcode@SetBit CBE6,16,  xHL_Read, 4, xHL_Write ;; SET xHL, 4 Cycles:16       
        Opcode@SetBit CBE7, 8,  A_Read, 4, A_Write ;; SET A, 4 Cycles:8         
      
        Opcode@SetBit CBE8, 8,  B_Read, 5, B_Write ;; SET B, 5 Cycles:8 
        Opcode@SetBit CBE9, 8,  C_Read, 5, C_Write ;; SET C, 5 Cycles:8        
        Opcode@SetBit CBEA, 8,  D_Read, 5, D_Write ;; SET D, 5 Cycles:8        
        Opcode@SetBit CBEB, 8,  E_Read, 5, E_Write ;; SET E, 5 Cycles:8     
        Opcode@SetBit CBEC, 8,  H_Read, 5, H_Write ;; SET H, 5 Cycles:8 
        Opcode@SetBit CBED, 8,  L_Read, 5, L_Write ;; SET L, 5 Cycles:8        
        Opcode@SetBit CBEE,16,  xHL_Read, 5, xHL_Write ;; SET xHL, 5 Cycles:16       
        Opcode@SetBit CBEF, 8,  A_Read, 5, A_Write ;; SET A, 5 Cycles:8 
        
        Opcode@SetBit CBF0, 8,  B_Read, 6, B_Write ;; SET B, 6 Cycles:8 
        Opcode@SetBit CBF1, 8,  C_Read, 6, C_Write ;; SET C, 6 Cycles:8        
        Opcode@SetBit CBF2, 8,  D_Read, 6, D_Write ;; SET D, 6 Cycles:8        
        Opcode@SetBit CBF3, 8,  E_Read, 6, E_Write ;; SET E, 6 Cycles:8     
        Opcode@SetBit CBF4, 8,  H_Read, 6, H_Write ;; SET H, 6 Cycles:8 
        Opcode@SetBit CBF5, 8,  L_Read, 6, L_Write ;; SET L, 6 Cycles:8        
        Opcode@SetBit CBF6,16,  xHL_Read, 6, xHL_Write ;; SET xHL, 6 Cycles:16       
        Opcode@SetBit CBF7, 8,  A_Read, 6, A_Write ;; SET A, 6 Cycles:8         
      
        Opcode@SetBit CBF8, 8,  B_Read, 7, B_Write ;; SET B, 7 Cycles:8 
        Opcode@SetBit CBF9, 8,  C_Read, 7, C_Write ;; SET C, 7 Cycles:8        
        Opcode@SetBit CBFA, 8,  D_Read, 7, D_Write ;; SET D, 7 Cycles:8        
        Opcode@SetBit CBFB, 8,  E_Read, 7, E_Write ;; SET E, 7 Cycles:8     
        Opcode@SetBit CBFC, 8,  H_Read, 7, H_Write ;; SET H, 7 Cycles:8 
        Opcode@SetBit CBFD, 8,  L_Read, 7, L_Write ;; SET L, 7 Cycles:8        
        Opcode@SetBit CBFE,16,  xHL_Read, 7, xHL_Write ;; SET xHL, 7 Cycles:16       
        Opcode@SetBit CBFF, 8,  A_Read, 7, A_Write ;; SET A, 7 Cycles:8         
V_EXIT:
      mov [YG_GP]._backup, 0
      pop esi 
      pop edi 
      pop ebx 
      ret          
OPD3:
OPDB:
OPDD:
OPE3:
OPE4:
OPEB:
OPEC:
OPED:
OPF4:
OPFC:
OPFD: ud2
      align 16
OPTAB dd  OP00, OP01, OP02, OP03, OP04, OP05, OP06, OP07, OP08, OP09, OP0A, OP0B, OP0C, OP0D, OP0E, OP0F
      dd  OP10, OP11, OP12, OP13, OP14, OP15, OP16, OP17, OP18, OP19, OP1A, OP1B, OP1C, OP1D, OP1E, OP1F
      dd  OP20, OP21, OP22, OP23, OP24, OP25, OP26, OP27, OP28, OP29, OP2A, OP2B, OP2C, OP2D, OP2E, OP2F
      dd  OP30, OP31, OP32, OP33, OP34, OP35, OP36, OP37, OP38, OP39, OP3A, OP3B, OP3C, OP3D, OP3E, OP3F
      dd  OP40, OP41, OP42, OP43, OP44, OP45, OP46, OP47, OP48, OP49, OP4A, OP4B, OP4C, OP4D, OP4E, OP4F
      dd  OP50, OP51, OP52, OP53, OP54, OP55, OP56, OP57, OP58, OP59, OP5A, OP5B, OP5C, OP5D, OP5E, OP5F
      dd  OP60, OP61, OP62, OP63, OP64, OP65, OP66, OP67, OP68, OP69, OP6A, OP6B, OP6C, OP6D, OP6E, OP6F
      dd  OP70, OP71, OP72, OP73, OP74, OP75, OP76, OP77, OP78, OP79, OP7A, OP7B, OP7C, OP7D, OP7E, OP7F
      dd  OP80, OP81, OP82, OP83, OP84, OP85, OP86, OP87, OP88, OP89, OP8A, OP8B, OP8C, OP8D, OP8E, OP8F
      dd  OP90, OP91, OP92, OP93, OP94, OP95, OP96, OP97, OP98, OP99, OP9A, OP9B, OP9C, OP9D, OP9E, OP9F
      dd  OPA0, OPA1, OPA2, OPA3, OPA4, OPA5, OPA6, OPA7, OPA8, OPA9, OPAA, OPAB, OPAC, OPAD, OPAE, OPAF
      dd  OPB0, OPB1, OPB2, OPB3, OPB4, OPB5, OPB6, OPB7, OPB8, OPB9, OPBA, OPBB, OPBC, OPBD, OPBE, OPBF
      dd  OPC0, OPC1, OPC2, OPC3, OPC4, OPC5, OPC6, OPC7, OPC8, OPC9, OPCA, OPCB, OPCC, OPCD, OPCE, OPCF
      dd  OPD0, OPD1, OPD2, OPD3, OPD4, OPD5, OPD6, OPD7, OPD8, OPD9, OPDA, OPDB, OPDC, OPDD, OPDE, OPDF
      dd  OPE0, OPE1, OPE2, OPE3, OPE4, OPE5, OPE6, OPE7, OPE8, OPE9, OPEA, OPEB, OPEC, OPED, OPEE, OPEF
      dd  OPF0, OPF1, OPF2, OPF3, OPF4, OPF5, OPF6, OPF7, OPF8, OPF9, OPFA, OPFB, OPFC, OPFD, OPFE, OPFF
      align 16
CBTAB dd  CB00, CB01, CB02, CB03, CB04, CB05, CB06, CB07, CB08, CB09, CB0A, CB0B, CB0C, CB0D, CB0E, CB0F
      dd  CB10, CB11, CB12, CB13, CB14, CB15, CB16, CB17, CB18, CB19, CB1A, CB1B, CB1C, CB1D, CB1E, CB1F
      dd  CB20, CB21, CB22, CB23, CB24, CB25, CB26, CB27, CB28, CB29, CB2A, CB2B, CB2C, CB2D, CB2E, CB2F
      dd  CB30, CB31, CB32, CB33, CB34, CB35, CB36, CB37, CB38, CB39, CB3A, CB3B, CB3C, CB3D, CB3E, CB3F
      dd  CB40, CB41, CB42, CB43, CB44, CB45, CB46, CB47, CB48, CB49, CB4A, CB4B, CB4C, CB4D, CB4E, CB4F
      dd  CB50, CB51, CB52, CB53, CB54, CB55, CB56, CB57, CB58, CB59, CB5A, CB5B, CB5C, CB5D, CB5E, CB5F
      dd  CB60, CB61, CB62, CB63, CB64, CB65, CB66, CB67, CB68, CB69, CB6A, CB6B, CB6C, CB6D, CB6E, CB6F
      dd  CB70, CB71, CB72, CB73, CB74, CB75, CB76, CB77, CB78, CB79, CB7A, CB7B, CB7C, CB7D, CB7E, CB7F
      dd  CB80, CB81, CB82, CB83, CB84, CB85, CB86, CB87, CB88, CB89, CB8A, CB8B, CB8C, CB8D, CB8E, CB8F
      dd  CB90, CB91, CB92, CB93, CB94, CB95, CB96, CB97, CB98, CB99, CB9A, CB9B, CB9C, CB9D, CB9E, CB9F
      dd  CBA0, CBA1, CBA2, CBA3, CBA4, CBA5, CBA6, CBA7, CBA8, CBA9, CBAA, CBAB, CBAC, CBAD, CBAE, CBAF
      dd  CBB0, CBB1, CBB2, CBB3, CBB4, CBB5, CBB6, CBB7, CBB8, CBB9, CBBA, CBBB, CBBC, CBBD, CBBE, CBBF
      dd  CBC0, CBC1, CBC2, CBC3, CBC4, CBC5, CBC6, CBC7, CBC8, CBC9, CBCA, CBCB, CBCC, CBCD, CBCE, CBCF
      dd  CBD0, CBD1, CBD2, CBD3, CBD4, CBD5, CBD6, CBD7, CBD8, CBD9, CBDA, CBDB, CBDC, CBDD, CBDE, CBDF
      dd  CBE0, CBE1, CBE2, CBE3, CBE4, CBE5, CBE6, CBE7, CBE8, CBE9, CBEA, CBEB, CBEC, CBED, CBEE, CBEF
      dd  CBF0, CBF1, CBF2, CBF3, CBF4, CBF5, CBF6, CBF7, CBF8, CBF9, CBFA, CBFB, CBFC, CBFD, CBFE, CBFF
        
cpu_optick endp 

  end 