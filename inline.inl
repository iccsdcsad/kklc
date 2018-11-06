/* DMG/CGB internal object 
 * 
 * Copyright (C) 2018 moecmks
 * This file is part of YuduliyaGB.
 * 
 * The contents of this file are subject to the Mozilla Public License Version
 * 1.1 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 */

#ifndef INLINE_INL
#define INLINE_INL 1

#include "gameboy.h"

struct gameboy;
struct controller;
struct divider;
struct timer;
struct cpu;
struct ppu;
struct cartidge;
struct serial;
struct apu;
struct machine_setup;
struct romheader;
struct ppu_framebuffer;
/*-\
\-*/
static const 
struct machine_setup {
  double cpu_freq;  
  double vsync_freq;
  double clk_ms;
  double clk_ns;
  double frame_cycles; 
  double line_cycles;
  double oam_cycles;
  double oambg_b_cycles;
  double oambg_hbl_cycles; 
  double vbl_clk_last;
  double oambg_clk_st;
  double hbl_clk_st_b; 
  double vbl_clk_st; 
  double oam_clk_pick_per;
  double oam_clk_add_hbl_per;
  double cgb_gbp_p;
  double gbp_cgb_p;
  double oamdma_clks;
  double gdma_clks_b;
  double gdma_clks_per16; // always 7.63 us (DMG/CGB)
  double hdma_clks_per16; // always 8 us (DMG/CGB)
  double joypad_gen_request_clks; 
  double apu_squ_freq_b;
  bool cgb_machine;
} std_machine = {
  4194304.0, /* cpu_freq*/  
  59.73, /* vsync_freq*/
  4194304.0/1000.0, /* clk_ms*/
  4194304.0/1000000.0, /* clk_ns*/
  4194304.0/59.73, /* frame_cycles*/ 
  4194304.0/59.73/154.0, /* line_cycles*/
  80.992010239999985, /* oam_cycles*/
  173.51835647999999, /* oambg_b_cycles*/
  (70221.061443161/154.0) - 80.992010239999985, /* oambg_hbl_cycles*/ 
  4559.809184621, /* vbl_clk_last*/
  80.99201023999998, /* oambg_clk_st*/
  173.51835647999999 + 80.992010239999985, /* hbl_clk_st_b*/ 
  70221.061443161 - 4559.809184621, /* vbl_clk_st*/ 
  80.992010239999985 / 10.0, /* oam_clk_pick_per*/
  12.549357568000001, /* oam_clk_add_hbl_per*/
  8400000.0/ 4194304.0, /* cgb_gbp_p*/
  4194304.0/ 8400000.0, /* gbp_cgb_p*/
  671.08864000000005, /* oamdma_clks*/
  (4194304.0*220.0)/1000000.0, /* gdma_clks_b*/
  (4194304.0*7.63)/1000000.0,/* gdma_clks_per16*/
  (4194304.0*8.00)/1000000.0,/* hdma_clks_per16*/
  (4194304.0)/JOYPAD_FREQ_IN_DMG, /* joypad_gen_request_clks*/ 
  131072.0, /* apu_squ_freq_b */
  false 
}, adv_machine = {	
  8400000.0, /* cpu_freq*/  
  59.73, /* vsync_freq*/
  8400000.0/1000.0, /* clk_ms*/
  8400000.0/1000000.0, /* clk_ns*/
  8400000.0/59.73, /* frame_cycles*/ 
  8400000.0/59.73/154.0, /* line_cycles*/
  162.20400000000001, /* oam_cycles*/
  347.50799999999998, /* oambg_b_cycles*/
  (8400000.0/59.73/154.0) - 162.20400000000001, /* oambg_hbl_cycles*/ 
  9132.0031048810633, /* vbl_clk_last*/
  162.20400000000001, /* oambg_clk_st*/
  347.50799999999998 + 162.20400000000001, /* hbl_clk_st_b*/ 
  (8400000.0/59.73) - 9132.0031048810633, /* vbl_clk_st*/ 
  162.20400000000001 / 10.0, /* oam_clk_pick_per*/
  25.132800000000003, /* oam_clk_add_hbl_per*/
  8400000.0/ 4194304.0, /* cgb_gbp_p*/
  4194304.0/ 8400000.0, /* gbp_cgb_p*/
  671.08864000000005, /* oamdma_clks*/
  (8400000.0*110.0)/1000000.0, /* gdma_clks_b*/
  (8400000.0*7.63)/1000000.0,/* gdma_clks_per16*/
  (8400000.0*8.00)/1000000.0,/* hdma_clks_per16*/
  (8400000.0)/JOYPAD_FREQ_IN_CGB, /* joypad_gen_request_clks*/ 
  131072.0 * 8400000.0/ 4194304.0,/* apu_squ_freq_b */
  true
};

struct controller {
  uint8_t reg00_P1;

  /* for gamebot update host keybuf and IRQ */
  void (*clks) (struct controller *);

  void *obj;
  void *ubdata_user;
  /* pad infos for gameboy*.*/
  struct controller_pad gb_pad;
  struct gameboy *gb;
};
struct timer {
  uint8_t reg05_TIMA;
  uint8_t reg06_TMA;
  uint8_t reg07_TAC;
  double request_clks[4];

  void (*clks) (struct timer *);
  double timestamp;

  struct gameboy *gb;
};
struct divider {
  uint8_t reg04_DIV;
  double freq;

  void (*clks) (struct divider *);
  double timestamp;
  struct gameboy *gb;
};
struct gameboy {
  struct controller *joypad;
  struct cpu *lr35902;
  struct ppu *lh5028;
  struct apu *apu;
  struct cartridge *cart;
  struct divider *divider;
  struct timer *timer;
  struct serial *serial;
  struct machine_setup *mach_tools;

  double cpu_clks;
  double cpu_clks_moment;
  double cpu_clks_total;
  double cpu_clks_timer; // cur full - block 
  double cpu_clks_timerdbg; // cur full - block 
  double cpu_clks_joypad; // cur full - block 
  double cpu_clks_divider; // cur full - block 
  double cpu_clks_ppu; // cur full - block 
  double cpu_clks_apu; // cur full - block 
  double cpu_clks_cart; // cur full - block 
  double cpu_clks_serial; // RS232 Serial Port for CGB/ DMG.
  double cpu_clks_dma;
  double deflect_ms;

  uint8_t reg01_SB;
  uint8_t reg02_SC;

  uint8_t reg0F_IF; /* interrupt flags register */
  uint8_t reg56_IC;
  uint8_t reg70_SVBK;
  uint8_t regFF_IE; /* Interrupt enable register */

  /* work ram. */
  uint8_t wram[0x8000];
  /* high ram. */
  uint8_t hram[0x200];
  /* unknow mem, io read/write */
  uint8_t unknow_ram[0x10000];

  /* controller drv */
  void (*controller_hostdrv) 
             (struct gameboy *, void *controller_drvobj,
                struct controller_pad *, /* gb-self for recv joypadbuffer */ 
             struct controller_pad * /* host-edge 1?pulse gen:nodone */);
  void *controller_drvobj;
  
  /* display drv */
  void (*display_hostdrv) 
             (struct gameboy *, void *display_drvobj,
                struct ppu_framebuffer *fmebuf);
  void *display_drvobj;
  /* sound drv */
  void (*sound_hostdrv) 
             (struct gameboy *, void *sound_drvobj,
                struct apu_framebuffer *fmebuf);
  void *sound_drvobj;

  bool suspend_byuser;
};
struct oam {
  union {
    struct {
      uint8_t y;
      uint8_t x;
      uint8_t id;
      uint8_t attr;
    };
    uint8_t blk[4];
  };
};
struct ppu_cgb_palette16 {
  union {
    struct {
      uint8_t _lo;
      uint8_t _hi;
    };
    uint16_t rgb15;
    uint8_t blk[2];
  };
};
struct ppu_cgb_palette32 {
  union {
    struct {
      uint16_t _lo;
      uint16_t _hi;
    };
    uint32_t rgb32;
    uint16_t blk[2];
  };
};
/* ppu:: _oamlineframe::attr */
#define PIXEL_SPRITE_NOTRANS 1 
#define PIXEL_SPRITE_BACK 2

/* LCDC Status MASK */
#define LCDS_MODE_FLAG_HBLANK 0
#define LCDS_MODE_FLAG_VLANK 1
#define LCDS_MODE_FLAG_SERACH_OAM 2
#define LCDS_MODE_FLAG_SERACH_OAMVRAM 3
#define LCDS_MODE_FLAG_ALL_MASK 3

/* LCDC Status Interrupt MASK */
#define LCDS_INTERRUPET_LINE_MASK 0x40
#define LCDS_INTERRUPET_HBLANK_MASK 0x10
#define LCDS_INTERRUPET_VBLANK_MASK 0x08
#define LCDS_INTERRUPET_OAM_MASK 0x20
#define LCDS_INTERRUPET_ALL_MASK (LCDS_INTERRUPET_LINE_MASK|LCDS_INTERRUPET_HBLANK_MASK|LCDS_INTERRUPET_VBLANK_MASK|LCDS_INTERRUPET_OAM_MASK)

/* LCDC Control MASK */
#define LCDC_DISPLAY_MASK 0x80
#define LCDC_WINDOW_MASK 0x20 
#define LCDC_OAM_SIZE16_MASK 0x04 
#define LCDC_OAM_MASK 0x02 

#define LCD_OAM_FLIP_Y_MASK 0x40
#define LCD_OAM_FLIP_X_MASK 0x20
#define LCD_OAM_BACKGROUND_MASK 0x80
#define LCD_OAM_VRAMBANK1_MASK 0x08 /* CGB only */
#define LCD_BG_VRAMBANK1_MASK 0x08 
#define LCD_BG_HFLIP_MASK 0x20 
#define LCD_BG_VFLIP_MASK 0x40 

struct ppu {
  uint8_t *bufb16; /* for alignmem*/
  uint8_t *bufb32; /* for alignmem*/
  uint8_t ram[0x4000];  /* 8K for DMG, 16 for CGB */

  struct _oamlineframe16 /* sprite line buffer cache *.*/
  {
    uint16_t attr;
    uint16_t pixel;
  } olf16[176];

  struct _oamlineframe32 /* sprite line buffer cache pixel-rgb24*.*/
  {
    uint32_t attr;
    uint32_t pixel;
  } olf32[176];

  struct gameboy *gb;
  struct oam sp[40]; 
  struct ppu_cgb_palette16 cgb_sp_pal16[8][4]; 
  struct ppu_cgb_palette16 cgb_bg_pal16[8][4]; 
  struct ppu_cgb_palette32 cgb_sp_pal32[8][4]; 
  struct ppu_cgb_palette32 cgb_bg_pal32[8][4]; 
  struct ppu_cgb_palette16 cgb_sp_pal16_x1r5g5b5[8][4]; 
  struct ppu_cgb_palette16 cgb_bg_pal16_x1r5g5b5[8][4]; 
  struct ppu_cgb_palette32 cgb_sp_pal32_x0r8g8b8[8][4]; 
  struct ppu_cgb_palette32 cgb_bg_pal32_x0r8g8b8[8][4];
  struct ppu_dmg_palette16 dmg_pal16;
  struct ppu_dmg_palette16 dmg_pal16T;
  struct ppu_dmg_palette32 dmg_pal32;
  struct ppu_dmg_palette32 dmg_pal32T;
  struct ppu_framebuffer fmebuf;
  
  uint8_t reg40_LCDC; 
  uint8_t reg40_NMIf;
  uint8_t reg41_LCDS;   
  uint8_t reg41_LCDM_T;
  uint8_t reg41_IRQf;
  uint8_t reg42_SCY;
  uint8_t reg43_SCX;     
  uint8_t reg44_LY;
  uint8_t reg44_LY_T;
  uint8_t reg45_LYC;
  uint8_t reg46_DMA;
  uint8_t reg47_BGP;
  uint8_t reg48_OBP0;
  uint8_t reg49_OBP1;
  uint8_t reg4A_WY;
  uint8_t reg4A_WYRSC;
  uint8_t reg4A_WYLineHit;
  uint8_t reg4B_WX;
  uint8_t reg4F_VBK;
  uint8_t reg51_HDMA1;
  uint8_t reg52_HDMA2;
  uint8_t reg53_HDMA3;
  uint8_t reg54_HDMA4;
  uint8_t reg55_HDMA5;
  uint8_t reg68_BCPS;
  uint8_t reg69_BCPD;
  uint8_t reg6A_OCPS;
  uint8_t reg6B_OCPD;

  int32_t vscan; 
  int32_t vscanR;
  int32_t vscan40;
  int32_t xscanR;
  int32_t uscan; 
  int32_t uscanR; 

  bool hdma_gen;

  PPU_SAMPLE_FORMAT pixel_format;

  uint16_t hdma_src;
  uint16_t hdma_dst;
  uint16_t hdma_r16;

  double hdma_clks;
  double hbl_clks_st; 
  double oambg_clks_divider21;

  void (*bgwin_done) (struct ppu *_5028, int16_t scanline);
  void (*sprite_done) (struct ppu *_5028, int16_t delta);
  void (*device_blit) (struct ppu *_5028, void *obj, struct ppu_framebuffer *fbuf); /*blit for host devcice */

  void *obj;
  void (*clks) (struct ppu *);
};

#pragma pack (push)
#pragma pack (1)
struct cpu {
  /*  XXX:memory order dep. don't set struct align!*/
  union { struct { uint8_t F; uint8_t A; }; uint16_t AF; };
  union { struct { uint8_t C; uint8_t B; }; uint16_t BC; };
  union { struct { uint8_t E; uint8_t D; }; uint16_t DE; };
  union { struct { uint8_t L; uint8_t H; }; uint16_t HL; };
  union { struct { uint8_t SL; uint8_t SH; }; uint16_t SP; };
  union { struct { uint8_t PL; uint8_t PH; }; uint16_t PC; };

   /* Interrupt Master Enable*/
  uint8_t IME;

   /* for Halt */
  bool halt;  

   /* for stop */
  bool stop; 

   /* for halt bug */
  intptr_t _backup;

   /* for speed mode */
  uint8_t reg4D_key1;

   /* gameboy object  */
  struct gameboy *gb;              
};
#pragma pack (pop)

#define NR43_LFSR_STEP7_MASK 0x08
#define NR42_ENVLOPE_INC_MASK 0x08
#define NRX4_DIS_CONSECUTIVE_MASK 0x40
#define NR44_DIS_CONSECUTIVE_MASK 0x40
#define NR22_ENVLOPE_INC_MASK 0x08
#define NRX2_ENVLOPE_INC_MASK 0x08
#define NR10_FREQ_SWEEP_SUB_MASK 0x08
#define NR30_ON_MASK 0x80
#define NR52_SOUND_CIRCUITS_OPEN_MASK 0x80
#define NR50_S02_LEFT_MASK 0x80
#define NR50_S01_RIGHT_MASK 0x08
#define NRX4_ON_MASK 0x80
#define NR52_ON_MASK 0x80

struct sample8 {
  uint8_t left;
  uint8_t right;
};
struct sample16 {
  int16_t left;
  int16_t right;
};
struct sample32f {
  float left;
  float right;
};
struct seq_channel {
  int32_t len_divc; /* 256hz */
  int32_t env_divc; /* 64hz  */
  int32_t sweep_divc; /* 128hz */
};
struct square_channel {
  struct seq_channel seq;
  bool chan_en;
  bool sweep_en;
  bool envlope_en;
  uint16_t vol_len;
  uint16_t vol_lenT;
  uint8_t vol_init;
  uint8_t vol_render; /* volume for render */
  uint8_t signal; /* 1bit for render */
  int32_t sweep_div;
  int32_t env_div;
  int32_t freqT;
  double freqT2;
  int32_t shadow; /* for freq sweep see http://gbdev.gg8.se/wiki/articles/Gameboy_sound_hardware */
  double shadow2;
  double squGen;
  double squGen_T; /* for square signal scan */
  uint8_t squPhase;
# define s_nx_start squReg_NRX0_X4
# define s_sweep squReg_NRX0_X4[0]
# define s_dutylen squReg_NRX0_X4[1]
# define s_envlope squReg_NRX0_X4[2]
# define s_freqlo squReg_NRX0_X4[3]
# define s_freqhi_rst squReg_NRX0_X4[4]
  uint8_t squReg_NRX0_X4[5];
};
struct wave_channel {
  struct seq_channel seq;
  bool chan_en;
  uint16_t vol_len;
  uint16_t vol_lenT;
  uint8_t vol_cur;
  uint8_t vol_sft;
  double freqT2;
  double wavGen;
  double wavGen_T; /* for wave signal scan */
  uint8_t wavPhase;
# define w_nx_start  wavReg_NRX0_X4
# define w_dac wavReg_NRX0_X4[0]
# define w_sndlen wavReg_NRX0_X4[1]
# define w_outlevel wavReg_NRX0_X4[2]
# define w_freqlo wavReg_NRX0_X4[3]
# define w_freqhi_rst wavReg_NRX0_X4[4]
  uint8_t wavReg_NRX0_X4[5];
  uint8_t pcm4_T[32];
  uint8_t pcm4_NR30_3F[16];
};
struct noise_channel {
  struct seq_channel seq;
  bool chan_en;
  bool envlope_en;
  uint16_t vol_len;
  uint16_t vol_lenT;
  uint8_t vol_render; /* volume for render */
  uint8_t vol_init;
  uint8_t signal; /* 1bit for render */
  int32_t env_div;
  double noiGen;
  double noiGen_T; /* for noise signal scan */
# define n_nx_start  noiReg_NRX0_X4
# define n_useless noiReg_NRX0_X4[0]
# define n_sndlen noiReg_NRX0_X4[1]
# define n_envlope noiReg_NRX0_X4[2]
# define n_sftdiv_lfsr noiReg_NRX0_X4[3]
# define n_rst noiReg_NRX0_X4[4]
  uint8_t noiReg_NRX0_X4[5];
  intptr_t lfsrSeed;
};
struct fsequencer {
  double fseq;
  double fseqT; /* Frame Sequencer 512Hz */
  uint8_t fseq_div; 
  uint64_t fseq_count;
};
struct apu {
  struct fsequencer fseq;
  struct square_channel squ_chan[2];
  struct wave_channel wav_chan;
  struct noise_channel noi_chan;

  /*  
       FF24 - NR50 - Channel control / ON-OFF / Volume (R/W)
       The volume bits specify the "Master Volume" for Left/Right sound output. 
       SO2 goes to the left headphone, and SO1 goes to the right.

       Bit 7   - Output Vin to SO2 terminal (1=Enable)
       Bit 6-4 - SO2 output level (volume)  (0-7)
       Bit 3   - Output Vin to SO1 terminal (1=Enable)
       Bit 2-0 - SO1 output level (volume)  (0-7)

       The Vin signal is received from the game cartridge bus, 
       allowing external hardware in the cartridge to supply a fifth sound channel, 
       additionally to the gameboys internal four channels. 
       As far as I know this feature isn't used by any existing games.
  */
  struct {
    union {
      struct {
        uint8_t so1_outlevel:3;
        bool so1_vin:1;     
        uint8_t so2_outlevel:3;
        bool so2_vin:1;  
      };
      uint8_t reg24_NR50;
    };
  };
  /*   
       FF25 - NR51 - Selection of Sound output terminal (R/W)
       Each channel can be panned hard left, center, or hard right.
  
       Bit 7 - Output sound 4 to SO2 terminal
       Bit 6 - Output sound 3 to SO2 terminal
       Bit 5 - Output sound 2 to SO2 terminal
       Bit 4 - Output sound 1 to SO2 terminal
       Bit 3 - Output sound 4 to SO1 terminal
       Bit 2 - Output sound 3 to SO1 terminal
       Bit 1 - Output sound 2 to SO1 terminal
       Bit 0 - Output sound 1 to SO1 terminal
  */
  struct {
    union {
      struct {
        bool so1out_squ:1;
        bool so1out_squ2:1;
        bool so1out_wave:1;
        bool so1out_noi:1;
        bool so2out_squ:1;
        bool so2out_squ2:1;
        bool so2out_wave:1;
        bool so2out_noi:1;
      };
      uint8_t reg25_NR51;
    };
  };
  /*  
       FF26 - NR52 - Sound on/off
       If your GB programs don't use sound then write 00h to this register to save 16% or\
       more on GB power consumption. 
       Disabeling the sound controller by clearing Bit 7 destroys the contents of all sound registers. 
       Also, it is not possible to access any sound registers (execpt FF26) while the sound controller is disabled.

       Bit 7 - All sound on/off  (0: stop all sound circuits) (Read/Write)
       Bit 3 - Sound 4 ON flag (Read Only)
       Bit 2 - Sound 3 ON flag (Read Only)
       Bit 1 - Sound 2 ON flag (Read Only)
       Bit 0 - Sound 1 ON flag (Read Only)
   */
  struct {
    union {
      struct {
        bool squ_on:1;
        bool squ2_on:1;
        bool wav_on:1;
        bool noi_on:1;
        bool unused:1;
        bool unused2:1;
        bool unused3:1;
        bool snd_master_en:1;
      };
      uint8_t reg26_NR52;
    };
  };
  /*  undocumented io */
  uint8_t reg6C_undoc; 
  uint8_t reg72_undoc; 
  uint8_t reg73_undoc;  
  uint8_t reg74_undoc;
  uint8_t reg75_undoc;
  uint8_t reg76_PCMC12; 
  uint8_t reg77_PCMC34;
  
  union buffer_bank {
    /*  max_size 4(float)*2(stereo)*44100(max sample)/60 = 5880 byte */
    struct sample8 *pcm_buf8; /* 1/60 s buffer */
    struct sample16 *pcm_buf16; /* 1/60 s buffer */
    struct sample32f *pcm_buff; /* 1/60 s buffer */
    uint8_t *pcm_buf8r;
    void *pcm_buf; /* 1/60 s buffer * APU_BUFFER_BUMS (ring)*/
  } base_bank, poll_bank;

  uint32_t pcmringbuf_size; /* in byte */
  uint32_t pcmbank;
  uint32_t pcmbank_size;
  uint32_t pcm_channel; 
  uint32_t sam; // sample nums 
  uint32_t samT; // sample nums 
  uint32_t samBitDepth; // 

  double samGen; // sample timestamp count
  double samGenT; // clock period needed for a sampling point
  
  void (*clks) (struct apu *);
  struct gameboy *gb;

  /* sound drv */
  void (*sound_hostdrv) 
             (struct apu *, void *sound_drvobj,
                struct apu_framebuffer *fmebuf);
  void *sound_drvobj;

  APU_SAMPLE_FREQ sam_freq;
  APU_SAMPLE_TYPE sam_type;
};
struct serial {
  void (*clks) (struct serial *);
  struct gameboy *gb;
};
/* cartridge device type */
#define MBC_0 0
#define MBC_1 1
#define MBC_2 2
#define MBC_3 3
#define MBC_4 4
#define MBC_5 5 
#define MBC_6 6
#define MBC_7 7
#define TAMA5 8
#define HUCL1 9
#define HUCL3 10
#define MMM0 11
#define POCKER_CAM 12

/* MBC1 */
#define MBC1_MODE0_2MROM_8KRAM 0
#define MBC1_MODE1_512KROM_32KRAM 1
typedef int mbc1_cart_mode;

struct mbc1_chip {
  mbc1_cart_mode mode;
  bool ram_en;
  uint16_t prombank;
  uint16_t srambank;
  uint16_t bankcac;
};
/* MBC5 */
#define MBC5_MODE0_8MROM_8KRAM 0
#define MBC5_MODE1_512KROM_32KRAM 1
typedef int mbc5_cart_mode;

struct mbc5_chip {
  mbc5_cart_mode mode;
  bool ram_en;
  uint16_t prombank;
  uint16_t srambank;
  uint16_t bankcac;
};

struct romheader {
  uint8_t title[16];
  uint16_t curlic;
  uint8_t sgb;
  uint8_t ctype;
  uint8_t promsize;
  uint8_t ramsize;
  uint8_t targetcode;
  uint8_t anclic;
  uint8_t maskver;
  uint8_t hdcrc;
  uint16_t gcrc;
};
struct cartridge {
  struct romheader header;
  bool s_latch;
  bool battery;
  uint8_t *promworks;
  uint8_t *sramworks;
  uint8_t promsize;
  uint8_t sramsize; 

  void (*clks) (struct cartridge *);
  uint8_t (*read) (struct cartridge *cartridge, uint16_t address);
  void (*write) (struct cartridge *cartridge, uint16_t address, uint8_t value);

  union {
    struct mbc1_chip *mbc1;
    void *ubdata_user;
  };
  struct gameboy *gb;    
};

#define IRQ_1 0x01  /*  VBLANK (NMI)  */
#define IRQ_2 0x02  /*  LCDC  */
#define IRQ_3 0x04  /*  Programmable timer */
#define IRQ_4 0x08  /*  Serial port switching */
#define IRQ_5 0x10  /*  P14-15 Descent edge acknowledge */
#define IRQ_NIL 0xFF  

#define IRQ_1_ADDRESS 0x40 /*  VBLANK (NMI)  */
#define IRQ_2_ADDRESS 0x48 /*  LCDC  */
#define IRQ_3_ADDRESS 0x50 /*  Programmable timer */
#define IRQ_4_ADDRESS 0x58 /*  Serial port switching */
#define IRQ_5_ADDRESS 0x60 /*  P14-15 Descent edge acknowledge */

#endif 