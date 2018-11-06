/* easy, simple test for win32.
 *
 * subsystem:console
 * drawdev:GDI::DIB
 * inputdev:win32callback 
 * sounddev:mmio::waveAPI.
 *
 * Copyright (C) 2018 moecmks
 * This file is part of KS3578.
 * 
 * do What The Fuck you want to Public License
 * 
 * Version 1.0, March 2000
 * Copyright (C) 2000 Banlu Kemiyatorn (]d).
 * 136 Nives 7 Jangwattana 14 Laksi Bangkok
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 * 
 * Ok, the purpose of this license is simple
 * and you just
 * 
 * DO WHAT THE FUCK YOU WANT TO.
 */

#include <Windows.h>
#include <tchar.h>
#include <assert.h>
#include "gameboy.h"
#include <d3d9.h>

// 32K ROM TEST
//#define ROM_PATH1 ("C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\o.gb")
//#define ROM_PATH12 _T("C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\o.gb")

#define ROMPATH_1 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\01-special.gb"
#define ROMPATH_2 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\02-interrupts.gb"
#define ROMPATH_3 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\03-op sp,hl.gb"
#define ROMPATH_4 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\04-op r,imm.gb"
#define ROMPATH_5 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\05-op rp.gb"
#define ROMPATH_6 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\06-ld r,r.gb"
#define ROMPATH_7 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\07-jr,jp,call,ret,rst.gb"
#define ROMPATH_8 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\08-misc instrs.gb"
#define ROMPATH_9 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\09-op r,r.gb"
#define ROMPATH_10 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\10-bit ops.gb"
#define ROMPATH_11 "C:\\Users\\nozaki\\Documents\\Visual Studio 2012\\Projects\\KS3578\\KS3578\\core\\11-op a,(hl).gb"
#define ROMPATH_12 "F:\\YuduliyaGB\\core\\oo.gb"
#define ROMPATH_13 "F:\\YuduliyaGB\\core\\01-registers.gb"
#define ROMPATH_14 "F:\\YuduliyaGB\\core\\02-len ctr.gb"
#define ROMPATH1 ROMPATH_12


static IDirect3D9 *sm_Direct3D9_root=0;
static IDirect3DDevice9  *sm_Direct3D9_device = NULL;   /* device object */
static IDirect3DSurface9  *sm_Direct3D9_surface = NULL; /* vram object */
static IDirect3DTexture9 *sm_Direct3D9_texture = NULL;

// JOYPAD MAPPER.
#define JOYPAD_MAPPER_LEFT 'A'
#define JOYPAD_MAPPER_RIGHT 'D'
#define JOYPAD_MAPPER_TOP 'W'
#define JOYPAD_MAPPER_BOTTOM 'S'
#define JOYPAD_MAPPER_B 'J'
#define JOYPAD_MAPPER_A 'K'
#define JOYPAD_MAPPER_SELECT ' '
#define JOYPAD_MAPPER_START 'T'

// for simple use gval static gloabl functio n. 
static HWND g_window = NULL;
static HDC g_memdc = NULL;
static HGDIOBJ g_oob = NULL;
static PWORD g_pixbuf = NULL;
static DWORD g_style = WS_OVERLAPPEDWINDOW;
static DWORD g_styleex = 0;
static LONG g_width = 160;
static LONG g_height = 144;
static TCHAR g_appname[] = TEXT ("KS3578 Test");
static struct controller_pad g_joypad;
static struct gameboy *g_dmg;
static DWORD g_dbgline;
static HANDLE g_audio_thread;
static HWAVEOUT g_audio_handle;
static WAVEHDR g_audio_header[APU_BUFFER_BUMS];
static WAVEHDR g_audio_delay;
static BOOL volatile   buffer_player_com = FALSE;
static BYTE audio_delay[44100/5];

static 
void CALLBACK waveOutProc(HWAVEOUT hwo, UINT uMsg, DWORD dwInstance, DWORD dwParam1, DWORD dwParam2)
{
    switch (uMsg)
    {
    case WOM_DONE:
        buffer_player_com = TRUE;
        break;
    }
}

static 
void sound_hostdrv_s (struct gameboy *gb, 
                      void *sound_drvobj,
                      struct apu_framebuffer *fmebuf)
{
  //while (buffer_player_com != TRUE) ;
  //buffer_player_com = FALSE;
  static int i = 0;
  if (i == 0) {
    i = 1;
    // delay audio buffer 2000ms 
    waveOutWrite (g_audio_handle, & g_audio_delay, sizeof(WAVEHDR));



  }


  waveOutWrite (g_audio_handle, & g_audio_header[fmebuf->id], sizeof(WAVEHDR));

}

static void win32_sound_init() {
  int i;
  void *buf[APU_BUFFER_BUMS];
	WAVEFORMATEX wfx;
	wfx.nSamplesPerSec = 44100;
	wfx.wBitsPerSample = 16;
	wfx.nChannels = 2;
	wfx.cbSize = 0;		// size of _extra_ info
	wfx.wFormatTag = WAVE_FORMAT_PCM;
	wfx.nBlockAlign = (wfx.wBitsPerSample >> 3) * wfx.nChannels;
	wfx.nAvgBytesPerSec = wfx.nBlockAlign * wfx.nSamplesPerSec;
  gameboy_getaudiobuf (g_dmg, buf);

	if (waveOutOpen(&g_audio_handle, WAVE_MAPPER, &wfx, (DWORD_PTR)waveOutProc, 0, CALLBACK_FUNCTION) != MMSYSERR_NOERROR) {
		fprintf(stderr, "unable to open WAVE_MAPPER device\n");
		return;
	}
  for (i = 0; i < APU_BUFFER_BUMS; i++) {
		g_audio_header[i].lpData = (LPSTR)buf[i];
		g_audio_header[i].dwBufferLength = 5880;
		waveOutPrepareHeader(g_audio_handle, &g_audio_header[i], sizeof(WAVEHDR));
		// waveOutWrite(g_audio_handle, &g_audio_header[i], sizeof(WAVEHDR));
	}
	g_audio_delay.lpData = (LPSTR)audio_delay;
	g_audio_delay.dwBufferLength = sizeof (audio_delay);
  waveOutPrepareHeader(g_audio_handle, &g_audio_delay, sizeof(WAVEHDR));
}

// FOR ¡¡ClientSIze 
static 
BOOL ReSize160_144 (void) 
{
	RECT rc;
  DWORD cx = GetSystemMetrics (SM_CXSCREEN)/2 - 80;
  DWORD cy = GetSystemMetrics (SM_CYSCREEN)/2 - 72;

	if (!GetClientRect( g_window, &rc)) return FALSE;

	if (rc.right != -1) rc.right = g_width;
	if (rc.bottom != -1) rc.bottom = g_height;

	if ( !AdjustWindowRectEx(&rc, g_style, FALSE, g_styleex))
		return FALSE;

	return SetWindowPos (g_window, NULL, cx, cy, rc.right - rc.left, rc.bottom - rc.top, SWP_NOZORDER | SWP_NOACTIVATE);
}

static 
LRESULT CALLBACK WndProc (HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) 
{
  switch (message) {
  case WM_DESTROY:
    PostQuitMessage (0);
    return 0;
  case WM_KEYUP:
    switch (wParam) {
    case JOYPAD_MAPPER_LEFT: g_joypad.left = 0; break;
    case JOYPAD_MAPPER_RIGHT: g_joypad.right = 0; break;
    case JOYPAD_MAPPER_TOP: g_joypad.up = 0; break;
    case JOYPAD_MAPPER_BOTTOM: g_joypad.down = 0; break;
    case JOYPAD_MAPPER_A: g_joypad.a = 0; break;
    case JOYPAD_MAPPER_B: g_joypad.b = 0; break;
    case JOYPAD_MAPPER_SELECT: g_joypad.select = 0; break;
    case JOYPAD_MAPPER_START: g_joypad.start = 0; break;
    default:
      break;
    }
    break;
  case WM_KEYDOWN:
    switch (wParam) {
    case JOYPAD_MAPPER_LEFT: g_joypad.left = 1; break;
    case JOYPAD_MAPPER_RIGHT: g_joypad.right = 1; break;
    case JOYPAD_MAPPER_TOP: g_joypad.up = 1; break;
    case JOYPAD_MAPPER_BOTTOM: g_joypad.down = 1; break;
    case JOYPAD_MAPPER_A: g_joypad.a = 1; break;
    case JOYPAD_MAPPER_B: g_joypad.b = 1; break;
    case JOYPAD_MAPPER_SELECT: g_joypad.select = 1; break;
    case JOYPAD_MAPPER_START: g_joypad.start = 1; break;
    default:
      break;
    }
  default:
    break;
  }
  return DefWindowProc (hwnd, message, wParam, lParam);
}
 void __cdecl scanline_32K (uint16_t *dstPtr, intptr_t dstPitch, 
                      uint16_t *srcPtr, intptr_t srcPitch,
               intptr_t dstW, intptr_t dstH,
               intptr_t srcW, intptr_t srcH, uint32_t alphaV );


// Create DIB. 
static 
void CreateDIB (void) {

  HBITMAP bitmap = NULL; // attach buffer .
	BITMAPINFO bmpinfos = {0};
  HDC tmpDC = NULL;
  
  // GetDC 
  tmpDC = GetDC (g_window);
  assert (tmpDC != NULL);

  // Create MEMDC.
  g_memdc = CreateCompatibleDC (tmpDC);
  assert (g_memdc != NULL);

	bmpinfos.bmiHeader.biSize        =  sizeof (bmpinfos);
	bmpinfos.bmiHeader.biWidth       =  2000;
	bmpinfos.bmiHeader.biHeight      = -1500; // see https://msdn.microsoft.com/en-us/library/windows/desktop/dd183376(v=vs.85).aspx
	bmpinfos.bmiHeader.biPlanes      =  1;
	bmpinfos.bmiHeader.biBitCount    =  16;
	bmpinfos.bmiHeader.biCompression =  BI_RGB;

  // Create DIB bitmap.
	bitmap = CreateDIBSection	(g_memdc, & bmpinfos, 
										             DIB_RGB_COLORS, (void* *)& g_pixbuf, NULL, 0);
  assert (bitmap != NULL);

  // Secetl OBJ.
  g_oob = SelectObject (g_memdc, bitmap);
  assert (g_oob != NULL && (g_oob != HGDI_ERROR));

  // Release DC.
  ReleaseDC (g_window, tmpDC);
}

 void __cdecl bilinear_x32K (uint16_t *dstPtr, intptr_t dstPitch, 
                      uint16_t *srcPtr, intptr_t srcPitch,
               intptr_t dstW, intptr_t dstH,
               intptr_t srcW, intptr_t srcH);


void DMG_PpuDone (struct gameboy *dmg, void *obj, struct ppu_framebuffer *fbuf) {
  
  // copy to emmdc.
  DWORD id;
  HDC tmpDC;
  RECT rc;
  HRESULT si;
          int w;
        int h ;
        DWORD n = timeGetTime ();
        RECT rcClient;
        D3DLOCKED_RECT lock_rect;
        
        IDirect3DSurface9 *surface_temp;
          GetClientRect (g_window, & rc);
        GetClientRect (g_window, & rcClient);
        w = abs (rcClient.left - rcClient.right);
        h = abs (rcClient.bottom - rcClient.top);
        if (w <= 0 || h <= 0)
          return;
        
        si = IDirect3DDevice9_GetBackBuffer (sm_Direct3D9_device, 0, 0, D3DBACKBUFFER_TYPE_MONO, & surface_temp);
        assert (SUCCEEDED (si));
        si = IDirect3DSurface9_LockRect (sm_Direct3D9_surface, & lock_rect, & rcClient, D3DLOCK_DISCARD | D3DLOCK_NOOVERWRITE);
        assert (SUCCEEDED (si));
         
        // bilinear entryu .
        bilinear_x32K ((DWORD *)lock_rect.pBits, lock_rect.Pitch, 
          (DWORD *)fbuf->buf, fbuf->pitch, 
          rcClient.right - rcClient.left, 
          rcClient.bottom - rcClient.top, 160, 144);
         
        IDirect3DSurface9_UnlockRect (sm_Direct3D9_surface);
        IDirect3DDevice9_UpdateSurface (sm_Direct3D9_device, sm_Direct3D9_surface, & rcClient, surface_temp, NULL);
        IDirect3DDevice9_Present (sm_Direct3D9_device, & rcClient, NULL, NULL, NULL);
        //printf ("Present::frame:%d!!!!\n", timeGetTime () - n);



  //tmpDC = GetDC (g_window);
  //BitBlt (tmpDC, 0, 0, rc.right - rc.left, rc.bottom - rc.top, g_memdc, 0, 0, SRCCOPY);
  // StretchBlt (tmpDC, 0, 0, rc.right - rc.left, rc.bottom - rc.top,
    // g_memdc, 0, 0, 160, 144, SRCCOPY);
  //ReleaseDC (g_window, tmpDC);
}

void DMG_PpuDone___ (void) {
#if 0
                 (0x1F << 0) | (0x1F << 5) | (0x1F << 10), /* color index 0*/
                 (0x15 << 0) | (0x15 << 5) | (0x15 << 10),  /* color index 1*/
                 (0x0A << 0) | (0x0A << 5) | (0x0A << 10),  /* color index 2*/
                 (0x00 << 0) | (0x00 << 5) | (0x00 << 10)  /* color index 3*/
#endif 
  // copy to emmdc.
  DWORD id;
  HDC tmpDC;
  for (id = 0; id != 144; id++) {
    memset (& g_pixbuf[id*160], 0xCC, 320);
  }
  tmpDC = GetDC (g_window);
  BitBlt (tmpDC, 0, 0, 160, 144, g_memdc, 0, 0, SRCCOPY);
  ReleaseDC (g_window, tmpDC);
}

void DMG_InputDone (struct gameboy *dmg, 
                    void *obj,
                struct controller_pad *gb_pad, /* gb-self for recv joypadbuffer */ 
             struct controller_pad *host /* host-edge */) 
{
  memcpy (gb_pad, &g_joypad, sizeof (struct controller_pad));
  memset (host, 0, sizeof (*host));
}

// Create WIDNOWS>. 
static 
void CreateCGBWindow (void) {

  WNDCLASS     wndclass ;
  BOOL  assertit;

  wndclass.style         = CS_HREDRAW | CS_VREDRAW ;
  wndclass.lpfnWndProc   = WndProc ;
  wndclass.cbClsExtra    = 0 ;
  wndclass.cbWndExtra    = 0 ;
  wndclass.hInstance     = GetModuleHandle (NULL) ;
  wndclass.hIcon         = LoadIcon (NULL, IDI_APPLICATION) ;
  wndclass.hCursor       = LoadCursor (NULL, IDC_ARROW) ;
  wndclass.hbrBackground = (HBRUSH) GetStockObject (WHITE_BRUSH) ;
  wndclass.lpszMenuName  = NULL ;
  wndclass.lpszClassName = g_appname ;

  if (!RegisterClass (&wndclass))
  {
    MessageBox (NULL, TEXT ("This program requires Windows NT!"),
                g_appname, MB_ICONERROR) ;
    return ;
  }
  g_window = CreateWindow (g_appname,                  // window class name
                       g_appname, // window caption
                       g_style,        // window style
                       CW_USEDEFAULT,              // initial x position
                       CW_USEDEFAULT,              // initial y position
                       CW_USEDEFAULT,              // initial x size
                       CW_USEDEFAULT,              // initial y size
                       NULL,                       // parent window handle
                       NULL,                       // window menu handle
                       wndclass.hInstance,                  // program instance handle
                       NULL) ;                     // creation parameters

  assert (g_window != NULL);
  assertit = ReSize160_144 ();
  assert (assertit != FALSE);
  ShowWindow (g_window, SW_SHOWNORMAL);
  assertit = UpdateWindow (g_window);
  assert (assertit != FALSE);

  CreateDIB ();
}

int /* ret: prom bank size fail:-1*/
romheader_get (FILE *read, char *buf);
int 
lr35902_disassembler (FILE *read, FILE *output);

void ResetD3dpp (D3DPRESENT_PARAMETERS *d3dpp, HWND window, int width, int height)  {

  ZeroMemory (d3dpp, sizeof (*d3dpp));

  d3dpp->BackBufferFormat      = D3DFMT_X1R5G5B5;
  d3dpp->SwapEffect            = D3DSWAPEFFECT_DISCARD;
  d3dpp->Flags                 = 0;
  d3dpp->hDeviceWindow         = window;
  d3dpp->Windowed              = TRUE;
  d3dpp->BackBufferWidth       = GetSystemMetrics (SM_CXSCREEN);
  d3dpp->BackBufferHeight      = GetSystemMetrics (SM_CYSCREEN);
  d3dpp->PresentationInterval  = D3DPRESENT_INTERVAL_DEFAULT; /* Disable VSync */
}

int WINAPI _tWinMaint (HINSTANCE hInstance, HINSTANCE hPrev, LPTSTR pszCmdLine, int nCmdShow) {

  MSG msg;
  FILE *win32fd;
  DWORD c =0;
  D3DPRESENT_PARAMETERS d3dpp;
  HRESULT si;
  CreateCGBWindow ();
  // create direct3d 9 
  sm_Direct3D9_root = Direct3DCreate9 (DIRECT3D_VERSION);
  assert (sm_Direct3D9_root != NULL);
  

  // create device 
  
  ResetD3dpp (& d3dpp, g_window, 0, 0);
si = IDirect3D9_CreateDevice (sm_Direct3D9_root, 0,
                        D3DDEVTYPE_HAL, 
                           g_window, 
                              D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, & sm_Direct3D9_device);
  assert (SUCCEEDED (si));

  IDirect3DDevice9_CreateOffscreenPlainSurface (sm_Direct3D9_device, GetSystemMetrics (SM_CXSCREEN), 
    GetSystemMetrics (SM_CYSCREEN), D3DFMT_X1R5G5B5, D3DPOOL_SYSTEMMEM, 
        & sm_Direct3D9_surface, NULL);

  gameboy_init (& g_dmg);
  gameboy_controller_drv (g_dmg, DMG_InputDone, null);
  gameboy_lcdvideo_drv (g_dmg, DMG_PpuDone, null);
  gameboy_sound_drv (g_dmg, sound_hostdrv_s, null);


  win32fd = fopen (ROMPATH1, "rb");
  // dbgp = fopen ("F:\gbz80.asm", "wb");
  // lr35902_disassembler2 (_T(ROMPATH_12), _T("F:\gbz80.asm"));

  assert (win32fd != NULL);
  gameboy_loadrom (g_dmg, win32fd);
  // romheader_get (win32fd, buf);
 // lr35902_disassembler (win32fd, dbgp);

   win32_sound_init();
  while(TRUE)
  {
   // test if there is a message in queue, if so get it
    c = timeGetTime ();
   if (PeekMessage(&msg,NULL,0,0,PM_REMOVE))
   { 
     // test if this is a quit
     if (msg.message == WM_QUIT)
       break;

     // translate any accelerator keys
     TranslateMessage(&msg);

     // send the message to the window proc
     DispatchMessage(&msg);
   } // end if
   //DMG_PpuDone___ ();
   c = timeGetTime ();
   gameboy_run_ms (g_dmg, 16.742005692282);
   //printf ("frame ticks:%d\n", timeGetTime () - c);
#if 0
   {
     DWORD cs= 16;
     while (TRUE) {
       cs = timeGetTime ();
       if ((cs - c) > 16)
         break;
     }
   }
#endif 
  } // end while
  return msg.wParam ;
}

int main (void) {
  _tWinMaint (GetModuleHandle (NULL), NULL, _T ("KS3578.exe"), SW_SHOWNORMAL);
  return 0;
}