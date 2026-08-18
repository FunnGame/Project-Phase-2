/**
 ******************************************************************************
 * @file    ili9341.c
 * @brief   ILI9341 TFT driver implementation (STM32F446RE control station).
 ******************************************************************************
 */
#include "ili9341.h"
#include "systick.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/*  Controller command set (only what this driver actually issues)            */
/* -------------------------------------------------------------------------- */

#define CMD_SWRESET   0x01u
#define CMD_SLPOUT    0x11u
#define CMD_INVOFF    0x20u
#define CMD_INVON     0x21u
#define CMD_GAMMASET  0x26u
#define CMD_DISPOFF   0x28u
#define CMD_DISPON    0x29u
#define CMD_CASET     0x2Au  /* column address set                           */
#define CMD_PASET     0x2Bu  /* page (row) address set                       */
#define CMD_RAMWR     0x2Cu  /* start streaming pixels into GRAM             */
#define CMD_MADCTL    0x36u  /* memory access control - rotation and BGR     */
#define CMD_PIXFMT    0x3Au
#define CMD_FRMCTR1   0xB1u
#define CMD_DFUNCTR   0xB6u
#define CMD_PWCTR1    0xC0u
#define CMD_PWCTR2    0xC1u
#define CMD_VMCTR1    0xC5u
#define CMD_VMCTR2    0xC7u
#define CMD_GMCTRP1   0xE0u
#define CMD_GMCTRN1   0xE1u

/* MADCTL bits. BGR is set in every orientation: these panels are wired BGR,
 * and clearing it swaps red and blue - which on a screen whose whole job is
 * red-for-braking would be an actively dangerous bug. */
#define MADCTL_MY     0x80u
#define MADCTL_MX     0x40u
#define MADCTL_MV     0x20u
#define MADCTL_BGR    0x08u

/* Pixels per chunk when streaming a solid colour or a byte-swapped bitmap.
 * 32 pixels = 64 bytes: small enough to sit on the stack without thought,
 * large enough that spi_transfer()'s end-of-call BSY wait is amortised. */
#define CHUNK_PIXELS  32u

/* -------------------------------------------------------------------------- */
/*  Module state - one panel, so no handle is threaded through the API        */
/* -------------------------------------------------------------------------- */

static ili9341_hw_t s_hw;
static uint16_t     s_width  = ILI9341_NATIVE_WIDTH;
static uint16_t     s_height = ILI9341_NATIVE_HEIGHT;
static bool         s_ready  = false;

/* -------------------------------------------------------------------------- */
/*  5x7 font, ASCII 0x20..0x7E                                                */
/*                                                                            */
/*  Column-major: five bytes per glyph, one per column, bit 0 = top row. The  */
/*  sixth column of the 6x8 cell is the inter-character gutter and the eighth */
/*  row the line gutter; both are emitted as background.                      */
/* -------------------------------------------------------------------------- */

#define FONT_FIRST_CHAR  0x20u
#define FONT_LAST_CHAR   0x7Eu

static const uint8_t s_font[(FONT_LAST_CHAR - FONT_FIRST_CHAR + 1u) * 5u] = {
    0x00,0x00,0x00,0x00,0x00, /*   */  0x00,0x00,0x5F,0x00,0x00, /* ! */
    0x00,0x07,0x00,0x07,0x00, /* " */  0x14,0x7F,0x14,0x7F,0x14, /* # */
    0x24,0x2A,0x7F,0x2A,0x12, /* $ */  0x23,0x13,0x08,0x64,0x62, /* % */
    0x36,0x49,0x55,0x22,0x50, /* & */  0x00,0x05,0x03,0x00,0x00, /* ' */
    0x00,0x1C,0x22,0x41,0x00, /* ( */  0x00,0x41,0x22,0x1C,0x00, /* ) */
    0x14,0x08,0x3E,0x08,0x14, /* * */  0x08,0x08,0x3E,0x08,0x08, /* + */
    0x00,0x50,0x30,0x00,0x00, /* , */  0x08,0x08,0x08,0x08,0x08, /* - */
    0x00,0x60,0x60,0x00,0x00, /* . */  0x20,0x10,0x08,0x04,0x02, /* / */
    0x3E,0x51,0x49,0x45,0x3E, /* 0 */  0x00,0x42,0x7F,0x40,0x00, /* 1 */
    0x42,0x61,0x51,0x49,0x46, /* 2 */  0x21,0x41,0x45,0x4B,0x31, /* 3 */
    0x18,0x14,0x12,0x7F,0x10, /* 4 */  0x27,0x45,0x45,0x45,0x39, /* 5 */
    0x3C,0x4A,0x49,0x49,0x30, /* 6 */  0x01,0x71,0x09,0x05,0x03, /* 7 */
    0x36,0x49,0x49,0x49,0x36, /* 8 */  0x06,0x49,0x49,0x29,0x1E, /* 9 */
    0x00,0x36,0x36,0x00,0x00, /* : */  0x00,0x56,0x36,0x00,0x00, /* ; */
    0x08,0x14,0x22,0x41,0x00, /* < */  0x14,0x14,0x14,0x14,0x14, /* = */
    0x00,0x41,0x22,0x14,0x08, /* > */  0x02,0x01,0x51,0x09,0x06, /* ? */
    0x32,0x49,0x79,0x41,0x3E, /* @ */  0x7E,0x11,0x11,0x11,0x7E, /* A */
    0x7F,0x49,0x49,0x49,0x36, /* B */  0x3E,0x41,0x41,0x41,0x22, /* C */
    0x7F,0x41,0x41,0x22,0x1C, /* D */  0x7F,0x49,0x49,0x49,0x41, /* E */
    0x7F,0x09,0x09,0x09,0x01, /* F */  0x3E,0x41,0x49,0x49,0x7A, /* G */
    0x7F,0x08,0x08,0x08,0x7F, /* H */  0x00,0x41,0x7F,0x41,0x00, /* I */
    0x20,0x40,0x41,0x3F,0x01, /* J */  0x7F,0x08,0x14,0x22,0x41, /* K */
    0x7F,0x40,0x40,0x40,0x40, /* L */  0x7F,0x02,0x0C,0x02,0x7F, /* M */
    0x7F,0x04,0x08,0x10,0x7F, /* N */  0x3E,0x41,0x41,0x41,0x3E, /* O */
    0x7F,0x09,0x09,0x09,0x06, /* P */  0x3E,0x41,0x51,0x21,0x5E, /* Q */
    0x7F,0x09,0x19,0x29,0x46, /* R */  0x46,0x49,0x49,0x49,0x31, /* S */
    0x01,0x01,0x7F,0x01,0x01, /* T */  0x3F,0x40,0x40,0x40,0x3F, /* U */
    0x1F,0x20,0x40,0x20,0x1F, /* V */  0x3F,0x40,0x38,0x40,0x3F, /* W */
    0x63,0x14,0x08,0x14,0x63, /* X */  0x07,0x08,0x70,0x08,0x07, /* Y */
    0x61,0x51,0x49,0x45,0x43, /* Z */  0x00,0x7F,0x41,0x41,0x00, /* [ */
    0x02,0x04,0x08,0x10,0x20, /* \ */  0x00,0x41,0x41,0x7F,0x00, /* ] */
    0x04,0x02,0x01,0x02,0x04, /* ^ */  0x40,0x40,0x40,0x40,0x40, /* _ */
    0x00,0x01,0x02,0x04,0x00, /* ` */  0x20,0x54,0x54,0x54,0x78, /* a */
    0x7F,0x48,0x44,0x44,0x38, /* b */  0x38,0x44,0x44,0x44,0x20, /* c */
    0x38,0x44,0x44,0x48,0x7F, /* d */  0x38,0x54,0x54,0x54,0x18, /* e */
    0x08,0x7E,0x09,0x01,0x02, /* f */  0x0C,0x52,0x52,0x52,0x3E, /* g */
    0x7F,0x08,0x04,0x04,0x78, /* h */  0x00,0x44,0x7D,0x40,0x00, /* i */
    0x20,0x40,0x44,0x3D,0x00, /* j */  0x7F,0x10,0x28,0x44,0x00, /* k */
    0x00,0x41,0x7F,0x40,0x00, /* l */  0x7C,0x04,0x18,0x04,0x78, /* m */
    0x7C,0x08,0x04,0x04,0x78, /* n */  0x38,0x44,0x44,0x44,0x38, /* o */
    0x7C,0x14,0x14,0x14,0x08, /* p */  0x08,0x14,0x14,0x18,0x7C, /* q */
    0x7C,0x08,0x04,0x04,0x08, /* r */  0x48,0x54,0x54,0x54,0x20, /* s */
    0x04,0x3F,0x44,0x40,0x20, /* t */  0x3C,0x40,0x40,0x20,0x7C, /* u */
    0x1C,0x20,0x40,0x20,0x1C, /* v */  0x3C,0x40,0x30,0x40,0x3C, /* w */
    0x44,0x28,0x10,0x28,0x44, /* x */  0x0C,0x50,0x50,0x50,0x3C, /* y */
    0x44,0x64,0x54,0x4C,0x44, /* z */  0x00,0x08,0x36,0x41,0x00, /* { */
    0x00,0x00,0x7F,0x00,0x00, /* | */  0x00,0x41,0x36,0x08,0x00, /* } */
    0x08,0x04,0x08,0x10,0x08, /* ~ */
};

/* -------------------------------------------------------------------------- */
/*  Pin and bus helpers                                                       */
/* -------------------------------------------------------------------------- */

static inline void cs_low(void)   { gpio_clear(s_hw.cs_port, s_hw.cs_pin); }
static inline void cs_high(void)  { gpio_set(s_hw.cs_port, s_hw.cs_pin);   }
static inline void dc_command(void) { gpio_clear(s_hw.dc_port, s_hw.dc_pin); }
static inline void dc_data(void)    { gpio_set(s_hw.dc_port, s_hw.dc_pin);   }

/** Send one opcode plus its parameters, framed by CS. */
static void send_command(uint8_t cmd, const uint8_t *args, size_t n)
{
    cs_low();

    dc_command();
    (void)spi_write(s_hw.spi, &cmd, 1u);

    if (n != 0u) {
        dc_data();
        (void)spi_write(s_hw.spi, args, n);
    }

    cs_high();
}

/** Send one opcode with CS already asserted by the caller. */
static void send_command_open(uint8_t cmd, const uint8_t *args, size_t n)
{
    dc_command();
    (void)spi_write(s_hw.spi, &cmd, 1u);

    if (n != 0u) {
        dc_data();
        (void)spi_write(s_hw.spi, args, n);
    }
}

static uint8_t madctl_for(ili9341_rotation_t rot)
{
    switch (rot) {
    case ILI9341_ROT_90:  return (uint8_t)(MADCTL_MV | MADCTL_BGR);
    case ILI9341_ROT_180: return (uint8_t)(MADCTL_MY | MADCTL_BGR);
    case ILI9341_ROT_270: return (uint8_t)(MADCTL_MX | MADCTL_MY |
                                           MADCTL_MV | MADCTL_BGR);
    case ILI9341_ROT_0:
    default:              return (uint8_t)(MADCTL_MX | MADCTL_BGR);
    }
}

/* -------------------------------------------------------------------------- */
/*  Lifecycle                                                                 */
/* -------------------------------------------------------------------------- */

/* The panel-specific bring-up. Power, VCOM and gamma values are the vendor
 * reference set for this glass - they are not derived from anything and are
 * not worth "cleaning up". Order matters; the delays are datasheet minima. */
static void run_init_script(void)
{
    static const uint8_t init_ef[]      = { 0x03u, 0x80u, 0x02u };
    static const uint8_t init_cf[]      = { 0x00u, 0xC1u, 0x30u };
    static const uint8_t init_ed[]      = { 0x64u, 0x03u, 0x12u, 0x81u };
    static const uint8_t init_e8[]      = { 0x85u, 0x00u, 0x78u };
    static const uint8_t init_cb[]      = { 0x39u, 0x2Cu, 0x00u, 0x34u, 0x02u };
    static const uint8_t init_f7[]      = { 0x20u };
    static const uint8_t init_ea[]      = { 0x00u, 0x00u };
    static const uint8_t pwctr1[]       = { 0x23u };
    static const uint8_t pwctr2[]       = { 0x10u };
    static const uint8_t vmctr1[]       = { 0x3Eu, 0x28u };
    static const uint8_t vmctr2[]       = { 0x86u };
    static const uint8_t pixfmt[]       = { 0x55u };  /* 16 bit/px, RGB565   */
    static const uint8_t frmctr1[]      = { 0x00u, 0x18u };
    static const uint8_t dfunctr[]      = { 0x08u, 0x82u, 0x27u };
    static const uint8_t enable3g[]     = { 0x00u };
    static const uint8_t gammaset[]     = { 0x01u };
    static const uint8_t gamma_pos[]    = {
        0x0Fu, 0x31u, 0x2Bu, 0x0Cu, 0x0Eu, 0x08u, 0x4Eu, 0xF1u,
        0x37u, 0x07u, 0x10u, 0x03u, 0x0Eu, 0x09u, 0x00u };
    static const uint8_t gamma_neg[]    = {
        0x00u, 0x0Eu, 0x14u, 0x03u, 0x11u, 0x07u, 0x31u, 0xC1u,
        0x48u, 0x08u, 0x0Fu, 0x0Cu, 0x31u, 0x36u, 0x0Fu };

    send_command(0xEFu, init_ef,  sizeof init_ef);
    send_command(0xCFu, init_cf,  sizeof init_cf);
    send_command(0xEDu, init_ed,  sizeof init_ed);
    send_command(0xE8u, init_e8,  sizeof init_e8);
    send_command(0xCBu, init_cb,  sizeof init_cb);
    send_command(0xF7u, init_f7,  sizeof init_f7);
    send_command(0xEAu, init_ea,  sizeof init_ea);

    send_command(CMD_PWCTR1,  pwctr1,  sizeof pwctr1);
    send_command(CMD_PWCTR2,  pwctr2,  sizeof pwctr2);
    send_command(CMD_VMCTR1,  vmctr1,  sizeof vmctr1);
    send_command(CMD_VMCTR2,  vmctr2,  sizeof vmctr2);
    send_command(CMD_PIXFMT,  pixfmt,  sizeof pixfmt);
    send_command(CMD_FRMCTR1, frmctr1, sizeof frmctr1);
    send_command(CMD_DFUNCTR, dfunctr, sizeof dfunctr);

    send_command(0xF2u,        enable3g, sizeof enable3g);
    send_command(CMD_GAMMASET, gammaset, sizeof gammaset);
    send_command(CMD_GMCTRP1,  gamma_pos, sizeof gamma_pos);
    send_command(CMD_GMCTRN1,  gamma_neg, sizeof gamma_neg);

    send_command(CMD_SLPOUT, NULL, 0u);
    systick_delay_ms(120u);          /* mandatory before any further command */

    send_command(CMD_DISPON, NULL, 0u);
    systick_delay_ms(20u);
}

drv_status_t ili9341_init(const ili9341_hw_t *hw)
{
    if (hw == NULL || hw->spi == NULL ||
        hw->cs_port == NULL || hw->dc_port == NULL) {
        return DRV_INVALID_PARAM;
    }

    s_hw = *hw;

    /* Idle levels first, so nothing is half-selected while the panel powers
     * up: CS released, D/C in the command state. */
    (void)gpio_init_output(s_hw.cs_port, s_hw.cs_pin, GPIO_OTYPE_PP, GPIO_HIGH);
    (void)gpio_init_output(s_hw.dc_port, s_hw.dc_pin, GPIO_OTYPE_PP, GPIO_LOW);

    if (s_hw.rst_port != NULL) {
        (void)gpio_init_output(s_hw.rst_port, s_hw.rst_pin,
                               GPIO_OTYPE_PP, GPIO_HIGH);

        /* Hardware reset. The datasheet wants >=10 us low; 5 ms costs nothing
         * once at boot and survives a slow-rising supply rail. */
        systick_delay_ms(5u);
        gpio_clear(s_hw.rst_port, s_hw.rst_pin);
        systick_delay_ms(20u);
        gpio_set(s_hw.rst_port, s_hw.rst_pin);
        systick_delay_ms(150u);
    } else {
        /* No reset pin available - fall back to the software reset, which
         * needs the same settling time. */
        send_command(CMD_SWRESET, NULL, 0u);
        systick_delay_ms(150u);
    }

    s_ready = true;

    run_init_script();
    ili9341_set_rotation(s_hw.rotation);

    return DRV_OK;
}

void ili9341_set_rotation(ili9341_rotation_t rotation)
{
    if (!s_ready) {
        return;
    }

    const uint8_t madctl = madctl_for(rotation);

    s_hw.rotation = rotation;
    send_command(CMD_MADCTL, &madctl, 1u);

    if (rotation == ILI9341_ROT_90 || rotation == ILI9341_ROT_270) {
        s_width  = ILI9341_NATIVE_HEIGHT;
        s_height = ILI9341_NATIVE_WIDTH;
    } else {
        s_width  = ILI9341_NATIVE_WIDTH;
        s_height = ILI9341_NATIVE_HEIGHT;
    }
}

uint16_t ili9341_width(void)  { return s_width;  }
uint16_t ili9341_height(void) { return s_height; }

void ili9341_display_on(bool on)
{
    if (s_ready) {
        send_command(on ? CMD_DISPON : CMD_DISPOFF, NULL, 0u);
    }
}

void ili9341_invert(bool on)
{
    if (s_ready) {
        send_command(on ? CMD_INVON : CMD_INVOFF, NULL, 0u);
    }
}

/* -------------------------------------------------------------------------- */
/*  Window and pixel streaming                                                */
/* -------------------------------------------------------------------------- */

void ili9341_begin_write(void)
{
    if (s_ready) {
        cs_low();
    }
}

void ili9341_end_write(void)
{
    if (s_ready) {
        cs_high();
    }
}

void ili9341_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    if (!s_ready || w == 0u || h == 0u) {
        return;
    }

    const uint16_t x1 = (uint16_t)(x + w - 1u);
    const uint16_t y1 = (uint16_t)(y + h - 1u);

    const uint8_t caset[4] = { (uint8_t)(x  >> 8), (uint8_t)(x  & 0xFFu),
                               (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFFu) };
    const uint8_t paset[4] = { (uint8_t)(y  >> 8), (uint8_t)(y  & 0xFFu),
                               (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFFu) };

    send_command_open(CMD_CASET, caset, sizeof caset);
    send_command_open(CMD_PASET, paset, sizeof paset);
    send_command_open(CMD_RAMWR, NULL, 0u);

    /* Leave D/C high: everything that follows is pixel data. */
    dc_data();
}

void ili9341_push_color(uint16_t color, uint32_t count)
{
    if (!s_ready || count == 0u) {
        return;
    }

    uint8_t chunk[CHUNK_PIXELS * 2u];

    for (size_t i = 0; i < CHUNK_PIXELS; ++i) {
        chunk[i * 2u]      = (uint8_t)(color >> 8);
        chunk[i * 2u + 1u] = (uint8_t)(color & 0xFFu);
    }

    dc_data();

    while (count != 0u) {
        const uint32_t n = (count > CHUNK_PIXELS) ? CHUNK_PIXELS : count;
        (void)spi_write(s_hw.spi, chunk, (size_t)(n * 2u));
        count -= n;
    }
}

void ili9341_push_pixels(const uint16_t *px, uint32_t count)
{
    if (!s_ready || px == NULL || count == 0u) {
        return;
    }

    uint8_t chunk[CHUNK_PIXELS * 2u];

    dc_data();

    while (count != 0u) {
        const uint32_t n = (count > CHUNK_PIXELS) ? CHUNK_PIXELS : count;

        /* The panel takes the high byte first; the array is native-endian. */
        for (uint32_t i = 0; i < n; ++i) {
            chunk[i * 2u]      = (uint8_t)(px[i] >> 8);
            chunk[i * 2u + 1u] = (uint8_t)(px[i] & 0xFFu);
        }

        (void)spi_write(s_hw.spi, chunk, (size_t)(n * 2u));

        px    += n;
        count -= n;
    }
}

/* -------------------------------------------------------------------------- */
/*  Primitives                                                                */
/* -------------------------------------------------------------------------- */

/**
 * Clip a rectangle to the screen. Returns false if nothing is left to draw.
 * Coordinates are unsigned, so a negative origin is not expressible - callers
 * that compute positions must not underflow.
 */
static bool clip_rect(uint16_t *x, uint16_t *y, uint16_t *w, uint16_t *h)
{
    if (*x >= s_width || *y >= s_height || *w == 0u || *h == 0u) {
        return false;
    }
    if ((uint32_t)*x + *w > s_width)  { *w = (uint16_t)(s_width  - *x); }
    if ((uint32_t)*y + *h > s_height) { *h = (uint16_t)(s_height - *y); }

    return true;
}

void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       uint16_t color)
{
    if (!s_ready || !clip_rect(&x, &y, &w, &h)) {
        return;
    }

    ili9341_begin_write();
    ili9341_set_window(x, y, w, h);
    ili9341_push_color(color, (uint32_t)w * h);
    ili9341_end_write();
}

void ili9341_fill_screen(uint16_t color)
{
    ili9341_fill_rect(0u, 0u, s_width, s_height, color);
}

void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    ili9341_fill_rect(x, y, 1u, 1u, color);
}

void ili9341_draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
    ili9341_fill_rect(x, y, w, 1u, color);
}

void ili9341_draw_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
    ili9341_fill_rect(x, y, 1u, h, color);
}

void ili9341_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       uint16_t color)
{
    if (w == 0u || h == 0u) {
        return;
    }

    ili9341_draw_hline(x, y, w, color);
    ili9341_draw_hline(x, (uint16_t)(y + h - 1u), w, color);
    ili9341_draw_vline(x, y, h, color);
    ili9341_draw_vline((uint16_t)(x + w - 1u), y, h, color);
}

void ili9341_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         const uint16_t *px)
{
    if (!s_ready || px == NULL || w == 0u || h == 0u) {
        return;
    }
    /* Clipping a bitmap means walking it row by row; the telemetry screen has
     * no reason to draw one partly off-screen, so reject instead of guessing. */
    if ((uint32_t)x + w > s_width || (uint32_t)y + h > s_height) {
        return;
    }

    ili9341_begin_write();
    ili9341_set_window(x, y, w, h);
    ili9341_push_pixels(px, (uint32_t)w * h);
    ili9341_end_write();
}

/* -------------------------------------------------------------------------- */
/*  Text                                                                      */
/* -------------------------------------------------------------------------- */

/** The five column bitmaps for @p ch, or a solid block for anything unmapped. */
static const uint8_t *glyph_for(char ch)
{
    static const uint8_t solid[5] = { 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu };

    const uint8_t c = (uint8_t)ch;

    if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) {
        return solid;
    }
    return &s_font[(size_t)(c - FONT_FIRST_CHAR) * 5u];
}

void ili9341_draw_char(uint16_t x, uint16_t y, char ch, uint8_t scale,
                       uint16_t fg, uint16_t bg)
{
    if (!s_ready) {
        return;
    }
    if (scale == 0u) { scale = 1u; }
    if (scale > 8u)  { scale = 8u; }

    const uint16_t cell_w = (uint16_t)(ILI9341_FONT_W * scale);
    const uint16_t cell_h = (uint16_t)(ILI9341_FONT_H * scale);

    /* Whole-cell clipping only. Partially visible glyphs are a layout bug, and
     * silently drawing half of one hides it. */
    if ((uint32_t)x + cell_w > s_width || (uint32_t)y + cell_h > s_height) {
        return;
    }

    const uint8_t *cols = glyph_for(ch);

    /* One expanded row of the cell, reused for all `scale` repeats of it.
     * 6 columns * 8x magnification = 48 pixels, 96 bytes on the stack. */
    uint16_t row_px[ILI9341_FONT_W * 8u];

    /* One window for the whole cell, then a single top-to-bottom, left-to-
     * right pixel stream - the glyph's background is part of that stream, so
     * there is no separate erase and therefore no flicker. */
    ili9341_begin_write();
    ili9341_set_window(x, y, cell_w, cell_h);

    for (uint8_t row = 0; row < ILI9341_FONT_H; ++row) {
        /* Row 7 is the line gutter: always background. */
        const bool gutter_row = (row >= 7u);
        uint16_t   n          = 0u;

        for (uint8_t col = 0; col < ILI9341_FONT_W; ++col) {
            /* Column 5 is the inter-character gutter. */
            const bool on = (!gutter_row && col < 5u) &&
                            (((cols[col] >> row) & 0x01u) != 0u);
            const uint16_t c = on ? fg : bg;

            for (uint8_t rep = 0; rep < scale; ++rep) {
                row_px[n++] = c;
            }
        }

        for (uint8_t rep = 0; rep < scale; ++rep) {
            ili9341_push_pixels(row_px, n);
        }
    }

    ili9341_end_write();
}

uint16_t ili9341_draw_text(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                           uint16_t fg, uint16_t bg)
{
    if (s == NULL) {
        return x;
    }
    if (scale == 0u) { scale = 1u; }
    if (scale > 8u)  { scale = 8u; }

    const uint16_t advance = (uint16_t)(ILI9341_FONT_W * scale);

    while (*s != '\0') {
        if ((uint32_t)x + advance > s_width) {
            break;                      /* clip at the edge, do not wrap */
        }
        ili9341_draw_char(x, y, *s, scale, fg, bg);
        x = (uint16_t)(x + advance);
        ++s;
    }

    return x;
}

uint16_t ili9341_text_width(const char *s, uint8_t scale)
{
    if (s == NULL) {
        return 0u;
    }
    if (scale == 0u) { scale = 1u; }
    if (scale > 8u)  { scale = 8u; }

    return (uint16_t)(strlen(s) * ILI9341_FONT_W * scale);
}

void ili9341_draw_field(uint16_t x, uint16_t y, uint8_t cells, uint8_t scale,
                        const char *s, bool right_align,
                        uint16_t fg, uint16_t bg)
{
    if (!s_ready || cells == 0u) {
        return;
    }
    if (s == NULL) { s = ""; }
    if (scale == 0u) { scale = 1u; }
    if (scale > 8u)  { scale = 8u; }

    size_t len = strlen(s);
    if (len > cells) {
        len = cells;                    /* truncate rather than overflow */
    }

    const uint16_t advance = (uint16_t)(ILI9341_FONT_W * scale);
    const size_t   pad     = (size_t)cells - len;

    /* Every cell is painted every call - the padding is what guarantees a
     * shrinking value cannot leave a stale digit behind. */
    for (size_t i = 0; i < cells; ++i) {
        char ch;

        if (right_align) {
            ch = (i < pad) ? ' ' : s[i - pad];
        } else {
            ch = (i < len) ? s[i] : ' ';
        }

        ili9341_draw_char((uint16_t)(x + i * advance), y, ch, scale, fg, bg);
    }
}
