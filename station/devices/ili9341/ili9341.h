/**
 ******************************************************************************
 * @file    ili9341.h
 * @brief   ILI9341 240x320 TFT driver for the control station (STM32F446RE).
 *
 * Sits between the register-level station drivers (spi, gpio, systick) and the
 * application. Nothing below this file knows what a pixel is; nothing above it
 * touches a peripheral register.
 *
 * The panel is driven over 4-wire SPI: every byte on MOSI is qualified by the
 * out-of-band D/C pin (low = command opcode, high = parameter or pixel data).
 * There is no MCU-side framebuffer - a 240x320 RGB565 image is 150 KB and the
 * F446RE has 128 KB of SRAM - so all drawing is immediate-mode: address a
 * rectangle in the panel's own GRAM, then stream pixels into it.
 *
 * CS framing is explicit. The composite primitives (fill_rect, draw_text, ...)
 * open and close it for you; the low-level trio
 *
 *     ili9341_set_window() / ili9341_push_color() / ili9341_push_pixels()
 *
 * must be bracketed by ili9341_begin_write() and ili9341_end_write() by the
 * caller. Raising CS mid-transfer resets the panel's interface state machine
 * and abandons the pixel stream, so a window and its data belong in one
 * bracket.
 *
 * The bus is not shared: the nRF24 lives on its own SPI instance (see
 * app_config.h), so a long pixel burst here cannot stall the radio.
 ******************************************************************************
 */
#ifndef STATION_DEVICES_ILI9341_H
#define STATION_DEVICES_ILI9341_H

#include "drivers_common.h"
#include "gpio.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  Panel geometry                                                            */
/* -------------------------------------------------------------------------- */

/** @brief Native panel size, before rotation. */
#define ILI9341_NATIVE_WIDTH    240u
#define ILI9341_NATIVE_HEIGHT   320u

/** @brief Built-in font cell. The glyphs are 5x7 in a 6x8 cell (1 px gutter). */
#define ILI9341_FONT_W          6u
#define ILI9341_FONT_H          8u

/** @brief Display orientation. 90/270 give a 320x240 landscape dashboard. */
typedef enum {
    ILI9341_ROT_0 = 0,   /**< portrait,  240x320, ribbon at the bottom.     */
    ILI9341_ROT_90,      /**< landscape, 320x240.                           */
    ILI9341_ROT_180,     /**< portrait,  240x320, inverted.                 */
    ILI9341_ROT_270,     /**< landscape, 320x240, inverted.                 */
} ili9341_rotation_t;

/* -------------------------------------------------------------------------- */
/*  Colour                                                                    */
/* -------------------------------------------------------------------------- */

/** @brief Pack 8-bit RGB into the panel's RGB565 pixel format. */
#define ILI9341_COLOR(r, g, b)                       \
    ((uint16_t)((((uint16_t)(r) & 0xF8u) << 8) |     \
                (((uint16_t)(g) & 0xFCu) << 3) |     \
                (((uint16_t)(b) & 0xF8u) >> 3)))

#define ILI9341_BLACK    ILI9341_COLOR(0,   0,   0)
#define ILI9341_WHITE    ILI9341_COLOR(255, 255, 255)
#define ILI9341_RED      ILI9341_COLOR(255, 0,   0)
#define ILI9341_GREEN    ILI9341_COLOR(0,   255, 0)
#define ILI9341_BLUE     ILI9341_COLOR(0,   0,   255)
#define ILI9341_YELLOW   ILI9341_COLOR(255, 255, 0)
#define ILI9341_ORANGE   ILI9341_COLOR(255, 140, 0)
#define ILI9341_CYAN     ILI9341_COLOR(0,   255, 255)
#define ILI9341_MAGENTA  ILI9341_COLOR(255, 0,   255)
#define ILI9341_GREY     ILI9341_COLOR(128, 128, 128)
#define ILI9341_DARKGREY ILI9341_COLOR(64,  64,  64)

/* -------------------------------------------------------------------------- */
/*  Hardware binding                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Which SPI instance and which control pins the panel is wired to.
 *
 * The caller configures the SCK/MOSI pins for their alternate function and
 * calls spi_init() before ili9341_init(); this driver owns only CS, D/C and
 * RESET, which it configures itself. MISO is not used - the panel is treated
 * as write-only, since many breakout boards leave SDO unconnected.
 */
typedef struct {
    SPI_TypeDef  *spi;        /**< Already initialised, mode 0, MSB first.  */

    GPIO_TypeDef *cs_port;    /**< Chip select (active low).                */
    uint8_t       cs_pin;

    GPIO_TypeDef *dc_port;    /**< Data/command: low = command.             */
    uint8_t       dc_pin;

    GPIO_TypeDef *rst_port;   /**< Hardware reset, or NULL if tied to the   */
    uint8_t       rst_pin;    /**<   board reset line.                      */

    ili9341_rotation_t rotation;
} ili9341_hw_t;

/* -------------------------------------------------------------------------- */
/*  Lifecycle                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configure the control pins, reset the panel and run its init script.
 * @param hw Wiring description (must not be NULL, @p hw->spi must not be NULL).
 * @return DRV_OK, or DRV_INVALID_PARAM.
 *
 * Blocks for roughly 300 ms: the power-on state of the controller is undefined,
 * and both the hardware reset and SLPOUT need their datasheet settling times.
 * Call once, after spi_init() and systick_init(). @p hw is copied.
 *
 * The panel cannot be probed - with SDO unwired there is no read-back - so a
 * DRV_OK here means "the sequence was sent", not "the display is alive". The
 * first fill_screen is the real test.
 */
drv_status_t ili9341_init(const ili9341_hw_t *hw);

/** @brief Change orientation. Does not repaint - the caller redraws. */
void ili9341_set_rotation(ili9341_rotation_t rotation);

/** @brief Visible width in pixels, after rotation. */
uint16_t ili9341_width(void);
/** @brief Visible height in pixels, after rotation. */
uint16_t ili9341_height(void);

/** @brief Turn the display output on or off (DISPON / DISPOFF). */
void ili9341_display_on(bool on);

/** @brief Invert the panel's colours (INVON / INVOFF). Handy as an alarm. */
void ili9341_invert(bool on);

/* -------------------------------------------------------------------------- */
/*  Low-level: address a window, stream pixels                                */
/*                                                                            */
/*  Every one of these must sit between begin_write() and end_write().        */
/* -------------------------------------------------------------------------- */

/** @brief Assert CS. Opens a transaction. */
void ili9341_begin_write(void);

/** @brief Release CS. Closes a transaction. */
void ili9341_end_write(void);

/**
 * @brief Set the GRAM write window and issue RAMWR.
 * @param x,y Top-left corner, in the current rotation's coordinates.
 * @param w,h Size in pixels. Must be non-zero.
 *
 * After this the panel expects a pixel stream; its internal pointer walks the
 * window left-to-right, top-to-bottom and wraps at the end. Push exactly w*h
 * pixels, or fewer if you only mean to touch part of it.
 */
void ili9341_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/** @brief Stream one colour @p count times into the open window. */
void ili9341_push_color(uint16_t color, uint32_t count);

/**
 * @brief Stream @p count pixels from @p px into the open window.
 * @param px Native-endian RGB565 values; byte order is fixed up on the way out.
 */
void ili9341_push_pixels(const uint16_t *px, uint32_t count);

/* -------------------------------------------------------------------------- */
/*  Primitives                                                                */
/*                                                                            */
/*  These handle their own CS bracketing. All of them clip to the screen.     */
/* -------------------------------------------------------------------------- */

/** @brief Fill the whole screen with one colour. */
void ili9341_fill_screen(uint16_t color);

/** @brief Fill a rectangle. The workhorse - everything else builds on it. */
void ili9341_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       uint16_t color);

/** @brief Set a single pixel (a 1x1 window - cheap once, costly in a loop). */
void ili9341_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/** @brief Horizontal line of length @p w. */
void ili9341_draw_hline(uint16_t x, uint16_t y, uint16_t w, uint16_t color);

/** @brief Vertical line of length @p h. */
void ili9341_draw_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t color);

/** @brief One-pixel rectangle outline. */
void ili9341_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                       uint16_t color);

/**
 * @brief Blit a pre-rendered RGB565 image stored in flash.
 * @param px Row-major, @p w * @p h entries.
 */
void ili9341_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                         const uint16_t *px);

/* -------------------------------------------------------------------------- */
/*  Text                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Draw one character from the built-in 5x7 font.
 * @param x,y   Top-left of the character cell.
 * @param ch    ASCII 0x20..0x7E; anything else renders as a filled block.
 * @param scale Integer magnification, 1..8. The cell becomes 6*scale x 8*scale.
 * @param fg,bg Foreground and background colour.
 *
 * The background is emitted as part of the same pixel stream, so a character
 * overwrites whatever was underneath it. There is no transparent mode and no
 * erase pass - that is what stops text flickering.
 */
void ili9341_draw_char(uint16_t x, uint16_t y, char ch, uint8_t scale,
                       uint16_t fg, uint16_t bg);

/**
 * @brief Draw a NUL-terminated string. No wrapping - it clips at the edge.
 * @return The x coordinate just past the last character drawn.
 */
uint16_t ili9341_draw_text(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                           uint16_t fg, uint16_t bg);

/** @brief Pixel width of @p s at @p scale, for laying out static labels. */
uint16_t ili9341_text_width(const char *s, uint8_t scale);

/**
 * @brief Draw @p s into a fixed-width cell, padding the rest with @p bg.
 * @param x,y        Top-left of the cell.
 * @param cells      Cell width in characters. Reserve the widest value a field
 *                   can ever take, e.g. 5 for "-1234".
 * @param scale      Integer magnification.
 * @param s          Text; anything past @p cells characters is truncated.
 * @param right_align true to pad on the left (use for numbers), false for text.
 *
 * This is the primitive a telemetry screen is actually built from: because the
 * full cell is repainted every time, a value shrinking from "1240" to "380"
 * cannot leave a stale digit behind, and no separate erase is needed.
 */
void ili9341_draw_field(uint16_t x, uint16_t y, uint8_t cells, uint8_t scale,
                        const char *s, bool right_align,
                        uint16_t fg, uint16_t bg);

#ifdef __cplusplus
}
#endif

#endif /* STATION_DEVICES_ILI9341_H */
