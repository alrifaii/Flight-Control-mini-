#ifndef OLEDC_H
#define OLEDC_H

#include "stm32l4xx_hal.h"

// Display size (shared constant so other modules can use it)
#define OLED_WIDTH 128
#define OLED_HEIGHT 96

// ===================================================================================
// SSD1351 Controller-Befehle
// ===================================================================================
#define SSD1351_CMD_SET_COLUMN_ADDR     0x15
#define SSD1351_CMD_SET_ROW_ADDR        0x75
#define SSD1351_CMD_WRITE_RAM           0x5C
#define SSD1351_CMD_SET_DISPLAY_ON      0xAF
#define SSD1351_CMD_SET_DISPLAY_OFF     0xAE

// ===================================================================================
// Funktions-Prototypen
// ===================================================================================

/**
 * @brief Initialisiert das OLED-Display.
 */
void OLED_Init(void);

/**
 * @brief Führt einen Hardware-Reset des Displays durch.
 */
void OLED_Reset(void);

/**
 * @brief Löscht den gesamten Bildschirm (füllt ihn mit Schwarz).
 */
void OLED_Clear(void);

/**
 * @brief Füllt den gesamten Bildschirm mit einer angegebenen 16-Bit-Farbe.
 * @param color Die 16-Bit (RGB565) Farbe.
 */
void OLED_DisplayColor(uint16_t color);

/**
 * @brief Zeichnet ein vollflächiges Bitmap-Buffer an eine Position.
 * @param bitmap Pointer auf ein RGB565-Buffer (2 bytes per pixel)
 * @param bmp_w Bildbreite in Pixel
 * @param bmp_h Bildhöhe in Pixel
 * @param x Ziel-X-Koordinate (links oben)
 * @param y Ziel-Y-Koordinate (links oben)
 */
void OLED_DrawBitmap(const uint8_t* bitmap, uint16_t bmp_w, uint16_t bmp_h, uint16_t x, uint16_t y);

/**
 * @brief Komfort-Funktion: Zeige das im Projekt vorhandene Compass-Bitmap
 *        (wenn vorhanden als `compass_mod_map` in Core/Src/compass_image.c)
 *        an der Display-Position 0,0. Bildgröße wird standardmäßig auf
 *        OLED_WIDTH x OLED_HEIGHT angenommen (überschreibt zufällige
 *        Bildgrößen nicht automatisch).
 */
void OLED_ShowCompassImage(void);
/**
 * @brief Show the compiled-in compass bitmap at a custom position.
 * @param x Left coordinate (0..OLED_WIDTH-1)
 * @param y Top coordinate (0..OLED_HEIGHT-1)
 */
void OLED_ShowCompassImageAt(uint16_t x, uint16_t y);

/**
 * @brief Sendet einen Befehl (Command) an den OLED-Controller.
 * @param command Das zu sendende Befehls-Byte.
 */
void OLED_WriteCommand(uint8_t command);

/**
 * @brief Sendet ein Daten-Byte an den OLED-Controller.
 * @param data Das zu sendende Daten-Byte.
 */
void OLED_WriteData(uint8_t data);

// Low-level drawing primitives useful for overlays
void OLED_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
/**
 * @brief Draw a decimal number using 6x8 font scaled by an integer factor.
 *        scale=1 draws normal size, scale=2 doubles width/height, etc.
 */
void OLED_DrawNumberScaled(uint16_t x, uint16_t y, int value, uint16_t color, uint8_t scale);

/**
 * @brief Draw compass with heading in degrees (0-360).
 *        Shows background image, heading number, and red needle.
 * @param heading_deg Compass heading in degrees (float, 0.0-360.0)
 */
void OLED_DrawCompass(float heading_deg);

// Externe Benennung von Bilddaten (wenn z.B. compass_image.c ein Array bereitstellt)
extern const uint8_t compass_mod_map[]; // LVGL-converted bitmap (raw RGB565 bytes)
extern const uint16_t compass_mod_map_w; // pixel width
extern const uint16_t compass_mod_map_h; // pixel height
extern const uint32_t compass_mod_map_size; // byte size

#endif // OLEDC_H
