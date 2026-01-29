#include "oledc.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

// Internal state variables
static int compass_heading_centi = -1;
static int compass_x = 64;
static int compass_y = 48;
static int last_drawn_heading = -1;
// Kleine 5x7 Buchstaben für den Kompass (N, E, S, W)
static const uint8_t char_N[6] = {0x7F, 0x02, 0x04, 0x08, 0x7F, 0x00}; 
static const uint8_t char_E[6] = {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00}; 
static const uint8_t char_S[6] = {0x46, 0x49, 0x49, 0x49, 0x31, 0x00}; 
static const uint8_t char_W[6] = {0x7F, 0x20, 0x18, 0x20, 0x7F, 0x00};
// Public API overview (see oledc.h for signatures)
// Init/Reset: OLED_Init, OLED_Reset
// Low-level I/O: OLED_WriteCommand, OLED_WriteData
// Primitives: OLED_DisplayColor, OLED_Clear, OLED_DrawPixel, OLED_FillRun
// Numbers: OLED_DrawNumber, OLED_DrawNumberScaled
// Bitmaps: OLED_DrawBitmap, OLED_ShowCompassImage, OLED_ShowCompassImageAt

// ===================================================================================
// Externe Variablen
// ===================================================================================
// Annahme: Dein SPI-Handle heißt hspi3 und ist in einer anderen Datei (z.B. main.c) definiert.
extern SPI_HandleTypeDef hspi3;

// Forward declarations for internal helpers
static void OLED_DrawGlyph6x8(uint16_t x, uint16_t y, const uint8_t glyph[6], uint16_t color);
static void OLED_DrawGlyph6x8_Scaled(uint16_t x, uint16_t y, const uint8_t glyph[6], uint16_t color, uint8_t scale);

// ===================================================================================
// Private Defines und Variablen
// ===================================================================================
// OLED_WIDTH/OLED_HEIGHT moved to oledc.h so other modules can use them.
#define OLED_BUFFER_SIZE (OLED_WIDTH * OLED_HEIGHT * 2) // *2 für 16-Bit Farbe
// Viele HAL/SPI/DMA Implementierungen haben Beschränkungen bei sehr großen
// Transfers. Bei ~18KB Gesamtlänge (96*96*2) kommt es auf manchen Boards
// zu abgeschnittenen letzten Zeilen. Wir senden daher in handlichen Chunks.
#define OLED_SPI_CHUNK_SIZE 4096 // Byte-Größe, sollte ein Vielfaches von 2 sein

// Compile-time sanity checks
#if (OLED_BUFFER_SIZE % 2) != 0
#error "OLED_BUFFER_SIZE muss ein Vielfaches von 2 sein (RGB565 -> 2 Bytes/Pixel)"
#endif
#if (OLED_SPI_CHUNK_SIZE % 2) != 0
#error "OLED_SPI_CHUNK_SIZE muss ein Vielfaches von 2 sein"
#endif

// ===================================================================================
// Funktions-Implementierungen
// ===================================================================================

void OLED_Init(void) {
    OLED_Reset();
    HAL_Delay(50);

    OLED_WriteCommand(SSD1351_CMD_SET_DISPLAY_OFF); // Display erst ausschalten
    // Hier könnten weitere Initialisierungsbefehle stehen (z.B. MUX Ratio, Remap, Clock Div...)

    OLED_Clear(); // Bildschirm leeren
    OLED_WriteCommand(SSD1351_CMD_SET_DISPLAY_ON);  // Display einschalten
}

void OLED_Reset(void) {
    // Annahme: Die GPIO-Definitionen (z.B. RESET_GPIO_Port) kommen aus main.h
    HAL_GPIO_WritePin(RST_OLED_GPIO_Port, RST_OLED_Pin, GPIO_PIN_RESET);
    HAL_Delay (10);
    HAL_GPIO_WritePin(RST_OLED_GPIO_Port, RST_OLED_Pin, GPIO_PIN_SET);
    HAL_Delay (10);
}

void OLED_WriteCommand(uint8_t command) {
    HAL_GPIO_WritePin(DC_OLED_GPIO_Port, DC_OLED_Pin, GPIO_PIN_RESET);  // DC low für Befehl
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);  // CS low -> Gerät auswählen
    HAL_SPI_Transmit(&hspi3, &command, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);    // CS high -> Auswahl aufheben
}

void OLED_WriteData(uint8_t data) {
    HAL_GPIO_WritePin(DC_OLED_GPIO_Port, DC_OLED_Pin, GPIO_PIN_SET);    // DC high für Daten
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);  // CS low -> Gerät auswählen
    HAL_SPI_Transmit(&hspi3, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);    // CS high -> Auswahl aufheben
}

void OLED_FillRun(uint16_t x, uint16_t y, uint16_t len, uint16_t color)
{
    if (y >= OLED_HEIGHT || x >= OLED_WIDTH || len == 0) return;
    if (x + len > OLED_WIDTH) len = OLED_WIDTH - x;

    // set window for single row from x..x+len-1
    OLED_WriteCommand(SSD1351_CMD_SET_COLUMN_ADDR);
    OLED_WriteData(x);
    OLED_WriteData(x + len - 1);
    OLED_WriteCommand(SSD1351_CMD_SET_ROW_ADDR);
    OLED_WriteData(y);
    OLED_WriteData(y);
    OLED_WriteCommand(SSD1351_CMD_WRITE_RAM);

    // prepare color bytes
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);

    HAL_GPIO_WritePin(DC_OLED_GPIO_Port, DC_OLED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);
    // stream len pixels of same color
    // small buffer to batch writes
    uint8_t buf[64];
    for (int i = 0; i < (int)sizeof(buf); i += 2) { buf[i] = hi; buf[i+1] = lo; }
    uint16_t remaining = len;
    while (remaining) {
        uint16_t chunkPixels = remaining;
        uint16_t maxPixels = (uint16_t)(sizeof(buf)/2);
        if (chunkPixels > maxPixels) chunkPixels = maxPixels;
        HAL_SPI_Transmit(&hspi3, buf, (uint16_t)(chunkPixels * 2), HAL_MAX_DELAY);
        remaining -= chunkPixels;
    }
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);
}

// ----------------------------------------------------------------------------------
// Pixel/line helpers (blocking SPI, easy to use for overlays)
// ----------------------------------------------------------------------------------
void OLED_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    // set single pixel window
    OLED_WriteCommand(SSD1351_CMD_SET_COLUMN_ADDR);
    OLED_WriteData(x);
    OLED_WriteData(x);
    OLED_WriteCommand(SSD1351_CMD_SET_ROW_ADDR);
    OLED_WriteData(y);
    OLED_WriteData(y);
    OLED_WriteCommand(SSD1351_CMD_WRITE_RAM);

    uint8_t buf[2];
    buf[0] = (color >> 8) & 0xFF;
    buf[1] = color & 0xFF;

    HAL_GPIO_WritePin(DC_OLED_GPIO_Port, DC_OLED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);
}


// Tiny 6x8 digit font (ASCII '0'..'9'); bits: MSB left, 8 rows per glyph
static const uint8_t digit6x8[10][6] = {
  {0x3E,0x51,0x49,0x45,0x3E,0x00}, // '0'
  {0x00,0x42,0x7F,0x40,0x00,0x00}, // '1'
  {0x42,0x61,0x51,0x49,0x46,0x00}, // '2'
  {0x21,0x41,0x45,0x4B,0x31,0x00}, // '3'
  {0x18,0x14,0x12,0x7F,0x10,0x00}, // '4'
  {0x27,0x45,0x45,0x45,0x39,0x00}, // '5'
  {0x3C,0x4A,0x49,0x49,0x30,0x00}, // '6'
  {0x01,0x71,0x09,0x05,0x03,0x00}, // '7'
  {0x36,0x49,0x49,0x49,0x36,0x00}, // '8'
  {0x06,0x49,0x49,0x29,0x1E,0x00}, // '9'
};

static void OLED_DrawGlyph6x8(uint16_t x, uint16_t y, const uint8_t glyph[6], uint16_t color)
{
    // Draw mirrored horizontally: use columns in reverse order
    for (int col = 0; col < 6; ++col) {
        int mc = 5 - col; // mirrored column index
        uint8_t bits = glyph[mc];
        for (int row = 0; row < 8; ++row) {
            if (bits & (1 << (7 - row))) {
                OLED_DrawPixel(x + col, y + row, color);
            }
        }
    }
}

void OLED_DrawNumber(uint16_t x, uint16_t y, int value, uint16_t color)
{
    // handle sign
    unsigned int u = (value < 0) ? (unsigned int)(-value) : (unsigned int)value;
    char buf[12];
    int idx = 0;
    if (value < 0) {
        // draw simple minus as a horizontal line (3x1) centered in 6x8 cell
        for (int i = 1; i <= 4; ++i) OLED_DrawPixel(x + i, y + 4, color);
        x += 7; // advance with spacing
    }
    // convert to decimal string
    do {
        buf[idx++] = (char)('0' + (u % 10));
        u /= 10;
    } while (u && idx < (int)sizeof(buf));
    if (idx == 0) buf[idx++] = '0';
    // Draw digits in forward order (LSB first). This compensates for mirrored display orientation
    int count = idx;
    for (int i = 0; i < count; ++i) {
        char c = buf[i];
        if (c >= '0' && c <= '9') {
            const uint8_t *g = digit6x8[c - '0'];
            OLED_DrawGlyph6x8(x, y, g, color);
            x += 7; // 6px glyph + 1px space
        }
    }
}

static void OLED_DrawGlyph6x8_Scaled(uint16_t x, uint16_t y, const uint8_t glyph[6], uint16_t color, uint8_t scale)
{
    if (scale == 0) return;
    // For each glyph row, build horizontal runs across scaled columns
    for (int row = 0; row < 8; ++row) {
        // scaled row repeats 'scale' times vertically
        for (int sy = 0; sy < (int)scale; ++sy) {
            int runStart = -1;
            int runLen = 0;
            int baseY = y + row * scale + sy;
            // iterate glyph columns and expand to scaled width
            for (int col = 0; col < 6; ++col) {
                int mc = 5 - col; // mirrored column index
                uint8_t bits = glyph[mc];
                int set = (bits & (1 << (7 - row))) ? 1 : 0;
                if (set) {
                    // this column contributes 'scale' pixels horizontally
                    if (runStart < 0) runStart = col * (int)scale;
                    runLen += (int)scale;
                } else {
                    if (runLen > 0) {
                        OLED_FillRun(x + runStart, (uint16_t)baseY, (uint16_t)runLen, color);
                        runStart = -1; runLen = 0;
                    }
                }
            }
            if (runLen > 0) {
                OLED_FillRun(x + runStart, (uint16_t)baseY, (uint16_t)runLen, color);
            }
        }
    }
}

void OLED_DrawNumberScaled(uint16_t x, uint16_t y, int value, uint16_t color, uint8_t scale)
{
    if (scale == 0) return;
    unsigned int u = (value < 0) ? (unsigned int)(-value) : (unsigned int)value;
    char buf[12];
    int idx = 0;
    if (value < 0) {
        // minus sign: a horizontal line with scaled thickness at center
        int lineY = y + 4 * scale;
        for (int i = 1 * scale; i <= 4 * scale; ++i) {
            // thickness equal to scale
            for (int t = 0; t < (int)scale; ++t) OLED_DrawPixel(x + i, lineY + t, color);
        }
        x += 7 * scale; // advance with spacing
    }
    do {
        buf[idx++] = (char)('0' + (u % 10));
        u /= 10;
    } while (u && idx < (int)sizeof(buf));
    if (idx == 0) buf[idx++] = '0';
    int count = idx;
    for (int i = 0; i < count; ++i) {
        char c = buf[i];
        if (c >= '0' && c <= '9') {
            const uint8_t *g = digit6x8[c - '0'];
            OLED_DrawGlyph6x8_Scaled(x, y, g, color, scale);
            x += 7 * scale; // 6*scale glyph + 1*scale space
        }
    }
}


// Fill entire screen with a solid RGB565 color
void OLED_DisplayColor(uint16_t color)
{
    // Set full window
    OLED_WriteCommand(SSD1351_CMD_SET_COLUMN_ADDR);
    OLED_WriteData(0);
    OLED_WriteData(OLED_WIDTH - 1);
    OLED_WriteCommand(SSD1351_CMD_SET_ROW_ADDR);
    OLED_WriteData(0);
    OLED_WriteData(OLED_HEIGHT - 1);
    OLED_WriteCommand(SSD1351_CMD_WRITE_RAM);

    // Prepare color bytes
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);

    // Small chunk buffer to stream data reliably
    uint8_t buf[64];
    for (size_t i = 0; i < sizeof(buf); i += 2) {
        buf[i] = hi;
        buf[i + 1] = lo;
    }

    HAL_GPIO_WritePin(DC_OLED_GPIO_Port, DC_OLED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);

    uint32_t pixels = (uint32_t)OLED_WIDTH * (uint32_t)OLED_HEIGHT;
    while (pixels) {
        uint32_t chunk_pixels = pixels;
        uint32_t max_pixels = sizeof(buf) / 2;
        if (chunk_pixels > max_pixels) chunk_pixels = max_pixels;
        HAL_SPI_Transmit(&hspi3, buf, (uint16_t)(chunk_pixels * 2u), HAL_MAX_DELAY);
        pixels -= chunk_pixels;
    }

    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);
}



void OLED_Clear(void) {
    // Der Bildschirm wird gelöscht, indem er mit Schwarz gefüllt wird.
    OLED_DisplayColor(0x0000); // 0x0000 ist Schwarz im RGB565-Format
}


// -----------------------------------------------------------------------------------
// Generic bitmap drawing (RGB565 raw data)
// -----------------------------------------------------------------------------------
void OLED_DrawBitmap(const uint8_t* bitmap, uint16_t bmp_w, uint16_t bmp_h, uint16_t x, uint16_t y)
{
    if (!bitmap) return;

    // Clip to screen bounds
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    uint16_t max_w = (x + bmp_w > OLED_WIDTH) ? (OLED_WIDTH - x) : bmp_w;
    uint16_t max_h = (y + bmp_h > OLED_HEIGHT) ? (OLED_HEIGHT - y) : bmp_h;

    // Set column / row window to the target area
    OLED_WriteCommand(SSD1351_CMD_SET_COLUMN_ADDR);
    OLED_WriteData(x);
    OLED_WriteData(x + max_w - 1);
    OLED_WriteCommand(SSD1351_CMD_SET_ROW_ADDR);
    OLED_WriteData(y);
    OLED_WriteData(y + max_h - 1);

    // Prepare to write RAM
    OLED_WriteCommand(SSD1351_CMD_WRITE_RAM);

    // Send the selected rectangle line-by-line (keeps offsets simple and avoids
    // having to create a temporary buffer if the full image is larger than the window)
    HAL_GPIO_WritePin(DC_OLED_GPIO_Port, DC_OLED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);

    uint32_t bytes_per_row = (uint32_t)bmp_w * 2u; // input image stride in bytes
    uint32_t send_per_row = (uint32_t)max_w * 2u; // how many bytes we actually send per row

    const uint8_t* row_ptr = bitmap;
    for (uint16_t row = 0; row < max_h; ++row) {
        // send only the leftmost max_w pixels of this row
        uint32_t remaining = send_per_row;
        uint32_t offset = 0;
        while (remaining) {
            uint32_t chunk = (remaining > OLED_SPI_CHUNK_SIZE) ? OLED_SPI_CHUNK_SIZE : remaining;
            HAL_SPI_Transmit(&hspi3, (uint8_t*)(&row_ptr[offset]), (uint16_t)chunk, HAL_MAX_DELAY);
            offset += chunk;
            remaining -= chunk;
        }

        // Advance row pointer by the input stride
        row_ptr += bytes_per_row;
    }

    HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);
}

// Convenience: show compass_mod_map (if present in project). We assume the
// image is exactly the screen size or at least large enough to cover the screen.
void OLED_ShowCompassImage(void)
{
    // If the array `compass_mod_map` exists in the project it will be linked
    // and this call will display it. If there is no such symbol the linker
    // will complain — that's valuable feedback to add/convert an image.
    // Use the compiled-in image dimensions when available; fall back to
    // the screen size if the metadata is absent or inconsistent.
    uint16_t w = compass_mod_map_w ? compass_mod_map_w : OLED_WIDTH;
    uint16_t h = compass_mod_map_h ? compass_mod_map_h : OLED_HEIGHT;

    // Basic sanity: do we have enough data for a w*h RGB565 image?
    if (compass_mod_map_size < ((uint32_t)w * (uint32_t)h * 2u)) {
        // if not enough data, try drawing full-screen using screen-size assumption
        OLED_DrawBitmap(compass_mod_map, OLED_WIDTH, OLED_HEIGHT, 0, 0);
        return;
    }

    // If the image exactly matches the framebuffer size we can send the
    // entire buffer in a single DMA transfer (faster and more robust).
    if ((w == OLED_WIDTH) && (h == OLED_HEIGHT) && (compass_mod_map_size >= OLED_BUFFER_SIZE)) {
        // Set the full-screen window
        OLED_WriteCommand(SSD1351_CMD_SET_COLUMN_ADDR);
        OLED_WriteData(0);
        OLED_WriteData(OLED_WIDTH - 1);
        OLED_WriteCommand(SSD1351_CMD_SET_ROW_ADDR);
        OLED_WriteData(0);
        OLED_WriteData(OLED_HEIGHT - 1);

        // Prepare to write RAM
        OLED_WriteCommand(SSD1351_CMD_WRITE_RAM);

        // Start a DMA transfer for the entire screen buffer
        HAL_GPIO_WritePin(DC_OLED_GPIO_Port, DC_OLED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);

        // start DMA: non-blocking; wait until ready
        if (HAL_SPI_Transmit_DMA(&hspi3, (uint8_t*)compass_mod_map, (uint16_t)OLED_BUFFER_SIZE) == HAL_OK) {
            // busy-wait until the SPI peripheral is ready again
            while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY) {
                HAL_Delay(1);
            }
        } else {
            // if DMA start failed, fall back to blocking transfer
            uint32_t rem = OLED_BUFFER_SIZE;
            uint32_t off = 0;
            while (rem) {
                uint32_t chunk = (rem > OLED_SPI_CHUNK_SIZE) ? OLED_SPI_CHUNK_SIZE : rem;
                HAL_SPI_Transmit(&hspi3, (uint8_t*)(&compass_mod_map[off]), (uint16_t)chunk, HAL_MAX_DELAY);
                off += chunk;
                rem -= chunk;
            }
        }

        HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);
        return;
    }

    // Otherwise draw the (possibly smaller) bitmap using the safe per-row path.
    // default to centered placement: if the display is larger than the image
    // center it horizontally/vertically. The user can use OLED_ShowCompassImageAt()
    // to place manually.
    uint16_t x = 0;
    uint16_t y = 0;
    if (OLED_WIDTH > w) x = (OLED_WIDTH - w) / 2;
    if (OLED_HEIGHT > h) y = (OLED_HEIGHT - h) / 2;
    OLED_DrawBitmap(compass_mod_map, w, h, x, y);
}

void OLED_DrawCompass(float heading_deg) {
    // Normalize heading to 0-360
    while (heading_deg < 0.0f) heading_deg += 360.0f;
    while (heading_deg >= 360.0f) heading_deg -= 360.0f;
    
    int deg = (int)(heading_deg + 0.5f);  // Round to nearest integer
    int heading_centi_local = (int)(heading_deg * 100.0f);
    
    // Skip if same as last frame
    if (heading_centi_local == last_drawn_heading) {
        return;
    }
    last_drawn_heading = heading_centi_local;
    
    // Calculate compass needle position
    const int radius = 30;
    const int cx = 64;
    const int cy = 48;
    double rad = (double)heading_deg * (3.14159265359 / 180.0);
    compass_x = cx + (int)(radius * sin(rad));
    compass_y = cy - (int)(radius * cos(rad));
    
    // 1. Draw background image
    OLED_ShowCompassImage();
    
    // 2. Draw centered heading number
    uint8_t num_scale = 1;
    int digits = (deg >= 100) ? 3 : ((deg >= 10) ? 2 : 1);
    int text_w = digits * 6 * num_scale + (digits - 1) * num_scale;
    int nx = cx - text_w / 2;
    int ny = cy - (8 * num_scale) / 2;
    if (digits == 3) nx += num_scale;
    OLED_DrawNumberScaled(nx, ny, deg, 0xFFFF, num_scale);
    
    // 3. Draw red compass needle (mirrored at center)
    int dot_x = (int)(2 * cx - compass_x);
    int dot_y = (int)(2 * cy - compass_y);
    
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            OLED_DrawPixel(dot_x + dx, dot_y + dy, 0xF800);
        }
    }
}



// ===================================================================================
// Tactical Compass HUD
// ===================================================================================


//-----------------------------------------------------------------------------------
// Optional: we override HAL callback to ensure any SPI DMA tx completion clears
// state quickly and allows the waiting loop above to finish sooner.
// -----------------------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi3) {
        // nothing special required here — HAL_SPI_GetState will flip to READY.
        // The callback exists so we can debug or extend behavior later.
    }
}


