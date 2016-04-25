#ifndef _APA_LED_H_
#define _APA_LED_H_

#include <stdint.h>

#define NO_OF_LEDS  (144)
typedef union  {
  struct {
    unsigned start  :3;
    unsigned global :5;
    uint8_t blue;
    uint8_t green;
    uint8_t red;
  } data;
  uint8_t raw[4];

} LED_FRAME_u;

typedef union  {
  struct {
    uint8_t startFrame[4];
    LED_FRAME_u LED_FRAME[NO_OF_LEDS];
    uint8_t endFrame[(NO_OF_LEDS/8/2)];
  } data;
  uint8_t raw[4 + (NO_OF_LEDS*4) + (NO_OF_LEDS/8/2)];
} ALL_FRAMES_u;

ALL_FRAMES_u ALL_FRAMES;


void initLedFrame(void);
void setLed(uint32_t led, uint8_t r, uint8_t g, uint8_t b);
#define sendLedData(spi_hw) HAL_SPI_Transmit_DMA(spi_hw, (uint8_t *) &ALL_FRAMES, sizeof(ALL_FRAMES.raw))


#endif
