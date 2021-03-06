#include "apa_led.h"
#include <stdint.h>

extern const uint8_t gammaLUT[];

void initLedFrame(void)
{
    ALL_FRAMES.data.startFrame[0] = 0;ALL_FRAMES.data.startFrame[0] = 0;
    ALL_FRAMES.data.startFrame[2] = 0;ALL_FRAMES.data.startFrame[3] = 0;
    uint32_t i;
    for(i=0;i<NO_OF_LEDS;i++){
        ALL_FRAMES.data.LED_FRAME[i].data.start   = 0b111;
        ALL_FRAMES.data.LED_FRAME[i].data.global  = 0b11111;
        ALL_FRAMES.data.LED_FRAME[i].data.blue    = 0;
        ALL_FRAMES.data.LED_FRAME[i].data.green   = 0;
        ALL_FRAMES.data.LED_FRAME[i].data.red     = 0;
    }
    for(i=0;i<NO_OF_LEDS/8/2;i++){
        ALL_FRAMES.data.endFrame[i]=0xFF;
    }
}

void setLed(uint32_t led, uint8_t r, uint8_t g, uint8_t b)
{
    ALL_FRAMES.data.LED_FRAME[led].data.red   = gammaLUT[r];
    ALL_FRAMES.data.LED_FRAME[led].data.green = gammaLUT[g];
    ALL_FRAMES.data.LED_FRAME[led].data.blue  = gammaLUT[b];
}

const uint8_t gammaLUT[] = {
  0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   
  1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   
  3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,   6,   6,   6,   
  6,   7,   7,   7,   7,   8,   8,   8,   8,   9,   9,   9,   10,  10,  10,  11,  
  11,  11,  12,  12,  12,  13,  13,  13,  14,  14,  14,  15,  15,  16,  16,  16,  
  17,  17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  24,  
  25,  25,  26,  26,  27,  28,  28,  29,  29,  30,  31,  31,  32,  33,  33,  34,  
  35,  35,  36,  37,  37,  38,  39,  40,  40,  41,  42,  43,  44,  44,  45,  46,  
  47,  48,  49,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  
  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,  
  79,  80,  82,  83,  84,  85,  87,  88,  89,  90,  92,  93,  94,  96,  97,  99,  
  100, 101, 103, 104, 106, 107, 108, 110, 111, 113, 114, 116, 118, 119, 121, 122, 
  124, 125, 127, 129, 130, 132, 134, 135, 137, 139, 141, 142, 144, 146, 148, 149, 
  151, 153, 155, 157, 159, 161, 162, 164, 166, 168, 170, 172, 174, 176, 178, 180, 
  182, 185, 187, 189, 191, 193, 195, 197, 200, 202, 204, 206, 208, 211, 213, 215, 
  218, 220, 222, 225, 227, 230, 232, 234, 237, 239, 242, 244, 247, 249, 252, 255
};
