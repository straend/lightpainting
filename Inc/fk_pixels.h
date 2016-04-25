#ifndef _FK_PIXELS_H
#define _FK_PIXELS_H

#include <stdint.h>
#include <stdbool.h>
#include "fatfs.h"


//#define CLEAR_LEDS()  
void CLEAR_LEDS(void);
uint8_t listDir(void);

bool fk_load_image_nr(uint8_t img);
void loadRow(void);
bool fk_open_bmp(char *filename);
bool fk_load_image(void);
bool sendLine(void);
bool fk_open_line_nr(uint8_t img);


uint8_t *pixeldata;
bool has_image;
uint16_t rows_in_current_image,
         current_row;
uint8_t tmp[144*3];
FATFS USBDISKFatFs;


// Get rid of Optimization paddings
#pragma pack(push, 1)
struct fk_BMP_File_Header
{
    uint16_t signature;	// Should be BM(0x4D42) for BitMap
    uint32_t filesize;
    uint32_t reserverd;
    uint32_t bOffBits;
};

struct fk_BMP_Info_Header
{
	uint32_t info_size;
	uint32_t width;
	uint32_t height;
    uint16_t planes;
    uint16_t bitperpixel;
    uint32_t compression;
    uint32_t image_size;
    uint32_t pixels_per_meter_x;
    uint32_t pixels_per_meter_y;
    uint32_t colors;
    uint32_t colors_importand;  //number of colors that are important
};

typedef struct {
	struct fk_BMP_File_Header fileheader;
	struct fk_BMP_Info_Header infoheader;
} fk_BMP_Header;
#pragma pack(pop)

#endif