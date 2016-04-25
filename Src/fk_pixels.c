#include "fk_pixels.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "fatfs.h"

#include "apa_led.h"

FIL BMP_Fil;

uint8_t listDir(void)
{
    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory search object */
    FILINFO fno;    /* File information */

    fr = f_findfirst(&dj, &fno, "", "*.bmp");  /* Start to search for photo files */
    if (fr!=FR_OK){
        return 0;
    }
    
    uint8_t no_of_files;
    while (fr == FR_OK && fno.fname[0]) {         /* Repeat while an item is found */
        char *f = fno.fname;
        ITM_SendChar(no_of_files+48);ITM_SendChar("\t");
        
        while (*f++)
            ITM_SendChar(*f);
        ITM_SendChar("\r");ITM_SendChar("\n");
        //setLed(no_of_files, 0, 0, 20);
        no_of_files++;
        fr = f_findnext(&dj, &fno);               /* Search for next item */
    }
    
    f_closedir(&dj);
    
    return no_of_files;
}
void loadRow(void)
{
    uint32_t i = current_row * 144*3;
	uint32_t ii;
    for(uint32_t led = 0;led<144; led++){
		ii = i + led*3;
		setLed(led, pixeldata[ii+2], pixeldata[ii+1], pixeldata[ii]);
	}

	if(current_row < rows_in_current_image-1){
  		current_row++;
	} else {
		current_row = 0;
	}
}
bool fk_load_image_nr(uint8_t img)
{
    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory search object */
    FILINFO fno;    /* File information */

    fr = f_findfirst(&dj, &fno, "", "*.bmp");  /* Start to search for photo files */
    if (fr!=FR_OK){
        return 0;
    }
    
    uint8_t no_of_files;
    while (fr == FR_OK && fno.fname[0]) {         /* Repeat while an item is found */
        if (no_of_files == img)
            break;
        no_of_files++;
        fr = f_findnext(&dj, &fno);               /* Search for next item */
    }
    f_closedir(&dj);
    ITM_SendChar(img+48);ITM_SendChar("\t");
    
    char *f = fno.fname;
    while (*f++)
        ITM_SendChar(*f);
    ITM_SendChar("\r");ITM_SendChar("\n");
        
    if (fk_open_bmp(fno.fname)){
        has_image = true;
        return true;
    } else {
        setLed(15, 100, 0, 0);
        setLed(16, 100, 0, 0);
    }
    return false;
}
bool fk_load_image()
{
    if(f_mount(&USBDISKFatFs, (TCHAR const*) USBH_Path, 0) != FR_OK){
	} else {
        if (fk_open_bmp("nyan_cat.bmp")){
            has_image = true;
            return true;
        } else {
            setLed(15, 100, 0, 0);
            setLed(16, 100, 0, 0);
        }
        
	}
    return false;
    
}
bool fk_open_line_nr(uint8_t img)
{
    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory search object */
    FILINFO fno;    /* File information */

    fr = f_findfirst(&dj, &fno, "", "*.bmp");  /* Start to search for photo files */
    if (fr!=FR_OK){
        return 0;
    }
    
    uint8_t no_of_files;
    while (fr == FR_OK && fno.fname[0]) {         /* Repeat while an item is found */
        if (no_of_files == img)
            break;
        no_of_files++;
        fr = f_findnext(&dj, &fno);               /* Search for next item */
    }
    f_closedir(&dj);
    ITM_SendChar(img+48);ITM_SendChar("\t");
    
    char *f = fno.fname;
    while (*f++)
        ITM_SendChar(*f);
    ITM_SendChar("\r");ITM_SendChar("\n");
        
    FRESULT res;
	uint32_t bytes_read;
	fk_BMP_Header header;
	
    if (f_open(&BMP_Fil, fno.fname, FA_READ) != FR_OK){
        setLed(18, 100,0,0);
		return false;
	}

	res = f_read(&BMP_Fil, &header, sizeof(fk_BMP_Header), (void *)&bytes_read);
	if(bytes_read != sizeof(fk_BMP_Header) || res!=FR_OK){
		f_close(&BMP_Fil);
        setLed(19, 100,0,0);
        return false;
	}
    rows_in_current_image = header.infoheader.height;
    current_row = 0;
    CLEAR_LEDS();
    
    return true;
}


bool sendLine(void)
{
	FRESULT res;
    UINT bytes_read2;
    
    res = f_read(&BMP_Fil, tmp, (144*3), &bytes_read2);    
    if(res != FR_OK){
        setLed(19, 100,0,0);
        return false;
    }
    current_row++;
    
    if(current_row == rows_in_current_image-1){
        res = f_lseek(&BMP_Fil, sizeof(fk_BMP_Header));
        setLed(143, 0, 0, 100);
    } 
    return true;
}
bool fk_open_bmp(char *filename)
{

	FRESULT res;
	uint32_t bytes_read;
	fk_BMP_Header header;
	if (f_open(&BMP_Fil, filename, FA_READ) != FR_OK){
        setLed(18, 100,0,0);
		return false;
	}

	res = f_read(&BMP_Fil, &header, sizeof(fk_BMP_Header), (void *)&bytes_read);
	if(bytes_read != sizeof(fk_BMP_Header) || res!=FR_OK){
		f_close(&BMP_Fil);
        setLed(19, 100,0,0);
        return false;
	}

    // Allocate memory for pixeldata
    pixeldata = (uint8_t *) calloc(144*3*header.infoheader.height,sizeof(uint8_t));
    if(pixeldata == NULL){
        has_image = false;
        setLed(120, 50, 0, 0);
        setLed(121, 50, 0, 0);
        return false;
    }
 
  
    UINT bytes_read2;
    
    uint16_t row;
    for(row=0; row<header.infoheader.height;row++){
        res = f_read(&BMP_Fil, pixeldata+(row*144*3), (144*3), &bytes_read2);    
    }
    //memset(pixeldata, 0, 144*3*header.infoheader.height);
    
	switch (res){
    case FR_OK:
      setLed(1, 0, 00, 50);
      break;
    case FR_DISK_ERR:
      setLed(2, 10, 40, 0);
      break;
    case FR_INT_ERR:
      setLed(3, 10, 40, 0);
      break;
    case FR_INVALID_OBJECT:
      setLed(4, 10, 40, 0);
      break;
    case FR_TIMEOUT:
      setLed(5, 10, 40, 0);
      break;
      
    }
    if(res!=FR_OK) { 
        setLed(21, 0, 50, 0);
        return false;
	}

    f_close(&BMP_Fil);
    rows_in_current_image = header.infoheader.height;
    current_row = 0;
  
    // clear all leds
	//CLEAR_LEDS();
  
  	return true;
}

void CLEAR_LEDS(void)
{
     uint32_t i; 
     for(i=0;i<144;i++)
     setLed(i, 0, 0, 0);
}
