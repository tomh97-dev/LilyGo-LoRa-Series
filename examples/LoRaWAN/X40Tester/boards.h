#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "utilities.h"

#ifdef HAS_SDCARD
#include <SD.h>
#include <FS.h>
#endif

#ifdef HAS_DISPLAY
#include <U8g2lib.h>
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2;
#endif

#define initPMU()
#define disablePeripherals()

void initBoard();