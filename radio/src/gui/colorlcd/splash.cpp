/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x 
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"

#if defined(SPLASH)

const uint8_t __bmp_splash[] {
#include "splash.lbm"
};

const uint8_t __bmp_splash_chr[] {
#include "splash_chr.lbm"
};

void drawSplash()
{
  constexpr LcdFlags splash_background_color =
    COLOR2FLAGS(((0xC >> 3) << 11) | ((0x3F >> 2) << 5) | (0x66 >> 3));

  static bool loadSplashImg = true;
  static BitmapBuffer * splashImg = nullptr;

  // try splash from SD card first
  if (loadSplashImg && splashImg == nullptr) {
    if (!sdMounted()) sdInit();
    splashImg = BitmapBuffer::loadBitmap(BITMAPS_PATH "/" SPLASH_FILE);
    loadSplashImg = false;

    // otherwise load from FLASH
	struct gtm t;
	gettime(&t);
	// Please do not spoil the suprise by discussing following lines in public
	// mon == 11 == December, starts from 0!
	if (t.tm_mon == 11 && t.tm_mday >= 24 && t.tm_mday <= 26)
	{
		if (splashImg == nullptr) {
			splashImg = BitmapBuffer::loadRamBitmap(__bmp_splash_chr, sizeof(__bmp_splash_chr));
		}
	} else
	{
		if (splashImg == nullptr) {
			splashImg = BitmapBuffer::loadRamBitmap(__bmp_splash, sizeof(__bmp_splash));
		}
	}
  }

  lcd->clear(splash_background_color);

  if (splashImg) {
    lcd->drawBitmap((LCD_W - splashImg->width())/2,
                    (LCD_H - splashImg->height())/2,
                    splashImg);
  }

  lcdRefresh();
}
#endif
