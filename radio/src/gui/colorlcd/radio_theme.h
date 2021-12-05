/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
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
#include "tabsgroup.h"
#include "color_editor.h"
#include "file_preview.h"
#include "file_carosell.h"

constexpr int BOX_MARGIN = 2;
constexpr int MAX_BOX_WIDTH = 15;

class PageButton : public Window
{
  public:
   PageButton(FormGroup *parent, const rect_t &rect, std::string text,
            std::function<void ()> setTab = nullptr,
            WindowFlags windowFlags = BUTTON_BACKGROUND | OPAQUE,
            LcdFlags textFlags = 0) :
       Window(parent, rect, windowFlags | NO_FOCUS, textFlags),
       _setTab(std::move(setTab)),
       _text(text)
   {
   }

#if defined(HARDWARE_TOUCH)
  bool onTouchEnd(coord_t x, coord_t y) override
  {
    if (_setTab != nullptr) {
      _setTab();
    }

    return true;
  }
#endif

  void paint(BitmapBuffer *dc) override
  {   
    uint32_t textColor = _checked ? COLOR_THEME_PRIMARY2 : COLOR_THEME_PRIMARY1;
    dc->clear(_checked ? COLOR_THEME_ACTIVE : COLOR_THEME_SECONDARY2);

    dc->drawText(rect.w / 2, 1 + (rect.h - getFontHeight(textFlags)) / 2,
                 _text.c_str(), CENTERED | textColor);
  }

   void check(bool checked) 
   {
     _checked = checked;
     invalidate();
   }
  
  private:
    std::function<void ()> _setTab;
    bool _checked = false;
    std::string _text;
};

class ThemeColorPreview : public FormField
{
  public:
    ThemeColorPreview(Window *parent, const rect_t &rect, std::vector<ColorEntry> colorList) :
      FormField(parent, rect, NO_FOCUS),
      colorList(colorList)
    {
      setBoxWidth();
    }
    ~ThemeColorPreview()
    {
    }

    void setColorList(std::vector<ColorEntry> colorList)
    {
      this->colorList.assign(colorList.begin(), colorList.end());
      setBoxWidth();
      invalidate();
    }

    void paint(BitmapBuffer *dc) override
    {
      int totalNessessarySpace = colorList.size() * (boxWidth + 2);
      int axis = rect.w > rect.h ? rect.w : rect.h;
      axis = (axis - totalNessessarySpace) / 2;
      for (auto color: colorList) {
        if (rect.w > rect.h) {  
          dc->drawSolidFilledRect(axis, 0, boxWidth, boxWidth, COLOR2FLAGS(color.colorValue));
          dc->drawSolidRect(axis, 0, boxWidth, boxWidth, 1, COLOR2FLAGS(BLACK));
        } else {
          dc->drawSolidFilledRect(0, axis, boxWidth, boxWidth, COLOR2FLAGS(color.colorValue));
          dc->drawSolidRect(0, axis, boxWidth, boxWidth, 1, COLOR2FLAGS(BLACK));
        }
        axis += boxWidth + BOX_MARGIN;
      }
    }

  protected:
    std::vector<ColorEntry> colorList;
    int boxWidth = MAX_BOX_WIDTH;
    void setBoxWidth()
    {
      auto winSize = rect.w > rect.h ? rect.w : rect.h;
      boxWidth = winSize / (colorList.size() + BOX_MARGIN);
      boxWidth = min(boxWidth, MAX_BOX_WIDTH);
    }
};


class ThemeSetupPage: public PageTab {
  public:
    ThemeSetupPage();
    ~ThemeSetupPage();

    void build(FormWindow * window) override;
    void checkEvents() override;

  protected:
    Window *pageWindow = nullptr;
    Window *previewWindow = nullptr;
    FileCarosell *fileCarosell = nullptr;
    ThemeColorPreview *themeColorPreview = nullptr;
    ListBox *listBox = nullptr;
    StaticText *authorText = nullptr;
    StaticText *nameText = nullptr;
    int currentTheme = 0;
    void setupListbox(FormWindow *window, rect_t r, ThemePersistance *tp);
    void displayThemeMenu(Window *window, ThemePersistance *tp);
    void setAuthor(ThemeFile *theme);
};
