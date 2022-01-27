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

#include <algorithm>
#include "model_select.h"
#include "opentx.h"
#include "storage/modelslist.h"
#include "libopenui.h"

#if LCD_W > LCD_H
constexpr int MODEL_CELLS_PER_LINE = 3;
#else
constexpr int MODEL_CELLS_PER_LINE = 2;
#endif

constexpr coord_t MODEL_CELL_PADDING = 6;

constexpr coord_t MODEL_SELECT_CELL_WIDTH =
    (LCD_W - (MODEL_CELLS_PER_LINE + 1) * MODEL_CELL_PADDING) /
    MODEL_CELLS_PER_LINE;

constexpr coord_t MODEL_SELECT_CELL_HEIGHT = 92;

constexpr coord_t MODEL_IMAGE_WIDTH  = MODEL_SELECT_CELL_WIDTH;
constexpr coord_t MODEL_IMAGE_HEIGHT = 72;

inline tmr10ms_t getTicks() { return g_tmr10ms; }

class ModelButton : public Button
{
 public:
  ModelButton(FormGroup *parent, const rect_t &rect, ModelCell *modelCell) :
      Button(parent, rect), modelCell(modelCell)
  {
    load();
  }

  ~ModelButton()
  {
    if (buffer) { delete buffer; }
  }
  
  void load()
  {
#if defined(SDCARD_RAW)
    uint8_t version;
#endif

    PACK(struct {
      ModelHeader header;
      TimerData timers[MAX_TIMERS];
    })
    partialModel;
    const char *error = nullptr;

    if (strncmp(modelCell->modelFilename, g_eeGeneral.currModelFilename,
                LEN_MODEL_FILENAME) == 0) {
      memcpy(&partialModel.header, &g_model.header, sizeof(partialModel));
#if defined(SDCARD_RAW)
      version = EEPROM_VER;
#endif
    } else {
#if defined(SDCARD_RAW)
      error =
          readModelBin(modelCell->modelFilename, (uint8_t *)&partialModel.header,
                       sizeof(partialModel), &version);
#else
      error = readModel(modelCell->modelFilename,
                        (uint8_t *)&partialModel.header, sizeof(partialModel));
#endif
    }

    if (!error) {
      if (modelCell->modelName[0] == '\0' &&
          partialModel.header.name[0] != '\0') {

#if defined(SDCARD_RAW)
        if (version == 219) {
          int len = (int)sizeof(partialModel.header.name);
          char* str = partialModel.header.name;
          for (int i=0; i < len; i++) {
            str[i] = zchar2char(str[i]);
          }
          // Trim string
          while(len > 0 && str[len-1]) {
            if (str[len - 1] != ' ' && str[len - 1] != '\0') break;
            str[--len] = '\0';
          }
        }
#endif
        modelCell->setModelName(partialModel.header.name);
      }
    }

    delete buffer;
    buffer = new BitmapBuffer(BMP_RGB565, width(), height());
    if (buffer == nullptr) {
      return;
    }
    buffer->clear(COLOR_THEME_PRIMARY2);

    if (error) {
      buffer->drawText(width() / 2, 2, "(Invalid Model)",
                       COLOR_THEME_SECONDARY1 | CENTERED);
    } else {
      GET_FILENAME(filename, BITMAPS_PATH, partialModel.header.bitmap, "");
      const BitmapBuffer *bitmap = BitmapBuffer::loadBitmap(filename);
      if (bitmap) {
        buffer->drawScaledBitmap(bitmap, 0, 0, width(), height());
        delete bitmap;
      } else {
        buffer->drawText(width() / 2, 56, "(No Picture)",
                         FONT(XXS) | COLOR_THEME_SECONDARY1 | CENTERED);
      }
    }
  }

  void paint(BitmapBuffer *dc) override
  {
    FormField::paint(dc);

    if (buffer)
      dc->drawBitmap(0, 0, buffer);

    if (modelCell == modelslist.getCurrentModel()) {
      dc->drawSolidFilledRect(0, 0, width(), 20, COLOR_THEME_ACTIVE);
      dc->drawSizedText(width() / 2, 2, modelCell->modelName,
                        LEN_MODEL_NAME,
                        COLOR_THEME_SECONDARY1 | CENTERED);
    } else {
      LcdFlags textColor;
      // if (hasFocus()) {
      //   dc->drawFilledRect(0, 0, width(), 20, SOLID, COLOR_THEME_FOCUS, 5);
      //   textColor = COLOR_THEME_PRIMARY2;
      // }
      // else {
        dc->drawFilledRect(0, 0, width(), 20, SOLID, COLOR_THEME_PRIMARY2, 5);
        textColor = COLOR_THEME_SECONDARY1;
      // }

      dc->drawSizedText(width() / 2, 2, modelCell->modelName,
                        LEN_MODEL_NAME,
                        textColor | CENTERED);
    }

    if (!hasFocus()) {
      dc->drawSolidRect(0, 0, width(), height(), 1, COLOR_THEME_SECONDARY2);
    } else {
      dc->drawSolidRect(0, 0, width(), height(), 2, COLOR_THEME_FOCUS);
    }
  }

  const char *modelFilename() { return modelCell->modelFilename; }

 protected:
  ModelCell *modelCell;
  BitmapBuffer *buffer = nullptr;
};

class ModelCategoryPageBody : public FormWindow
{
 public:
  ModelCategoryPageBody(FormWindow *parent, const rect_t &rect,
                        ModelsCategory *category) :
      FormWindow(parent, rect, FORM_FORWARD_FOCUS), category(category)
  {
    update();
  }

  void update(int selected = -1)
  {
    clear();

    if (selected < 0) {
      auto model = modelslist.getCurrentModel();
      selected = category->getModelIndex(model);
      if (selected < 0) selected = 0;
    }

    int index = 0;
    coord_t y = MODEL_CELL_PADDING;
    coord_t x = MODEL_CELL_PADDING;

    ModelButton* selectButton = nullptr;
    for (auto &model : *category) {
      auto button = new ModelButton(
          this, {x, y, MODEL_SELECT_CELL_WIDTH, MODEL_SELECT_CELL_HEIGHT},
          model);
      button->setPressHandler([=]() -> uint8_t {
        if (button->hasFocus()) {
          Menu *menu = new Menu(parent);
          if (model != modelslist.getCurrentModel()) {
            menu->addLine(STR_SELECT_MODEL, [=]() {
              bool modelConnected = TELEMETRY_STREAMING() &&
                                    !g_eeGeneral.disableRssiPoweroffAlarm;
              if (modelConnected) {
                AUDIO_ERROR_MESSAGE(AU_MODEL_STILL_POWERED);
                if (!confirmationDialog(
                        STR_MODEL_STILL_POWERED, nullptr, false, []() {
                          tmr10ms_t startTime = getTicks();
                          while (!TELEMETRY_STREAMING()) {
                            if (getTicks() - startTime > TELEMETRY_CHECK_DELAY10ms)
                              break;
                          }
                          return !TELEMETRY_STREAMING() ||
                                 g_eeGeneral.disableRssiPoweroffAlarm;
                        })) {
                  return;  // stop if connected but not confirmed
                }
              }

              // store changes (if any) and load selected model
              storageFlushCurrentModel();
              storageCheck(true);
              memcpy(g_eeGeneral.currModelFilename, model->modelFilename,
                     LEN_MODEL_FILENAME);

              loadModel(g_eeGeneral.currModelFilename, false);
              storageDirty(EE_GENERAL);
              storageCheck(true);

              modelslist.setCurrentModel(model);
              modelslist.setCurrentCategory(category);
              this->onEvent(EVT_KEY_FIRST(KEY_EXIT));
              checkAll();
            });
          }
          menu->addLine(STR_CREATE_MODEL, getCreateModelAction());
          menu->addLine(STR_DUPLICATE_MODEL, [=]() {
            char duplicatedFilename[LEN_MODEL_FILENAME + 1];
            memcpy(duplicatedFilename, model->modelFilename,
                   sizeof(duplicatedFilename));
            if (findNextFileIndex(duplicatedFilename, LEN_MODEL_FILENAME,
                                  MODELS_PATH)) {
              sdCopyFile(model->modelFilename, MODELS_PATH, duplicatedFilename,
                         MODELS_PATH);
              modelslist.addModel(category, duplicatedFilename);
              update(index);
            } else {
              POPUP_WARNING("Invalid File");
            }
          });
          if (model != modelslist.getCurrentModel()) {
            // Move
            if(modelslist.getCategories().size() > 1) {
              menu->addLine(STR_MOVE_MODEL, [=]() {
              auto moveToMenu = new Menu(parent);
              moveToMenu->setTitle(STR_MOVE_MODEL);              
                for (auto newcategory: modelslist.getCategories()) {
                  if(category != newcategory) {
                    moveToMenu->addLine(std::string(newcategory->name, sizeof(newcategory->name)), [=]() {
                      modelslist.moveModel(model, category, newcategory);
                      update(index < (int)category->size() - 1 ? index : index - 1);
                      modelslist.save();
                    });
                  }
                }
              });
            }
            menu->addLine(STR_DELETE_MODEL, [=]() {
              new ConfirmDialog(
                  parent, STR_DELETE_MODEL,
                  std::string(model->modelName, sizeof(model->modelName))
                      .c_str(),
                  [=] {
                    modelslist.removeModel(category, model);
                    update(index < (int)category->size() - 1 ? index : index - 1);
                  });
            });
          }
        } else {
          button->setFocus(SET_FOCUS_DEFAULT);
        }
        return 1;
      });

      if (selected == index) {
        selectButton = button;
      }

      index++;

      if (index % MODEL_CELLS_PER_LINE == 0) {
        x = MODEL_CELL_PADDING;
        y += MODEL_SELECT_CELL_HEIGHT + MODEL_CELL_PADDING;
      } else {
        x += MODEL_CELL_PADDING + MODEL_SELECT_CELL_WIDTH;
      }
    }

    if (index % MODEL_CELLS_PER_LINE != 0) {
      y += MODEL_SELECT_CELL_HEIGHT + MODEL_CELL_PADDING;
    }
    setInnerHeight(y);

    if (category->empty()) {
      setFocus();
    } else if (selectButton) {
      selectButton->setFocus();
    }
  }

  void addFirstModel() {
    Menu *menu = new Menu(this);
    menu->addLine(STR_CREATE_MODEL, getCreateModelAction());      
  }

#if defined(HARDWARE_KEYS)
  void onEvent(event_t event) override
  {
    if (event == EVT_KEY_BREAK(KEY_ENTER)) {
      addFirstModel();
    } else {
      FormWindow::onEvent(event);
    }
  }
#endif

#if defined(HARDWARE_TOUCH)
    bool onTouchEnd(coord_t x, coord_t y) override
    {
      if(category->size() == 0)
        addFirstModel();
      else
        FormWindow::onTouchEnd(x,y);
      return true;
    }
#endif
  

  void setFocus(uint8_t flag = SET_FOCUS_DEFAULT,
                Window *from = nullptr) override
  {
    if (category->empty()) {
      // use Window::setFocus() to avoid forwarding focus to nowhere
      // this crashes currently in libopenui
      Window::setFocus(flag, from);
    } else {
      FormWindow::setFocus(flag, from);
    }
  }

 protected:
  ModelsCategory *category;

  std::function<void(void)> getCreateModelAction()
  {
    return [=]() {
      storageCheck(true);
      auto model = modelslist.addModel(category, createModel(), false);
      model->setModelName(g_model.header.name);
      modelslist.setCurrentModel(model);
      modelslist.save();
      update(category->size() - 1);
    };
  }
};

class ModelCategoryPage : public PageTab
{
 public:
  explicit ModelCategoryPage(ModelsCategory *category) :
      PageTab(category->name, ICON_MODEL_SELECT_CATEGORY), category(category)
  {
  }

 protected:
  ModelsCategory *category;

  void build(FormWindow *window) override
  {
    new ModelCategoryPageBody(
        window, {0, 0, LCD_W, window->height()},
        category);
  }
};

class CategoryGroup: public FormGroup
{
  public:
    CategoryGroup(Window * parent, const rect_t & rect, ModelsCategory *category) :
      FormGroup(parent, rect),
      category(category)
    {
    }

    void paint(BitmapBuffer * dc) override
    {
      dc->drawSolidFilledRect(0, 0, width(), height(), COLOR_THEME_PRIMARY2);
      FormGroup::paint(dc);
    }

  protected:
    ModelsCategory *category;
};

class CategoryEditPage : public PageTab
{
  public:
    explicit CategoryEditPage(ModelSelectMenu *modelselectmenu, bool scrolltobot=false) : 

      PageTab(STR_MODEL_CATEGORIES, ICON_MODEL_SETUP),
      modelselectmenu(modelselectmenu), 
      scrolltobot(scrolltobot)
    {     
    }

  protected:
    void update()
    {
      modelselectmenu->build(0);
    }

    void build(FormWindow *window) override
    {
      FormGridLayout grid;
      grid.setMarginRight(15);
      grid.setLabelWidth(0);
      grid.spacer();

      coord_t y = 2;

      for (auto category: modelslist.getCategories()) {
        // NAME
        auto catname = new TextEdit(window, grid.getFieldSlot(3,0), category->name, sizeof(category->name));
        catname->setChangeHandler([=]() {          
          if(category->name[0] == '\0') {
            category->name[0] = ' '; category->name[1] = '\0';            
          }
          modelslist.save();
          update();
        });

        // Details
        char cnt[19];
        snprintf(cnt, sizeof(cnt), "%u %s", category->size(), STR_MODELS);
        new StaticText(window, grid.getFieldSlot(3,1), cnt);

        if(category->empty()) {
          new TextButton(window, grid.getFieldSlot(3,2),TR_DELETE, [=]() -> uint8_t {
            new ConfirmDialog(window, STR_DELETE_CATEGORY,
              std::string(category->name, sizeof(category->name)).c_str(),
              [=] {
                modelslist.removeCategory(category);    
                modelslist.save();
                update();
              });
            return 0;
          });
          } else {
#ifdef CATEGORIES_SHOW_DELETE_NON_EMPTY
          new TextButton(window, grid.getFieldSlot(3,2),STR_DELETE, [=]() -> uint8_t {
            new MessageDialog(window, STR_DELETE_CATEGORY, TR_CAT_NOT_EMPTY);
            return 0;
          });
#endif
          }

        grid.nextLine();

        grid.spacer();
        coord_t height = grid.getWindowHeight();
        //window->setHeight(height);
        y += height + 2;
      }
            
      new TextButton(window, grid.getCenteredSlot(LCD_W/2), STR_CREATE_CATEGORY, [=]() -> uint8_t {
        modelslist.createCategory("New");
        update();
        return 0;
      });

      grid.nextLine();

      window->setInnerHeight(grid.getWindowHeight()); 

      if(scrolltobot)
        window->setScrollPositionY(y+40);
    }  
  private:
    ModelSelectMenu *modelselectmenu;
    bool scrolltobot;
};

ModelSelectMenu::ModelSelectMenu():
  TabsGroup(ICON_MODEL_SELECT)
{  
  build();
}

void ModelSelectMenu::build(int index) 
{  
  modelslist.clear();
  modelslist.load();

  removeAllTabs();

  TRACE("TabsGroup: %p", this);  
  
  addTab(new CategoryEditPage(this));

  for (auto category: modelslist.getCategories()) {
    addTab(new ModelCategoryPage(category));
  }

  if(index < 0) {
    int idx = modelslist.getCurrentCategoryIdx();
    if (idx >= 0) {
      setCurrentTab(idx+1);
    }
  } else {
    if(index < static_cast<int>(modelslist.getCategories().size()))
      setCurrentTab(index);
  }
}
