/*
 * Copyright (C) OpenTX
 *
 * Source:
 *  https://github.com/opentx/libopenui
 *
 * This file is a part of libopenui library.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#include "dialog.h"
#include "mainwindow.h"
#include "theme.h"

Dialog::Dialog(std::string title, const rect_t & rect):
  ModalWindow(),
  content(createDialogWindow(this, rect)),
  form(content, {0, POPUP_HEADER_HEIGHT, rect.w, coord_t(rect.h - POPUP_HEADER_HEIGHT)}, FORM_NO_BORDER)
{
  bringToTop();
  content->setTitle(title);
  form.setFocus();
}
