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
 
#define SPACEMOUSE_BAUDRATE             ( 38400 )
#define SPACEMOUSE_CHANNEL_COUNT        ( 6 )

#define SPACEMOUSE_INPUT_OFFSET         ( 8192 )
#define SPACEMOUSE_OUTPUT_OFFSET        ( 350 )

#define SPACEMOUSE_PROTO_HEADER         0x96
#define SPACEMOUSE_PROTO_FOOTER         0x8D

#define SPACEMOUSE_CMD_REQSINGLEDATA    0xAC
#define SPACEMOUSE_CMD_TARE             0xAD
#define SPACEMOUSE_CMD_STARTSTREAMING   0xAE
#define SPACEMOUSE_CMD_STOPSTREAMING    0xAF

#define SPACEMOUSE_LENGTH_HEADER        1
#define SPACEMOUSE_LENGTH_DATA          (2*SPACEMOUSE_CHANNEL_COUNT)
#define SPACEMOUSE_LENGTH_CHECKSUM      2
#define SPACEMOUSE_LENGTH_FOOTER        1
#define SPACEMOUSE_LENGTH_TELEGRAM      (SPACEMOUSE_LENGTH_HEADER + SPACEMOUSE_LENGTH_DATA + SPACEMOUSE_LENGTH_CHECKSUM + SPACEMOUSE_LENGTH_FOOTER)

typedef  struct
{
  unsigned char head;
  unsigned char data[SPACEMOUSE_LENGTH_TELEGRAM];
  unsigned short checkSum;
  unsigned char startIndex;
  unsigned char endIndex;
  unsigned char dataIndex;
  unsigned char completeFlg;
  unsigned char parsestate;
  unsigned char recevied;
  bool msg_OK;
} STRUCT_SPACEMOUSE;

enum
{
  SM_START = 0,
  SM_DATA,
  SM_CHECKSUM,
  SM_FOOTER,
};

#if defined(SPACEMOUSE_U3)
  extern signed short spacemouseu3_values[SPACEMOUSE_CHANNEL_COUNT];
  void spacemouseu3_tare( void );
#endif

#if defined(SPACEMOUSE_U6)
  extern signed short spacemouseu6_values[SPACEMOUSE_CHANNEL_COUNT];
  void spacemouseu6_tare( void );
#endif
