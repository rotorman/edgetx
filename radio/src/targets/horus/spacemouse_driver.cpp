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

#include "opentx.h"
#include "spacemouse_driver.h"

DMAFifo<4*SPACEMOUSE_LENGTH_TELEGRAM> spacemouseDMAFifo __DMA (SPACEMOUSE_DMA_Stream_RX);
//unsigned char SpaceMouseCmd;

STRUCT_SPACEMOUSE SpaceMouseProtocol = { 0 };
signed short spacemouse_values[SPACEMOUSE_CHANNEL_COUNT] = { 0 };

unsigned short calc_checksum(void *pBuffer,unsigned char BufferSize)
{
  unsigned short checksum = 0;
  while (BufferSize)
  {
      checksum += (*(unsigned char *)pBuffer);
      pBuffer = (void *)((unsigned char *)pBuffer + 1);
      BufferSize--;
  }
  return (checksum & 0x3FFF);
}

//uint16_t get_flysky_hall_adc_value(uint8_t ch)
uint16_t get_spacemouse_adc_value(uint8_t ch)
{
  if (ch >= SPACEMOUSE_CHANNEL_COUNT) {
    return 0;
  }

  return spacemouse_values[ch] + SPACEMOUSE_OFFSET_VALUE;
}

uint8_t SpaceMouseGetByte(uint8_t * byte)
{
  return spacemouseDMAFifo.pop(*byte);
}

void spacemouse_tare( void )
{
  USART_SendData(SPACEMOUSE_SERIAL_USART, SPACEMOUSE_CMD_TARE);
}

void spacemouse_startstreaming( void )
{
  USART_SendData(SPACEMOUSE_SERIAL_USART, SPACEMOUSE_CMD_STARTSTREAMING);
}

void spacemouse_init()
{
  USART_DeInit(SPACEMOUSE_SERIAL_USART);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPACEMOUSE_SERIAL_RX_DMA_Stream_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_PinAFConfig(SPACEMOUSE_SERIAL_GPIO, SPACEMOUSE_SERIAL_RX_GPIO_PinSource, SPACEMOUSE_SERIAL_GPIO_AF);
  GPIO_PinAFConfig(SPACEMOUSE_SERIAL_GPIO, SPACEMOUSE_SERIAL_TX_GPIO_PinSource, SPACEMOUSE_SERIAL_GPIO_AF);

  GPIO_InitStructure.GPIO_Pin = SPACEMOUSE_SERIAL_GPIO_PIN_TX | SPACEMOUSE_SERIAL_GPIO_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(SPACEMOUSE_SERIAL_GPIO, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = SPACEMOUSE_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(SPACEMOUSE_SERIAL_USART, &USART_InitStructure);

  DMA_Cmd(SPACEMOUSE_DMA_Stream_RX, DISABLE);
  USART_DMACmd(SPACEMOUSE_SERIAL_USART, USART_DMAReq_Rx, DISABLE);
  DMA_DeInit(SPACEMOUSE_DMA_Stream_RX);

  DMA_InitTypeDef DMA_InitStructure;
  spacemouseDMAFifo.clear();

  USART_ITConfig(SPACEMOUSE_SERIAL_USART, USART_IT_RXNE, DISABLE);
  USART_ITConfig(SPACEMOUSE_SERIAL_USART, USART_IT_TXE, DISABLE);

  // Turn on AUX1 power
  GPIO_InitStructure.GPIO_Pin = SPACEMOUSE_SERIAL_PWR_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPACEMOUSE_SERIAL_PWR_GPIO, &GPIO_InitStructure);
  GPIO_SetBits(SPACEMOUSE_SERIAL_PWR_GPIO, SPACEMOUSE_SERIAL_PWR_GPIO_PIN);

  // Configure and enable DMA for receive
  DMA_InitStructure.DMA_Channel = SPACEMOUSE_SERIAL_DMA_Channel;
  DMA_InitStructure.DMA_PeripheralBaseAddr = CONVERT_PTR_UINT(&SPACEMOUSE_SERIAL_USART->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = CONVERT_PTR_UINT(spacemouseDMAFifo.buffer());
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = spacemouseDMAFifo.size();
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(SPACEMOUSE_DMA_Stream_RX, &DMA_InitStructure);
  USART_DMACmd(SPACEMOUSE_SERIAL_USART, USART_DMAReq_Rx, ENABLE);
  USART_Cmd(SPACEMOUSE_SERIAL_USART, ENABLE);
  DMA_Cmd(SPACEMOUSE_DMA_Stream_RX, ENABLE);

  spacemouse_startstreaming();
}

void Parse_Character(STRUCT_SPACEMOUSE *spacemouseBuffer, unsigned char ch)
{
  static volatile bool parse_lock = false;

  if (parse_lock) return;
  parse_lock = true;

  switch (spacemouseBuffer->parsestate) {
    case SM_START: {
      if (ch == SPACEMOUSE_PROTO_HEADER) {
        spacemouseBuffer->head = SPACEMOUSE_PROTO_HEADER;
        spacemouseBuffer->parsestate = SM_DATA;
        spacemouseBuffer->msg_OK = false;
        spacemouseBuffer->dataIndex = 0;
      }
      break;
    }
    case SM_DATA: {
      spacemouseBuffer->data[spacemouseBuffer->dataIndex++] = ch;
      if (spacemouseBuffer->dataIndex >= SPACEMOUSE_LENGTH_DATA) {
        spacemouseBuffer->checkSum = 0;
        spacemouseBuffer->dataIndex = 0;
        spacemouseBuffer->parsestate = SM_CHECKSUM;
      }
      break;
    }
    case SM_CHECKSUM: {
      spacemouseBuffer->checkSum |= ch << ((spacemouseBuffer->dataIndex++) * 7);
      if (spacemouseBuffer->dataIndex >= 2) {
        spacemouseBuffer->dataIndex = 0;
        spacemouseBuffer->parsestate = SM_FOOTER;
      }
      break;
    }
    case SM_FOOTER: {
      if ((ch == SPACEMOUSE_PROTO_FOOTER) ||
          (spacemouseBuffer->checkSum ==
          calc_checksum((void *)&spacemouseBuffer->head, SPACEMOUSE_LENGTH_HEADER + SPACEMOUSE_LENGTH_DATA))) {
        spacemouseBuffer->msg_OK = true;
        goto Label_restart;
      } else {
        goto Label_error;
      }
      break;
    }
  }

  goto exit;

Label_error:
Label_restart:
  spacemouseBuffer->parsestate = SM_START;
exit:
  parse_lock = false;
  return;
}

/* Run it in 10ms timer routine */
void spacemouse_loop(void)
{
    uint8_t byte;

    while(SpaceMouseGetByte(&byte))
    {
        Parse_Character(&SpaceMouseProtocol, byte);
        if ( SpaceMouseProtocol.msg_OK )
        {
            SpaceMouseProtocol.msg_OK = false;
            for ( uint8_t channel = 0; channel < SPACEMOUSE_CHANNEL_COUNT; channel++ )
            {
              // The values are 7-bit in LSByte and MSByte only. Set MSBit is reserved for start, stop & commands.
              spacemouse_values[channel] = ((SpaceMouseProtocol.data[channel*2] << 7) + SpaceMouseProtocol.data[(channel*2)+1]) - SPACEMOUSE_OFFSET_VALUE;
            }
        }
    }
}
