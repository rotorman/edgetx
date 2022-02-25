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

#if defined(SPACEMOUSE_U3)
  DMAFifo<4*SPACEMOUSE_LENGTH_TELEGRAM> spacemouseu3DMAFifo __DMA (SPACEMOUSEU3_DMA_Stream_RX);
#endif
#if defined(SPACEMOUSE_U6)
  DMAFifo<4*SPACEMOUSE_LENGTH_TELEGRAM> spacemouseu6DMAFifo __DMA (SPACEMOUSEU6_DMA_Stream_RX);
#endif
//unsigned char SpaceMouseCmd;

#if defined(SPACEMOUSE_U3)
  STRUCT_SPACEMOUSE SpaceMouseProtocolU3 = { 0 };
#endif
#if defined(SPACEMOUSE_U6)
  STRUCT_SPACEMOUSE SpaceMouseProtocolU6 = { 0 };
#endif

#if defined(SPACEMOUSE_U3)
  signed short spacemouseu3_values[SPACEMOUSE_CHANNEL_COUNT] = { 0 };
#endif
#if defined(SPACEMOUSE_U6)
  signed short spacemouseu6_values[SPACEMOUSE_CHANNEL_COUNT] = { 0 };
#endif

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

#if defined(SPACEMOUSE_U3)
uint16_t get_spacemouseu3_adc_value(uint8_t ch)
{
  if (ch >= SPACEMOUSE_CHANNEL_COUNT) {
    return 0;
  }

  return spacemouseu3_values[ch] + SPACEMOUSE_OUTPUT_OFFSET;
}
#endif

#if defined(SPACEMOUSE_U6)
uint16_t get_spacemouseu6_adc_value(uint8_t ch)
{
  if (ch >= SPACEMOUSE_CHANNEL_COUNT) {
    return 0;
  }

  return spacemouseu6_values[ch] + SPACEMOUSE_OUTPUT_OFFSET;
}
#endif

#if defined(SPACEMOUSE_U3)
uint8_t SpaceMouseU3GetByte(uint8_t * byte)
{
  return spacemouseu3DMAFifo.pop(*byte);
}
#endif

#if defined(SPACEMOUSE_U6)
uint8_t SpaceMouseU6GetByte(uint8_t * byte)
{
  return spacemouseu6DMAFifo.pop(*byte);
}
#endif

#if defined(SPACEMOUSE_U3)
void spacemouseu3_tare( void )
{
  USART_SendData(SPACEMOUSEU3_SERIAL_USART, SPACEMOUSE_CMD_TARE);
}
#endif

#if defined(SPACEMOUSE_U6)
void spacemouseu6_tare( void )
{
  USART_SendData(SPACEMOUSEU6_SERIAL_USART, SPACEMOUSE_CMD_TARE);
}
#endif

#if defined(SPACEMOUSE_U3)
void spacemouseu3_startstreaming( void )
{
  USART_SendData(SPACEMOUSEU3_SERIAL_USART, SPACEMOUSE_CMD_STARTSTREAMING);
}
#endif

#if defined(SPACEMOUSE_U6)
void spacemouseu6_startstreaming( void )
{
  USART_SendData(SPACEMOUSEU6_SERIAL_USART, SPACEMOUSE_CMD_STARTSTREAMING);
}
#endif

#if defined(SPACEMOUSE_U3)
void spacemouseu3_reqsingledata( void )
{
  USART_SendData(SPACEMOUSEU3_SERIAL_USART, SPACEMOUSE_CMD_REQSINGLEDATA);
}
#endif

#if defined(SPACEMOUSE_U6)
void spacemouseu6_reqsingledata( void )
{
  USART_SendData(SPACEMOUSEU6_SERIAL_USART, SPACEMOUSE_CMD_REQSINGLEDATA);
}
#endif

#if defined(SPACEMOUSE_U3)
void spacemouseu3_init()
{
  USART_DeInit(SPACEMOUSEU3_SERIAL_USART);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPACEMOUSEU3_SERIAL_RX_DMA_Stream_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_PinAFConfig(SPACEMOUSEU3_SERIAL_GPIO, SPACEMOUSEU3_SERIAL_RX_GPIO_PinSource, SPACEMOUSEU3_SERIAL_GPIO_AF);
  GPIO_PinAFConfig(SPACEMOUSEU3_SERIAL_GPIO, SPACEMOUSEU3_SERIAL_TX_GPIO_PinSource, SPACEMOUSEU3_SERIAL_GPIO_AF);

  GPIO_InitStructure.GPIO_Pin = SPACEMOUSEU3_SERIAL_GPIO_PIN_TX | SPACEMOUSEU3_SERIAL_GPIO_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(SPACEMOUSEU3_SERIAL_GPIO, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = SPACEMOUSE_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(SPACEMOUSEU3_SERIAL_USART, &USART_InitStructure);

  DMA_Cmd(SPACEMOUSEU3_DMA_Stream_RX, DISABLE);
  USART_DMACmd(SPACEMOUSEU3_SERIAL_USART, USART_DMAReq_Rx, DISABLE);
  DMA_DeInit(SPACEMOUSEU3_DMA_Stream_RX);

  DMA_InitTypeDef DMA_InitStructure;
  spacemouseu3DMAFifo.clear();

  USART_ITConfig(SPACEMOUSEU3_SERIAL_USART, USART_IT_RXNE, DISABLE);
  USART_ITConfig(SPACEMOUSEU3_SERIAL_USART, USART_IT_TXE, DISABLE);

  // Turn on AUX1 power
  GPIO_InitStructure.GPIO_Pin = SPACEMOUSEU3_SERIAL_PWR_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPACEMOUSEU3_SERIAL_PWR_GPIO, &GPIO_InitStructure);
  GPIO_SetBits(SPACEMOUSEU3_SERIAL_PWR_GPIO, SPACEMOUSEU3_SERIAL_PWR_GPIO_PIN);

  // Configure and enable DMA for receive
  DMA_InitStructure.DMA_Channel = SPACEMOUSEU3_SERIAL_DMA_Channel;
  DMA_InitStructure.DMA_PeripheralBaseAddr = CONVERT_PTR_UINT(&SPACEMOUSEU3_SERIAL_USART->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = CONVERT_PTR_UINT(spacemouseu3DMAFifo.buffer());
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = spacemouseu3DMAFifo.size();
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
  DMA_Init(SPACEMOUSEU3_DMA_Stream_RX, &DMA_InitStructure);
  USART_DMACmd(SPACEMOUSEU3_SERIAL_USART, USART_DMAReq_Rx, ENABLE);
  USART_Cmd(SPACEMOUSEU3_SERIAL_USART, ENABLE);
  DMA_Cmd(SPACEMOUSEU3_DMA_Stream_RX, ENABLE);

  //spacemouse_startstreaming();
}
#endif

#if defined(SPACEMOUSE_U6)
void spacemouseu6_init()
{
  USART_DeInit(SPACEMOUSEU6_SERIAL_USART);

  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = SPACEMOUSEU6_SERIAL_RX_DMA_Stream_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_PinAFConfig(SPACEMOUSEU6_SERIAL_GPIO, SPACEMOUSEU6_SERIAL_RX_GPIO_PinSource, SPACEMOUSEU6_SERIAL_GPIO_AF);
  GPIO_PinAFConfig(SPACEMOUSEU6_SERIAL_GPIO, SPACEMOUSEU6_SERIAL_TX_GPIO_PinSource, SPACEMOUSEU6_SERIAL_GPIO_AF);

  GPIO_InitStructure.GPIO_Pin = SPACEMOUSEU6_SERIAL_GPIO_PIN_TX | SPACEMOUSEU6_SERIAL_GPIO_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(SPACEMOUSEU6_SERIAL_GPIO, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = SPACEMOUSE_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(SPACEMOUSEU6_SERIAL_USART, &USART_InitStructure);

  DMA_Cmd(SPACEMOUSEU6_DMA_Stream_RX, DISABLE);
  USART_DMACmd(SPACEMOUSEU6_SERIAL_USART, USART_DMAReq_Rx, DISABLE);
  DMA_DeInit(SPACEMOUSEU6_DMA_Stream_RX);

  DMA_InitTypeDef DMA_InitStructure;
  spacemouseu6DMAFifo.clear();

  USART_ITConfig(SPACEMOUSEU6_SERIAL_USART, USART_IT_RXNE, DISABLE);
  USART_ITConfig(SPACEMOUSEU6_SERIAL_USART, USART_IT_TXE, DISABLE);

  // Turn on AUX2 power
  GPIO_InitStructure.GPIO_Pin = SPACEMOUSEU6_SERIAL_PWR_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPACEMOUSEU6_SERIAL_PWR_GPIO, &GPIO_InitStructure);
  GPIO_SetBits(SPACEMOUSEU6_SERIAL_PWR_GPIO, SPACEMOUSEU6_SERIAL_PWR_GPIO_PIN);

  // Configure and enable DMA for receive
  DMA_InitStructure.DMA_Channel = SPACEMOUSEU6_SERIAL_DMA_Channel;
  DMA_InitStructure.DMA_PeripheralBaseAddr = CONVERT_PTR_UINT(&SPACEMOUSEU6_SERIAL_USART->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = CONVERT_PTR_UINT(spacemouseu6DMAFifo.buffer());
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = spacemouseu6DMAFifo.size();
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
  DMA_Init(SPACEMOUSEU6_DMA_Stream_RX, &DMA_InitStructure);
  USART_DMACmd(SPACEMOUSEU6_SERIAL_USART, USART_DMAReq_Rx, ENABLE);
  USART_Cmd(SPACEMOUSEU6_SERIAL_USART, ENABLE);
  DMA_Cmd(SPACEMOUSEU6_DMA_Stream_RX, ENABLE);

  //spacemouse_startstreaming();
}
#endif

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

#if defined(SPACEMOUSE_U3)
    while(SpaceMouseU3GetByte(&byte))
    {
        Parse_Character(&SpaceMouseProtocolU3, byte);
        if ( SpaceMouseProtocolU3.msg_OK )
        {
            SpaceMouseProtocolU3.msg_OK = false;
            for ( uint8_t channel = 0; channel < SPACEMOUSE_CHANNEL_COUNT; channel++ )
            {
              // The values are 7-bit in LSByte and MSByte only. Set MSBit is reserved for start, stop & commands.
              spacemouseu3_values[channel] = ((SpaceMouseProtocolU3.data[channel*2] << 7) + SpaceMouseProtocolU3.data[(channel*2)+1]) - SPACEMOUSE_INPUT_OFFSET;
            }
        }
    }
    spacemouseu3_reqsingledata();
#endif

#if defined(SPACEMOUSE_U6)
    while(SpaceMouseU6GetByte(&byte))
    {
        Parse_Character(&SpaceMouseProtocolU6, byte);
        if ( SpaceMouseProtocolU6.msg_OK )
        {
            SpaceMouseProtocolU6.msg_OK = false;
            for ( uint8_t channel = 0; channel < SPACEMOUSE_CHANNEL_COUNT; channel++ )
            {
              // The values are 7-bit in LSByte and MSByte only. Set MSBit is reserved for start, stop & commands.
              spacemouseu6_values[channel] = ((SpaceMouseProtocolU6.data[channel*2] << 7) + SpaceMouseProtocolU6.data[(channel*2)+1]) - SPACEMOUSE_INPUT_OFFSET;
            }
        }
    }
    spacemouseu6_reqsingledata();
#endif
}
