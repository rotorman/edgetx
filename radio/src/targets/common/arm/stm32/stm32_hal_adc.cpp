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
#include "hal/adc_driver.h"
#include "stm32_hal_adc.h"

static void adc_init_pins()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

#if defined(ADC_GPIOA_PINS)
  GPIO_InitStructure.GPIO_Pin = ADC_GPIOA_PINS;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#if defined(ADC_GPIOB_PINS)
  GPIO_InitStructure.GPIO_Pin = ADC_GPIOB_PINS;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#if defined(ADC_GPIOC_PINS)
  GPIO_InitStructure.GPIO_Pin = ADC_GPIOC_PINS;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#if defined(ADC_GPIOF_PINS)
  GPIO_InitStructure.GPIO_Pin = ADC_GPIOF_PINS;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
#endif  
}

static void adc_setup_scan_mode(ADC_TypeDef* ADCx, uint8_t nconv)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_StructInit(&ADC_InitStructure);

  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = nconv;
  
  ADC_Init(ADCx, &ADC_InitStructure);

  // enable ADC_MAIN
  ADC_Cmd(ADCx, ENABLE);                            // ADC_CR2_ADON
  // enable DMA for ADC
  ADC_DMACmd(ADCx, ENABLE);                         // ADC_CR2_DMA
  ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE); // ADC_CR2_DDS
}

stm32_hal_adc_channel ADC_MAIN_channels[] = {
    {ADC_CHANNEL_STICK_LH, ADC_SAMPTIME},
    {ADC_CHANNEL_STICK_LV, ADC_SAMPTIME},
    {ADC_CHANNEL_STICK_RV, ADC_SAMPTIME},
    {ADC_CHANNEL_STICK_RH, ADC_SAMPTIME},
#if defined(PCBX10)
    { ADC_CHANNEL_POT1,    ADC_SAMPTIME },
    { ADC_CHANNEL_POT2,    ADC_SAMPTIME },
    { ADC_CHANNEL_POT3,    ADC_SAMPTIME },
    { ADC_CHANNEL_EXT1,    ADC_SAMPTIME },
    { ADC_CHANNEL_EXT2,    ADC_SAMPTIME },
    { ADC_CHANNEL_SLIDER1, ADC_SAMPTIME },
    { ADC_CHANNEL_SLIDER2, ADC_SAMPTIME },
    { ADC_CHANNEL_BATT,    ADC_SAMPTIME }
#else
#if defined(RADIO_T8) || defined(RADIO_TLITE)
    // fake channels to fill unsused POT1/POT2
    {0, 0},
    {0, 0},
#elif defined(PCBX7) || defined(PCBXLITE)
    {ADC_CHANNEL_POT1, ADC_SAMPTIME},
    {ADC_CHANNEL_POT2, ADC_SAMPTIME},
#elif defined(PCBX9LITE)
    {ADC_CHANNEL_POT1, ADC_SAMPTIME},
#elif defined(PCBX9E)
    {ADC_CHANNEL_POT2, ADC_SAMPTIME},
    {ADC_CHANNEL_POT3, ADC_SAMPTIME},
    {ADC_CHANNEL_POT4, ADC_SAMPTIME},
    {ADC_CHANNEL_SLIDER3, ADC_SAMPTIME},
    {ADC_CHANNEL_SLIDER4, ADC_SAMPTIME},
#elif defined(PCBX9D) || defined(PCBX9DP)
    {ADC_CHANNEL_POT1,    ADC_SAMPTIME},
    {ADC_CHANNEL_POT2,    ADC_SAMPTIME},
    {ADC_CHANNEL_POT3,    ADC_SAMPTIME},
    {ADC_CHANNEL_SLIDER1, ADC_SAMPTIME},
    {ADC_CHANNEL_SLIDER2, ADC_SAMPTIME},
#endif
    {ADC_CHANNEL_BATT, ADC_SAMPTIME},
    {ADC_Channel_Vbat, ADC_SAMPTIME}
#endif
};

static const stm32_hal_adc_channel* ADC_MAIN_get_channels()
{
#if NUM_PWMSTICKS > 0
  if (STICKS_PWM_ENABLED())
    return ADC_MAIN_channels + 4;
#endif
  return ADC_MAIN_channels;
}

static uint8_t ADC_MAIN_get_nconv()
{
  return NUM_ANALOGS_ADC; // based on STICKS_PWM_ENABLED()
}

#if defined(ADC_EXT)
#if defined(PCBX9E)

static const stm32_hal_adc_channel ADC_EXT_channels[] = {
    {ADC_CHANNEL_POT1, ADC_SAMPTIME},
    {ADC_CHANNEL_SLIDER1, ADC_SAMPTIME},
    {ADC_CHANNEL_SLIDER2, ADC_SAMPTIME}
};

static uint8_t ADC_EXT_get_nconv() { return NUM_ANALOGS_ADC_EXT; }

#elif defined(PCBX10)

static const stm32_hal_adc_channel ADC_EXT_channels[] = {
  { ADC_Channel_Vbat, 1, ADC_SAMPTIME }
};

static uint8_t ADC_EXT_get_nconv() { return 1; }

#endif

static const stm32_hal_adc_channel* ADC_EXT_get_channels()
{
  return ADC_EXT_channels;
}

#endif

static uint16_t* ADC_MAIN_get_dma_buffer()
{
  return &adcValues[FIRST_ANALOG_ADC];
}

#if defined(ADC_EXT) && defined(ADC_EXT_DMA_Stream)
static uint16_t* ADC_EXT_get_dma_buffer()
{
  return adcValues + NUM_ANALOGS_ADC;
}
#endif

stm32_hal_adc ADC_hal_def[] = {
    {
      ADC_MAIN,
      ADC_DMA_Stream, ADC_MAIN_get_dma_buffer,
      ADC_MAIN_get_nconv, ADC_MAIN_get_channels
    },
#if defined(ADC_EXT)
#if defined(ADC_EXT_DMA_Stream)
    {
      ADC_EXT,
      ADC_EXT_DMA_Stream, ADC_EXT_get_dma_buffer,
      ADC_EXT_get_nconv, ADC_EXT_get_channels
    },
#else
    {
      ADC_EXT,
      nullptr, nullptr,
      ADC_EXT_get_nconv, ADC_EXT_get_channels
    },
#endif
#endif
    { nullptr, nullptr, nullptr, nullptr, nullptr },
};

static void adc_init_channels(ADC_TypeDef* adc, const stm32_hal_adc_channel* chan,
                              uint8_t nconv)
{
  if (!chan || !nconv) return;

  uint8_t rank = 1;
  while (nconv > 0) {
    ADC_RegularChannelConfig(adc, chan->adc_channel, rank, chan->sample_time);
    nconv--; rank++; chan++;
  }
}

static void adc_init_dma_stream(ADC_TypeDef* adc, DMA_Stream_TypeDef * dma_stream,
                                uint16_t* dest, uint8_t nconv)
{
  // setup DMA request
  dma_stream->CR = DMA_SxCR_PL | ADC_DMA_SxCR_CHSEL | DMA_SxCR_MSIZE_0 |
                   DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC;
  dma_stream->PAR = CONVERT_PTR_UINT(&adc->DR);
  dma_stream->M0AR = CONVERT_PTR_UINT(dest);
  dma_stream->NDTR = nconv;
  dma_stream->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0;
}

static void stm32_hal_adc_init()
{
  adc_init_pins();

  // Init common to all ADCs
  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonStructInit(&ADC_CommonInitStruct);
  ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInit(&ADC_CommonInitStruct);

  const stm32_hal_adc* adc_def = ADC_hal_def;
  while (adc_def->adc) {
    uint8_t nconv = 0;
    if (adc_def->get_nconv) {
      nconv = adc_def->get_nconv();
      if (nconv > 0) {
        adc_setup_scan_mode(adc_def->adc, nconv);

        // configure each channel
        if (adc_def->get_channels) {
          const stm32_hal_adc_channel* chan = adc_def->get_channels();
          adc_init_channels(adc_def->adc, chan, nconv);
        }

        if (adc_def->dma_stream && adc_def->get_dma_buffer) {
          uint16_t* dma_buffer = adc_def->get_dma_buffer();
          adc_init_dma_stream(adc_def->adc, adc_def->dma_stream, dma_buffer, nconv);
        }
      }
    }
    // move to next ADC definition
    adc_def++;
  }

  //TODO: move VBat & PWM sticks somewhere else.

  // Enable vbat sensor
  ADC_VBATCmd(ENABLE);

#if NUM_PWMSTICKS > 0
  if (STICKS_PWM_ENABLED()) {
    sticksPwmInit();
  }
#endif
}

#define DMA_Stream0_IT_MASK     (uint32_t)(DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | \
                                           DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | \
                                           DMA_LISR_TCIF0)
  
#define DMA_Stream4_IT_MASK     (uint32_t)(DMA_HISR_FEIF4 | DMA_HISR_DMEIF4 | \
                                           DMA_HISR_TEIF4 | DMA_HISR_HTIF4 | \
                                           DMA_HISR_TCIF4)

static void adc_dma_clear_flags(DMA_Stream_TypeDef * dma_stream)
{
  // no other choice, sorry for that...
  if (dma_stream == DMA2_Stream4) {
    /* Reset interrupt pending bits for DMA2 Stream4 */
    DMA2->HIFCR = DMA_Stream4_IT_MASK;

  } else if (dma_stream == DMA2_Stream0) {
    /* Reset interrupt pending bits for DMA2 Stream0 */
    DMA2->LIFCR = DMA_Stream0_IT_MASK;
  }
}

static void adc_start_dma_conversion(ADC_TypeDef* ADCx,
                                     DMA_Stream_TypeDef * dma_stream)
{
  dma_stream->CR &= ~DMA_SxCR_EN; // Disable DMA
  ADCx->SR &= ~(uint32_t)(ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR);
  adc_dma_clear_flags(dma_stream);

  dma_stream->CR |= DMA_SxCR_EN; // Enable DMA
  ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;  
}

static void adc_disable_dma(DMA_Stream_TypeDef * dma_stream)
{
  dma_stream->CR &= ~DMA_SxCR_EN; // Disable DMA stream
}

static void adc_start_single_conversion(ADC_TypeDef* ADCx)
{
  ADCx->SR &= ~(uint32_t)(ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR);
  ADCx->CR2 |= (uint32_t)ADC_CR2_SWSTART;  
}

static void stm32_hal_adc_start_read()
{
  const stm32_hal_adc* adc_def = ADC_hal_def;
  while (adc_def->adc) {
    uint8_t nconv = 0;
    if (adc_def->get_nconv) {
      nconv = adc_def->get_nconv();
      if (nconv > 0) {
        if (adc_def->dma_stream && adc_def->get_dma_buffer) {
          adc_start_dma_conversion(adc_def->adc, adc_def->dma_stream);
        } else if (nconv == 1){
          adc_start_single_conversion(adc_def->adc);
        }
      }
    }
    // move to next ADC definition
    adc_def++;
  }
}

static void stm32_hal_adc_wait_completion()
{
  //TODO:
  // - replace with IRQ trigger (both)
  // - move RTC batt reading somewhere else
  
#if defined(ADC_EXT) && defined(ADC_EXT_DMA_Stream)
  // Wait for all ADCs to complete
  for (unsigned int i=0; i<10000; i++) {
    if (ADC_TRANSFER_COMPLETE() && ADC_EXT_TRANSFER_COMPLETE()) {
      break;
    }
  }
  //TODO: move into completion IRQ trigger
  adc_disable_dma(ADC_DMA_Stream);
  adc_disable_dma(ADC_EXT_DMA_Stream);
#else
  // Wait only for the main ADC, and hope others are done as well
  for (unsigned int i = 0; i < 10000; i++) {
    if (ADC_TRANSFER_COMPLETE()) {
      break;
    }
  }
  //TODO: move into completion IRQ trigger
  adc_disable_dma(ADC_DMA_Stream);

#if defined(ADC_EXT) && !defined(ADC_EXT_DMA_Stream)
  if (isVBatBridgeEnabled()) {
    rtcBatteryVoltage = ADC_EXT->DR;
  }
#endif
#endif
}

const etx_hal_adc_driver_t stm32_hal_adc_driver = {
  stm32_hal_adc_init,
  stm32_hal_adc_start_read,
  stm32_hal_adc_wait_completion
};
