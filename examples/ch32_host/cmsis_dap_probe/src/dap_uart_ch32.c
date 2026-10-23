/*!
 * @file dap_uart_ch32.c
 *
 * Non-blocking UART for the USB serial bridge: USART2 on PA2 (TX) / PA3
 * (RX). RX runs DMA1 channel 6 into a circular buffer; TX sends contiguous
 * chunks of a ring with DMA1 channel 7. No interrupts. Configuration after
 * XIVN1987/DAPLink's CH32V203 port (MIT).
 *
 * The core's HardwareSerial is not used: it has no receive buffer and its
 * write() goes to the debug UART.
 *
 * MIT license.
 */

#if defined(CH32V20x)

#include "dap_uart_ch32.h"
#include "ch32v20x.h"

#define RX_SIZE 512U
#define TX_SIZE 256U

static uint8_t _rx_buf[RX_SIZE] __attribute__((aligned(4)));
static uint8_t _tx_buf[TX_SIZE] __attribute__((aligned(4)));
static uint32_t _rx_tail = 0;    // next byte to read
static uint32_t _tx_head = 0;    // next free slot
static uint32_t _tx_tail = 0;    // first byte not yet sent
static uint32_t _tx_dma_len = 0; // bytes in the running TX DMA (0 = idle)
static uint8_t _rx_mask = 0xFF;
static uint8_t _running = 0;

static uint32_t rx_head(void) {
  // DMA counts down from RX_SIZE; right after a wrap it may read RX_SIZE.
  uint32_t remaining = DMA1_Channel6->CNTR;
  return (RX_SIZE - remaining) % RX_SIZE;
}

void dap_uart_end(void) {
  if (!_running) {
    return;
  }
  USART_Cmd(USART2, DISABLE);
  USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);
  DMA_Cmd(DMA1_Channel6, DISABLE);
  DMA_Cmd(DMA1_Channel7, DISABLE);
  _running = 0;
}

void dap_uart_begin(uint32_t baud, const dap_uart_format_t *fmt) {
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef usart;
  DMA_InitTypeDef dma;

  dap_uart_end();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  gpio.GPIO_Pin = GPIO_Pin_2; // PA2 = USART2_TX
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOA, &gpio);
  gpio.GPIO_Pin = GPIO_Pin_3; // PA3 = USART2_RX, pulled up when idle
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &gpio);

  // RX DMA: circular into _rx_buf
  DMA_DeInit(DMA1_Channel6);
  dma.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DATAR;
  dma.DMA_MemoryBaseAddr = (uint32_t)_rx_buf;
  dma.DMA_DIR = DMA_DIR_PeripheralSRC;
  dma.DMA_BufferSize = RX_SIZE;
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma.DMA_Mode = DMA_Mode_Circular;
  dma.DMA_Priority = DMA_Priority_High;
  dma.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &dma);

  // TX DMA: configured now, started per chunk in dap_uart_task()
  DMA_DeInit(DMA1_Channel7);
  dma.DMA_MemoryBaseAddr = (uint32_t)_tx_buf;
  dma.DMA_DIR = DMA_DIR_PeripheralDST;
  dma.DMA_BufferSize = 1;
  dma.DMA_Mode = DMA_Mode_Normal;
  DMA_Init(DMA1_Channel7, &dma);

  // Frame format
  usart.USART_BaudRate = baud;
  usart.USART_WordLength =
      fmt->word9 ? USART_WordLength_9b : USART_WordLength_8b;
  usart.USART_Parity = fmt->parity == DAP_CDC_PARITY_ODD    ? USART_Parity_Odd
                       : fmt->parity == DAP_CDC_PARITY_EVEN ? USART_Parity_Even
                                                            : USART_Parity_No;
  usart.USART_StopBits = fmt->stop == DAP_CDC_STOP_2     ? USART_StopBits_2
                         : fmt->stop == DAP_CDC_STOP_1_5 ? USART_StopBits_1_5
                                                         : USART_StopBits_1;
  usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &usart);
  _rx_mask = fmt->rx_mask;

  _rx_tail = 0;
  _tx_head = _tx_tail = 0;
  _tx_dma_len = 0;

  USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
  DMA_Cmd(DMA1_Channel6, ENABLE);
  USART_Cmd(USART2, ENABLE);
  _running = 1;
}

uint32_t dap_uart_rx_available(void) {
  if (!_running) {
    return 0;
  }
  return (rx_head() + RX_SIZE - _rx_tail) % RX_SIZE;
}

uint32_t dap_uart_read(uint8_t *buf, uint32_t max) {
  uint32_t n = dap_uart_rx_available();
  if (n > max) {
    n = max;
  }
  for (uint32_t i = 0; i < n; i++) {
    buf[i] = _rx_buf[_rx_tail] & _rx_mask;
    _rx_tail = (_rx_tail + 1) % RX_SIZE;
  }
  return n;
}

uint32_t dap_uart_tx_free(void) {
  // One slot stays empty to tell "full" from "empty".
  return (_tx_tail + TX_SIZE - _tx_head - 1) % TX_SIZE;
}

uint32_t dap_uart_write(const uint8_t *buf, uint32_t len) {
  uint32_t n = dap_uart_tx_free();
  if (n > len) {
    n = len;
  }
  for (uint32_t i = 0; i < n; i++) {
    _tx_buf[_tx_head] = buf[i];
    _tx_head = (_tx_head + 1) % TX_SIZE;
  }
  return n;
}

void dap_uart_task(void) {
  if (!_running) {
    return;
  }
  if (_tx_dma_len) {
    if (DMA1_Channel7->CNTR != 0) {
      return; // still sending
    }
    _tx_tail = (_tx_tail + _tx_dma_len) % TX_SIZE;
    _tx_dma_len = 0;
  }
  if (_tx_head == _tx_tail) {
    return; // nothing to send
  }
  // Largest contiguous run from the tail (stop at the end of the buffer).
  uint32_t len =
      (_tx_head > _tx_tail) ? (_tx_head - _tx_tail) : (TX_SIZE - _tx_tail);
  DMA_Cmd(DMA1_Channel7, DISABLE);
  DMA1_Channel7->MADDR = (uint32_t)&_tx_buf[_tx_tail];
  DMA1_Channel7->CNTR = len;
  _tx_dma_len = len;
  DMA_Cmd(DMA1_Channel7, ENABLE);
}

#endif // CH32V20x
