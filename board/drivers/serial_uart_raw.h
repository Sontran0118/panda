// Raw blocking UART send/recv on USART2 for the panda-over-serial transport.
// USART2 on the Nucleo-F446RE is wired to the ST-Link Virtual COM Port (PA2=TX, PA3=RX).
// APB1 = 45 MHz. Baud is set in peripherals.h init; here we just do byte I/O + timeouts.
//
// microsecond timing uses the panda's microsecond_timer (TIM). We poll SR flags.

#ifndef SERIAL_UART
#define SERIAL_UART USART2
#endif

// millisecond deadline helper using the panda's microsecond_timer (already used elsewhere)
static bool uart_wait_flag(volatile uint32_t *sr, uint32_t flag, uint32_t timeout_ms) {
  uint32_t start = microsecond_timer_get();
  while ((*sr & flag) == 0U) {
    // ORE recovery: once the overrun flag latches, the USART stops asserting
    // RXNE entirely. It is cleared by a read of SR followed by a read of DR.
    // Without this the receive path wedges permanently after one overrun.
    if ((*sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U) {
      (void)*sr;
      (void)SERIAL_UART->DR;
    }
    if ((microsecond_timer_get() - start) > (timeout_ms * 1000U)) { return false; }
  }
  return true;
}

void uart_send_raw(const uint8_t *d, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    if (!uart_wait_flag(&SERIAL_UART->SR, USART_SR_TXE, 10U)) { return; }
    SERIAL_UART->DR = d[i];
  }
  // wait for transmission complete so the line is idle before we listen again
  (void)uart_wait_flag(&SERIAL_UART->SR, USART_SR_TC, 10U);
}

bool uart_recv_raw(uint8_t *d, uint16_t len, uint32_t timeout_ms) {
  for (uint16_t i = 0; i < len; i++) {
    if (!uart_wait_flag(&SERIAL_UART->SR, USART_SR_RXNE, timeout_ms)) { return false; }
    d[i] = (uint8_t)(SERIAL_UART->DR & 0xFFU);
  }
  return true;
}

// Read a single byte with timeout. Returns false on timeout (nothing consumed).
static bool uart_recv_byte(uint8_t *b, uint32_t timeout_ms) {
  if (!uart_wait_flag(&SERIAL_UART->SR, USART_SR_RXNE, timeout_ms)) { return false; }
  *b = (uint8_t)(SERIAL_UART->DR & 0xFFU);
  return true;
}

// Drain anything sitting in the RX register (used to resync after a bad frame).
static void uart_flush_rx(void) {
  uint32_t guard = 0U;
  while (((SERIAL_UART->SR & USART_SR_RXNE) != 0U) && (guard < 4096U)) {
    (void)SERIAL_UART->DR;
    guard++;
  }
  // clear ORE/FE/NE/PE unconditionally: read SR then DR
  (void)SERIAL_UART->SR;
  (void)SERIAL_UART->DR;
}
