// UART transport for the panda protocol (Nucleo-F446 over ST-Link VCP serial).
// Faithful port of the SPI transport framing (board/drivers/spi.h): same 6-byte header,
// same endpoints (0=control, 1/0x81=can_read, 2=ep2_write, 3=can_write), same XOR checksum,
// same comms_* dispatch — only the transport is UART instead of SPI DMA.
//
// Framing (host<->device):
//   HEADER  = [SYNC=0x5A][endpoint][mosi_len:u16 LE][miso_len:u16 LE][checksum:1]  (7 bytes)
//   host sends header; device replies 1 byte HACK(0x79)/NACK(0x1F).
//   if endpoint carries mosi data: host then sends [mosi_len data][data_checksum:1];
//     device replies [HACK][resp_len:u16][resp data][resp_checksum].
// Uses the same constants as spi.h so pandad's serial handle can share them.

#define SERIAL_SYNC_BYTE   0x5AU
#define SERIAL_HACK        0x79U
#define SERIAL_NACK        0x1FU
#define SERIAL_CHECKSUM_START 0xABU
#define SERIAL_HEADER_SIZE 7U
#define SERIAL_BUF_SIZE    2048U

static uint8_t ser_rx[SERIAL_BUF_SIZE];
static uint8_t ser_tx[SERIAL_BUF_SIZE];

static uint8_t serial_checksum(const uint8_t *data, uint16_t len) {
  uint8_t c = SERIAL_CHECKSUM_START;
  for (uint16_t i = 0; i < len; i++) { c ^= data[i]; }
  return c;
}

// blocking UART helpers (defined by the ll uart layer: uart_send_byte / uart_recv_byte)
extern void uart_send_raw(const uint8_t *d, uint16_t len);
extern bool uart_recv_raw(uint8_t *d, uint16_t len, uint32_t timeout_ms);

// process exactly one request/response transaction. Called from the main loop.
void serial_comms_tick(void) {
  // 1) hunt for the SYNC byte one byte at a time. Never consume a fixed block
  //    before we are aligned, or a single dropped/extra byte desyncs forever.
  if (!uart_recv_byte(&ser_rx[0], 2U)) { return; }
  if (ser_rx[0] != SERIAL_SYNC_BYTE) { return; }   // not aligned: drop 1 byte, retry next tick

  // read the remaining 6 header bytes. Keep this SHORT: at 1.5 Mbaud a full
  // header takes ~47 us, so 3 ms is generous. A long timeout here parks the
  // whole main loop when the host sends a stray SYNC byte.
#define SERIAL_HDR_TIMEOUT_MS 3U
  if (!uart_recv_raw(&ser_rx[1], SERIAL_HEADER_SIZE - 1U, SERIAL_HDR_TIMEOUT_MS)) {
    uart_flush_rx();
    return;
  }
  if (serial_checksum(ser_rx, SERIAL_HEADER_SIZE) != 0U) {
    uart_flush_rx();
    uint8_t nack = SERIAL_NACK; uart_send_raw(&nack, 1U); return;
  }
  uint8_t  endpoint  = ser_rx[1];
  uint16_t mosi_len  = (uint16_t)ser_rx[2] | ((uint16_t)ser_rx[3] << 8);
  uint16_t miso_len  = (uint16_t)ser_rx[4] | ((uint16_t)ser_rx[5] << 8);
  if (mosi_len > (SERIAL_BUF_SIZE - SERIAL_HEADER_SIZE - 1U)) {
    uart_flush_rx();
    uint8_t nack = SERIAL_NACK; uart_send_raw(&nack, 1U); return;
  }

  // 2) ack header, then read mosi data (if any) + its checksum byte
  uint8_t hack = SERIAL_HACK; uart_send_raw(&hack, 1U);
  if (mosi_len > 0U) {
    if (!uart_recv_raw(&ser_rx[SERIAL_HEADER_SIZE], mosi_len + 1U, 25U)) { uart_flush_rx(); return; }
    if (serial_checksum(&ser_rx[SERIAL_HEADER_SIZE], mosi_len + 1U) != 0U) {
      uart_flush_rx();
      uint8_t nack = SERIAL_NACK; uart_send_raw(&nack, 1U); return;
    }
  }

  // 3) dispatch to comms_* handlers (same as spi.h). resp data goes at ser_tx[3].
  uint16_t resp_len = 0U; bool ack = false;
  if (endpoint == 0U) {
    if (mosi_len >= sizeof(ControlPacket_t)) {
      ControlPacket_t ctrl = {0};
      (void)memcpy((uint8_t*)&ctrl, &ser_rx[SERIAL_HEADER_SIZE], sizeof(ControlPacket_t));
      resp_len = comms_control_handler(&ctrl, &ser_tx[3]); ack = true;
    }
  } else if ((endpoint == 1U) || (endpoint == 0x81U)) {
    if (mosi_len == 0U) { resp_len = comms_can_read(&ser_tx[3], miso_len); ack = true; }
  } else if (endpoint == 2U) {
    comms_endpoint2_write(&ser_rx[SERIAL_HEADER_SIZE], mosi_len); ack = true;
  } else if (endpoint == 3U) {
    if (mosi_len > 0U) { comms_can_write(&ser_rx[SERIAL_HEADER_SIZE], mosi_len); ack = true; }
  } else {
    // unknown endpoint
  }

  // 4) respond: [HACK/NACK][resp_len:u16][resp data][checksum]
  if (ack) {
    ser_tx[0] = SERIAL_HACK;
    ser_tx[1] = resp_len & 0xFFU;
    ser_tx[2] = (resp_len >> 8) & 0xFFU;
    uint16_t total = 3U + resp_len;
    ser_tx[total] = serial_checksum(ser_tx, total);
    uart_send_raw(ser_tx, total + 1U);
  } else {
    uint8_t nack = SERIAL_NACK; uart_send_raw(&nack, 1U);
  }
}

void serial_comms_init(void) {
  // bring up USART2 (PA2/PA3 -> ST-Link VCP) and clear any junk in the RX path
  usart2_init();
  uart_flush_rx();
}
