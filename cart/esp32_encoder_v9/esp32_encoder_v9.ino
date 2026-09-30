// =============================================================================
// ESP32 #1 — Encoder Firmware
// Version: 9.2
// Description: Reads motor and pendulum quadrature encoders via PCNT hardware.
//              Serves encoder counts to Raspberry Pi over SPI slave interface.
//              Sends motor and pendulum position to ESP32 #2 over UART2 for
//              homing and standalone swing-up/balance.
//              Replaces MCP23017/PCI-1711 data path entirely.
//
// Changes in 9.2 (requires ESP32 #2 (cart_control) v10.0 — flash both boards):
//   - UART packet also carries the pendulum count (7 bytes, every 2ms).
//   - PCNT counts accumulate past the 16-bit hardware limit instead of
//     resetting to 0, so the pendulum angle stays correct after many turns.
//   - ENC_DIAG channel diagnostic (per-channel edge counts). Found a dead
//     level-shifter channel on pendulum B: the count only moved 0 <-> 1.
//
// Changes in 9.1:
//   - UART packet data bytes are 7-bit (4-byte packet). In 9.0 any position
//     from -256 to -1 put 0xFF in a data byte, which ESP32 #2 took as a packet
//     header — its position froze in that range and centering hunted.
//     Requires ESP32 #2 (cart_control) v9.8 or later.
//   - Fixed compile error in check_uart_rx (pcnt_unit_clear_count{...}).
//
// UART Packet to ESP32 #2 (7 bytes, every 2ms):
//   0xFF | m[6:0] | m[13:7] | m[15:14] | p[6:0] | p[13:7] | p[15:14]
//   m = int16 motor count, p = low 16 bits of the pendulum count (ESP32 #2
//   unwraps it). Data bytes are 7-bit so they never equal the 0xFF header.
//
// SPI Packet (10 bytes, CS-framed):
//   Byte 0-3 : motor count     (int32, little-endian)
//   Byte 4-7 : pendulum count  (int32, little-endian)
//   Byte 8   : status flags    (bit0=system_ready, bit1=homing_complete)
//   Byte 9   : XOR checksum    (bytes 0-8)
//
// Pin Assignments:
//   GPIO 39 : Motor encoder A
//   GPIO 36 : Motor encoder B
//   GPIO 35 : Pendulum encoder A
//   GPIO 34 : Pendulum encoder B
//   GPIO 19 : SPI MISO (data to Pi)
//   GPIO 23 : SPI MOSI (not used, Pi sends nothing)
//   GPIO 18 : SPI SCLK
//   GPIO  5 : SPI CS
//   GPIO 17 : UART2 TX to ESP32 #2
//   GPIO 16 : UART2 RX from ESP32 #2
// =============================================================================

#include "driver/pulse_cnt.h"
#include "driver/spi_slave.h"
#include <WiFi.h>

// --- Encoder Pins ---
#define MOTOR_ENC_A     39
#define MOTOR_ENC_B     36
#define PEND_ENC_A      35
#define PEND_ENC_B      34

// --- SPI Slave Pins ---
#define SPI_MISO        19
#define SPI_MOSI        23
#define SPI_SCLK        18
#define SPI_CS           5

// --- UART2 Pins (to ESP32 #2) ---
#define UART_TX         17
#define UART_RX         16

// --- UART timing --- (7 bytes per 2ms uses ~30% of 115200 baud)
#define UART_INTERVAL_MS  2

// --- SPI packet size ---
#define SPI_PACKET_BYTES  10

// --- Encoder channel diagnostic ---
// 1 = poll every encoder pin in loop() and print per-channel edge counts and
// levels once a second. Turn a shaft slowly by hand: a healthy encoder shows
// edges on BOTH A and B. A count that only moves between 0 and 1 means one
// channel is not toggling (or A and B carry the same signal).
#define ENC_DIAG          0

// --- Status flag bits ---
#define FLAG_SYSTEM_READY    0x01
#define FLAG_HOMING_COMPLETE 0x02

// --- PCNT handles ---
pcnt_unit_handle_t motor_pcnt = NULL;
pcnt_unit_handle_t pend_pcnt  = NULL;

// --- State ---
volatile bool system_ready    = false;
volatile bool homing_complete = false;

// --- SPI DMA buffers (must be DMA-capable memory, 32-bit aligned) ---
DMA_ATTR uint8_t spi_tx_buf[SPI_PACKET_BYTES];
DMA_ATTR uint8_t spi_rx_buf[SPI_PACKET_BYTES];  // unused but required by driver

// =============================================================================
// Build SPI transmit packet from current encoder counts
// =============================================================================
void build_spi_packet() {
  int motor_count = 0;
  int pend_count  = 0;
  pcnt_unit_get_count(motor_pcnt, &motor_count);
  pcnt_unit_get_count(pend_pcnt,  &pend_count);

  int32_t motor = (int32_t)motor_count;
  int32_t pend  = (int32_t)pend_count;

  // Bytes 0-3: motor count (little-endian)
  spi_tx_buf[0] = (uint8_t)(motor & 0xFF);
  spi_tx_buf[1] = (uint8_t)((motor >> 8)  & 0xFF);
  spi_tx_buf[2] = (uint8_t)((motor >> 16) & 0xFF);
  spi_tx_buf[3] = (uint8_t)((motor >> 24) & 0xFF);

  // Bytes 4-7: pendulum count (little-endian)
  spi_tx_buf[4] = (uint8_t)(pend & 0xFF);
  spi_tx_buf[5] = (uint8_t)((pend >> 8)  & 0xFF);
  spi_tx_buf[6] = (uint8_t)((pend >> 16) & 0xFF);
  spi_tx_buf[7] = (uint8_t)((pend >> 24) & 0xFF);

  // Byte 8: status flags
  uint8_t flags = 0;
  if (system_ready)    flags |= FLAG_SYSTEM_READY;
  if (homing_complete) flags |= FLAG_HOMING_COMPLETE;
  spi_tx_buf[8] = flags;

  // Byte 9: XOR checksum over bytes 0-8
  uint8_t checksum = 0;
  for (int i = 0; i < 9; i++) checksum ^= spi_tx_buf[i];
  spi_tx_buf[9] = checksum;
}

// =============================================================================
// SPI slave transaction complete callback
// Called after each CS-framed transaction — reload buffer for next read
// =============================================================================
void IRAM_ATTR spi_post_trans_cb(spi_slave_transaction_t* trans) {
  build_spi_packet();
}

// =============================================================================
// Setup SPI slave
// =============================================================================
bool setup_spi_slave() {
  spi_bus_config_t bus_cfg = {
    .mosi_io_num   = SPI_MOSI,
    .miso_io_num   = SPI_MISO,
    .sclk_io_num   = SPI_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };

  spi_slave_interface_config_t slave_cfg = {
    .spics_io_num   = SPI_CS,
    .flags          = 0,
    .queue_size     = 2,
    .mode           = 0,              // SPI mode 0 — match Pi Simulink block setting
    .post_setup_cb  = NULL,
    .post_trans_cb  = spi_post_trans_cb,
  };

  if (spi_slave_initialize(VSPI_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO) != ESP_OK) {
    return false;
  }

  // Pre-load first packet
  build_spi_packet();

  // Queue first transaction
  static spi_slave_transaction_t trans;
  memset(&trans, 0, sizeof(trans));
  trans.length    = SPI_PACKET_BYTES * 8;  // length in bits
  trans.tx_buffer = spi_tx_buf;
  trans.rx_buffer = spi_rx_buf;
  spi_slave_queue_trans(VSPI_HOST, &trans, portMAX_DELAY);

  return true;
}

// =============================================================================
// Setup PCNT quadrature decoder
// =============================================================================
bool setup_pcnt(pcnt_unit_handle_t* unit, int pin_a, int pin_b) {
  // accum_count + watch points on both limits: the driver adds the limit to a
  // software total each time the hardware counter overflows and resets, so
  // pcnt_unit_get_count() keeps counting past +/-32767.
  pcnt_unit_config_t unit_config = {
    .low_limit  = -32768,
    .high_limit =  32767,
  };
  unit_config.flags.accum_count = 1;
  if (pcnt_new_unit(&unit_config, unit) != ESP_OK) return false;
  if (pcnt_unit_add_watch_point(*unit, unit_config.low_limit)  != ESP_OK) return false;
  if (pcnt_unit_add_watch_point(*unit, unit_config.high_limit) != ESP_OK) return false;

  pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 5000 };
  if (pcnt_unit_set_glitch_filter(*unit, &filter_config) != ESP_OK) return false;

  pcnt_chan_config_t chan_a_config = {
    .edge_gpio_num  = pin_a,
    .level_gpio_num = pin_b,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  if (pcnt_new_channel(*unit, &chan_a_config, &pcnt_chan_a) != ESP_OK) return false;
  pcnt_channel_set_edge_action(pcnt_chan_a,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcnt_chan_a,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_chan_config_t chan_b_config = {
    .edge_gpio_num  = pin_b,
    .level_gpio_num = pin_a,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  if (pcnt_new_channel(*unit, &chan_b_config, &pcnt_chan_b) != ESP_OK) return false;
  pcnt_channel_set_edge_action(pcnt_chan_b,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(pcnt_chan_b,
      PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);

  if (pcnt_unit_enable(*unit)      != ESP_OK) return false;
  if (pcnt_unit_clear_count(*unit) != ESP_OK) return false;
  if (pcnt_unit_start(*unit)       != ESP_OK) return false;
  return true;
}

// =============================================================================
// Check UART2 for homing_complete message from ESP32 #2
// Protocol: single byte 0xAA = homing complete
// =============================================================================
void check_uart_rx() {
  while (Serial2.available()) {
    uint8_t byte = Serial2.read();
    if (byte == 0xAA) {
      homing_complete = true;
      Serial.println("Homing complete confirmed from ESP32 #2");
      pcnt_unit_clear_count(motor_pcnt);
      pcnt_unit_clear_count(pend_pcnt);
    }
  }
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 #1 Encoder Firmware v9.2 ===");

  WiFi.mode(WIFI_OFF);

  // UART2 to ESP32 #2
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("UART2 ready");

  // Encoders
  if (!setup_pcnt(&motor_pcnt, MOTOR_ENC_A, MOTOR_ENC_B)) {
    Serial.println("ERROR: Motor PCNT failed"); while(1) delay(1000);
  }
  if (!setup_pcnt(&pend_pcnt, PEND_ENC_A, PEND_ENC_B)) {
    Serial.println("ERROR: Pendulum PCNT failed"); while(1) delay(1000);
  }
  Serial.println("Encoders ready");

  // SPI slave
  if (!setup_spi_slave()) {
    Serial.println("ERROR: SPI slave failed"); while(1) delay(1000);
  }
  Serial.println("SPI slave ready");

  // Mark system ready — no longer waiting for Simulink RESET pin
  // Pi Simulink block checks FLAG_SYSTEM_READY in status byte instead
  system_ready = true;

  Serial.println("=== Ready ===");
}

// =============================================================================
// Loop
// =============================================================================
void loop() {
  // Check for homing complete signal from ESP32 #2
  check_uart_rx();

  // Send motor and pendulum position to ESP32 #2 every 2ms.
  // Data bytes are 7-bit so they can never equal the 0xFF header byte.
  static unsigned long last_uart = 0;
  if (millis() - last_uart >= UART_INTERVAL_MS) {
    int motor_current = 0;
    int pend_current  = 0;
    pcnt_unit_get_count(motor_pcnt, &motor_current);
    pcnt_unit_get_count(pend_pcnt,  &pend_current);
    uint16_t pos  = (uint16_t)(int16_t)motor_current;
    uint16_t pend = (uint16_t)pend_current;   // low 16 bits; ESP32 #2 unwraps
    uint8_t pkt[7] = {
      0xFF,
      (uint8_t)( pos         & 0x7F),
      (uint8_t)((pos  >> 7)  & 0x7F),
      (uint8_t)((pos  >> 14) & 0x03),
      (uint8_t)( pend        & 0x7F),
      (uint8_t)((pend >> 7)  & 0x7F),
      (uint8_t)((pend >> 14) & 0x03),
    };
    Serial2.write(pkt, sizeof(pkt));
    last_uart = millis();
  }

#if ENC_DIAG
  // Count level changes on each raw encoder pin (independent of PCNT)
  static const int diag_pins[4] = { MOTOR_ENC_A, MOTOR_ENC_B, PEND_ENC_A, PEND_ENC_B };
  static int  diag_level[4] = { -1, -1, -1, -1 };
  static long diag_edges[4] = { 0, 0, 0, 0 };
  for (int i = 0; i < 4; i++) {
    int lvl = digitalRead(diag_pins[i]);
    if (diag_level[i] >= 0 && lvl != diag_level[i]) diag_edges[i]++;
    diag_level[i] = lvl;
  }
#endif

  // Debug print every second
  static unsigned long last_print = 0;
  if (millis() - last_print >= 1000) {
    int motor_current = 0;
    int pend_current  = 0;
    pcnt_unit_get_count(motor_pcnt, &motor_current);
    pcnt_unit_get_count(pend_pcnt,  &pend_current);
    Serial.print("M: ");    Serial.print(motor_current);
    Serial.print("  P: ");  Serial.print(pend_current);
    Serial.print("  Flags: 0x"); Serial.print(spi_tx_buf[8], HEX);
    Serial.print("  Checksum: 0x"); Serial.println(spi_tx_buf[9], HEX);
#if ENC_DIAG
    // Edges seen in the last second, and the current level, per channel
    Serial.printf("  edges/s  M_A:%ld M_B:%ld  P_A:%ld P_B:%ld   "
                  "levels  M_A:%d M_B:%d  P_A:%d P_B:%d\n",
                  diag_edges[0], diag_edges[1], diag_edges[2], diag_edges[3],
                  diag_level[0], diag_level[1], diag_level[2], diag_level[3]);
    for (int i = 0; i < 4; i++) diag_edges[i] = 0;
#endif
    last_print = millis();
  }
}
