// =============================================================================
// ESP32 #1 — Encoder Firmware
// Version: 9.0
// Description: Reads motor and pendulum quadrature encoders via PCNT hardware.
//              Serves encoder counts to Raspberry Pi over SPI slave interface.
//              Sends motor position to ESP32 #2 over UART2 for homing routine.
//              Replaces MCP23017/PCI-1711 data path entirely.
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

// --- UART timing ---
#define UART_INTERVAL_MS  4

// --- SPI packet size ---
#define SPI_PACKET_BYTES  10

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
  pcnt_unit_config_t unit_config = {
    .low_limit  = -32768,
    .high_limit =  32767,
  };
  if (pcnt_new_unit(&unit_config, unit) != ESP_OK) return false;

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
      pcnt_unit_clear_count{motor_pcnt};
      pcnt_unit_clear_count{pend_pcnt};
    }
  }
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 #1 Encoder Firmware v9.0 ===");

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

  // Send motor position to ESP32 #2 every 4ms for homing
  static unsigned long last_uart = 0;
  if (millis() - last_uart >= UART_INTERVAL_MS) {
    int motor_current = 0;
    pcnt_unit_get_count(motor_pcnt, &motor_current);
    int16_t pos = (int16_t)motor_current;
    Serial2.write(0xFF);
    Serial2.write((uint8_t)(pos & 0xFF));
    Serial2.write((uint8_t)((pos >> 8) & 0xFF));
    last_uart = millis();
  }

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
    last_print = millis();
  }
}
