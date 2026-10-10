// =============================================================================
// UART link to the Raspberry Pi (Simulink mode), UART2: RX 16, TX 17, 921600.
// Byte handling only; exp_simulink.ino decides what to do with a command.
//   Pi -> ESP32 command, 6 bytes:      A5 5A  u (float32 LE, volts, + = right)
//     u = 0.5   -> A5 5A 00 00 00 3F
//     u = -1.25 -> A5 5A 00 00 A0 BF
//   ESP32 -> Pi measurement, 12 bytes: A5 5A  cart (int32 LE)  pend (int32 LE)
//                                      flags  XOR of bytes 2-10
//     cart 1000, pend -5, flags 0x07       -> A5 5A E8 03 00 00 FB FF FF FF 07 E8
//     cart -123456, pend 2, flags 0x02     -> A5 5A C0 1D FE FF 02 00 00 00 02 DC
// Same examples as picontrol/tests/test_pilink.py. Bytes 2-11 are the old SPI
// packet, so the Simulink models' unpack_spi reads them unchanged.
// Counts in the old rig's conventions: cart 0 at centre, + to the right;
// pendulum 0 hanging (measured hanging reference).
// =============================================================================
uint8_t  plBuf[4];
int      plState    = 0;    // 0: want A5, 1: want 5A, 2..5: next data byte index + 2
uint32_t plCommands = 0;    // valid commands this run
uint32_t plDrops    = 0;    // complete packets with an impossible u (NaN, inf, > 5 V)

// Discard anything waiting (bytes from an earlier run or sent during homing)
void piLinkReset() {
  while (Serial2.available()) Serial2.read();
  plState    = 0;
  plCommands = 0;
  plDrops    = 0;
}

uint32_t piLinkCommands() { return plCommands; }
uint32_t piLinkDrops()    { return plDrops; }

// Non-blocking. Returns true when one valid command is complete (*u = volts);
// bytes after it stay in the buffer for the next call.
bool piLinkRead(float* u) {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (plState == 0) { if (b == 0xA5) plState = 1; continue; }
    if (plState == 1) { plState = (b == 0x5A) ? 2 : (b == 0xA5 ? 1 : 0); continue; }
    plBuf[plState - 2] = b;
    if (++plState < 6) continue;
    plState = 0;
    float v;
    memcpy(&v, plBuf, 4);                 // the ESP32 is little-endian, like the packet
    if (!isfinite(v) || fabsf(v) > PI_U_MAX_V) { plDrops++; continue; }
    plCommands++;
    *u = v;
    return true;
  }
  return false;
}

void piLinkSend(uint8_t flags) {
  int32_t cart = cartDir * cart_position;          // 0 at centre, + to the right
  int32_t pend = pend_position - pendDownRef;      // 0 hanging
  uint8_t p[12] = { 0xA5, 0x5A };
  memcpy(p + 2, &cart, 4);
  memcpy(p + 6, &pend, 4);
  p[10] = flags;
  uint8_t x = 0;
  for (int i = 2; i <= 10; i++) x ^= p[i];
  p[11] = x;
  Serial2.write(p, sizeof(p));
}
