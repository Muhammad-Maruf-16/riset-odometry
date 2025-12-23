bool soft = 0;
unsigned long lastSend = 0;
void sendToSlave(float Left, float Right, float Back) {
  int BRRPM = 0,
      motor_belakang = Back,
      motor_kanan = Right,
      motor_kiri = Left,
      last_BRRPM = 0,
      last_motor_belakang = 0,
      last_motor_kanan = 0,
      last_motor_kiri = 0;


  if (millis() - lastSend > 500) {
    lastSend = millis();
    last_BRRPM = last_motor_belakang = last_motor_kiri = last_motor_kanan = 1;
  }

  if (BRRPM != last_BRRPM || motor_belakang != last_motor_belakang || motor_kiri != last_motor_kiri || motor_kanan != last_motor_kanan) {
    sendToSlaveUART(soft, motor_kiri, soft, motor_kanan, soft, motor_belakang, soft, BRRPM);
    last_BRRPM = BRRPM;
    last_motor_belakang = motor_belakang;
    last_motor_kiri = motor_kiri;
    last_motor_kanan = motor_kanan;
  }
}



// ========== FUNGSI KIRIM KE 4 SLAVE ==========
void sendToSlaveUART(bool cmd1, int16_t rpm1,
                     bool cmd2, int16_t rpm2,
                     bool cmd3, int16_t rpm3,
                     bool cmd4, int16_t rpm4) {

  // Protocol: 0xAA | D1H | D1M | D1L | D2H | D2M | D2L | D3H | D3M | D3L | D4H | D4M | D4L | CHK | 0x55
  uint8_t packet[15];

  packet[0] = 0xAA;  // Header

  // Slave 1 data
  packet[1] = cmd1 ? 0x01 : 0x00;  // D1H = CMD
  packet[2] = (rpm1 >> 8) & 0xFF;  // D1M = RPM high byte
  packet[3] = rpm1 & 0xFF;         // D1L = RPM low byte

  // Slave 2 data
  packet[4] = cmd2 ? 0x01 : 0x00;  // D2H = CMD
  packet[5] = (rpm2 >> 8) & 0xFF;  // D2M = RPM high byte
  packet[6] = rpm2 & 0xFF;         // D2L = RPM low byte

  // Slave 3 data
  packet[7] = cmd3 ? 0x01 : 0x00;  // D3H = CMD
  packet[8] = (rpm3 >> 8) & 0xFF;  // D3M = RPM high byte
  packet[9] = rpm3 & 0xFF;         // D3L = RPM low byte

  // Slave 4 data
  packet[10] = cmd4 ? 0x01 : 0x00;  // D4H = CMD
  packet[11] = (rpm4 >> 8) & 0xFF;  // D4M = RPM high byte
  packet[12] = rpm4 & 0xFF;         // D4L = RPM low byte

  // Calculate XOR checksum
  uint8_t checksum = 0;
  for (int i = 1; i <= 12; i++) {
    checksum ^= packet[i];
  }
  packet[13] = checksum;

  packet[14] = 0x55;  // Footer

  // Kirim ke semua slave (broadcast via individual TX pins)
  // Semua slave akan terima packet yang sama, tapi parse sesuai ID masing-masing
  slaveSerial.write(packet, 15);  // Slave 1

  // Debug (optional)
  // Serial.printf("[TX] RPM1=%d RPM2=%d RPM3=%d RPM4=%d | CHK=0x%02X\n",
  //               rpm1, rpm2, rpm3, rpm4, checksum);
}
