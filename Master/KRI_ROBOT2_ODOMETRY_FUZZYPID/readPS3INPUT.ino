bool isAllowedController(ControllerPtr ctl) {
  ControllerProperties p = ctl->getProperties();
  uint8_t allowedMac[] = { 0x03, 0x20, 0x08, 0x23, 0x5a, 0x2e };
  return memcmp(p.btaddr, allowedMac, 6) == 0;
}
void onConnectedController(ControllerPtr ctl) {
  // if (!isAllowedController(ctl)) return;
  if (!isAllowedController(ctl)) {
    ctl->disconnect();
    return;
  }

  if (myController == nullptr) {
    myController = ctl;
    ControllerProperties properties = ctl->getProperties();
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (ctl == myController) myController = nullptr;
}


void processGamepad(ControllerPtr ctl) {
  const uint8_t dpad = ctl->dpad();
  const uint16_t btn = ctl->buttons();
  const uint8_t miscc = ctl->miscButtons();
  const int16_t axX = ctl->axisX();
  const int16_t axY = ctl->axisY();
  const int16_t axRX = ctl->axisRX();
  const int16_t axRY = ctl->axisRY();
  const uint16_t brake = ctl->brake();
  const uint16_t throt = ctl->throttle();

  // ---- DPad mapping (lookup table → tercepat) ----
  static const bool upLUT[16] = { 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 };
  static const bool downLUT[16] = { 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0 };
  static const bool rightLUT[16] = { 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  static const bool leftLUT[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0 };

  up = upLUT[dpad];
  down = downLUT[dpad];
  right = rightLUT[dpad];
  left = leftLUT[dpad];

  Start = (miscc == 0x04) || (miscc == 0x06);
  Select = (miscc == 0x02) || (miscc == 0x06);

  cross = btn & 0x0001;
  circle = btn & 0x0002;
  square_ = btn & 0x0004;
  triangle = btn & 0x0008;

  L1 = btn & 0x0010;
  R1 = btn & 0x0020;
  L2 = brake > 50;
  R2 = throt > 50;
  L3 = btn & 0x0100;
  R3 = btn & 0x0200;

  stickLX_raw = axX;
  stickLY_raw = axY;
  stickRX_raw = axRX;
  stickRY_raw = axRY;

  stickLX = (axX + 512) >> 2;
  stickLY = (axY + 512) >> 2;
  stickRX = (axRX + 512) >> 2;
  stickRY = (axRY + 512) >> 2;
}


void processControllers() {
  if (myController && myController->isConnected()) {
    processGamepad(myController);
  } else {
    up = down = left = right =
      triangle = cross = circle = square_ =
        L1 = R1 = L2 = R2 = L3 = R3 = Start = Select = 0;
    stickLX = stickLY = stickRX = stickRY = 128;
  }
}
