////////////////////////////////////
//library
////////////////////////////////////
#include <Bluepad32.h>


/* pin serial*/
#define slaveSerial Serial1
#define GY25Serial Serial2

/* PIN ENCODER */
#define encDepan_CHA 36
#define encDepan_CHB 39
#define encKanan_CHA 32
#define encKanan_CHB 33
#define encKiri_CHA 34
#define encKiri_CHB 35



////////////////////////////////////
//variable kontroller
////////////////////////////////////
ControllerPtr myController = nullptr;
int stickLX = 128, stickLY = 128, stickRX = 128, stickRY = 128;
int stickLX_raw = 128, stickLY_raw = 128, stickRX_raw = 128, stickRY_raw = 128;
bool triangle = false;
bool cross = false;
bool circle = false;
bool square_ = false;
bool up = false;
bool down = false;
bool left = false;
bool right = false;
bool L1 = false;
bool R1 = false;
bool L2 = false;
bool R2 = false;
bool L3 = false;
bool R3 = false;
bool Start = false;
bool Select = false;
bool alreadyPressedL1 = false;
bool alreadyPressedR1 = false;
bool alreadyPressedL2 = false;
bool alreadyPressedR2 = false;
bool alreadyPressedL3 = false;
bool alreadyPressedR3 = false;
bool alreadyPressedUp = false;
bool alreadyPressedDown = false;
bool alreadyPressedLeft = false;
bool alreadyPressedRight = false;
bool alreadyPressedTriangle = false;
bool alreadyPressedCross = false;
bool alreadyPressedSquare = false;
bool alreadyPressedSelect = false;
bool alreadyPressedStart = false;


////////////////////////////////////
//ENCODER
////////////////////////////////////
struct ENC {
  volatile long int pulseCnt;
  long int lastPulse;
} EncDepan, EncKanan, EncKiri;

/* read encoder */
void IRAM_ATTR encDepan_A_Funct() {
  if (digitalRead(encDepan_CHA) == HIGH) {
    if (digitalRead(encDepan_CHB) == LOW) {
      EncDepan.pulseCnt++;
    } else {
      EncDepan.pulseCnt--;
    }
  } else {
    if (digitalRead(encDepan_CHB) == HIGH) {
      EncDepan.pulseCnt++;
    } else {
      EncDepan.pulseCnt--;
    }
  }
}
void IRAM_ATTR encDepan_B_Funct() {
  if (digitalRead(encDepan_CHA) == HIGH) {
    if (digitalRead(encDepan_CHB) == HIGH) {
      EncDepan.pulseCnt++;
    } else {
      EncDepan.pulseCnt--;
    }
  } else {
    if (digitalRead(encDepan_CHB) == LOW) {
      EncDepan.pulseCnt++;
    } else {
      EncDepan.pulseCnt--;
    }
  }
}

void IRAM_ATTR encKanan_A_Funct() {
  if (digitalRead(encKanan_CHA) == HIGH) {
    if (digitalRead(encKanan_CHB) == LOW) {
      EncKanan.pulseCnt++;
    } else {
      EncKanan.pulseCnt--;
    }
  } else {
    if (digitalRead(encKanan_CHB) == HIGH) {
      EncKanan.pulseCnt++;
    } else {
      EncKanan.pulseCnt--;
    }
  }
}
void IRAM_ATTR encKanan_B_Funct() {
  if (digitalRead(encKanan_CHA) == HIGH) {
    if (digitalRead(encKanan_CHB) == HIGH) {
      EncKanan.pulseCnt++;
    } else {
      EncKanan.pulseCnt--;
    }
  } else {
    if (digitalRead(encKanan_CHB) == LOW) {
      EncKanan.pulseCnt++;
    } else {
      EncKanan.pulseCnt--;
    }
  }
}

void IRAM_ATTR encKiri_A_Funct() {
  if (digitalRead(encKiri_CHA) == HIGH) {
    if (digitalRead(encKiri_CHB) == LOW) {
      EncKiri.pulseCnt++;
    } else {
      EncKiri.pulseCnt--;
    }
  } else {
    if (digitalRead(encKiri_CHB) == HIGH) {
      EncKiri.pulseCnt++;
    } else {
      EncKiri.pulseCnt--;
    }
  }
}
void IRAM_ATTR encKiri_B_Funct() {
  if (digitalRead(encKiri_CHA) == HIGH) {
    if (digitalRead(encKiri_CHB) == HIGH) {
      EncKiri.pulseCnt++;
    } else {
      EncKiri.pulseCnt--;
    }
  } else {
    if (digitalRead(encKiri_CHB) == LOW) {
      EncKiri.pulseCnt++;
    } else {
      EncKiri.pulseCnt--;
    }
  }
}

/* setup encoder */
void encoder_init() {
  pinMode(encDepan_CHA, INPUT);
  pinMode(encDepan_CHB, INPUT);
  pinMode(encKanan_CHA, INPUT);
  pinMode(encKanan_CHB, INPUT);
  pinMode(encKiri_CHA, INPUT);
  pinMode(encKiri_CHB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encDepan_CHA), encDepan_A_Funct, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encDepan_CHB), encDepan_B_Funct, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKanan_CHA), encKanan_A_Funct, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKanan_CHB), encKanan_B_Funct, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKiri_CHA), encKiri_A_Funct, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKiri_CHB), encKiri_B_Funct, CHANGE);
}



/////////////////////////////////////////////
//COMPAS GY 25
/////////////////////////////////////////////
struct cmps {
  char  buffer[50];
  int   counter;
  float heading;
  float continuousHeading;
  int rotationCount;
  float difference;
  float lastHeading;
} cmps;

/* Fungsi Compas */
void resetCMPS() {
  // kalibrasi Heading
  GY25Serial.write(0xA5);
  GY25Serial.write(0x55);
  delay(100);
  GY25Serial.write(0xA5);
  GY25Serial.write(0x53);
}

void cmps_init() {
  // Kalibrasi tilt
  GY25Serial.write(0xA5);
  GY25Serial.write(0x54);
  delay(1000);
  resetCMPS();
}

void updateCMPS() {
  char tmp;
  while (GY25Serial.available()) {
    tmp = GY25Serial.read();
    cmps.buffer[cmps.counter++] = tmp;
    if (tmp == '\n') {
      cmps.buffer[cmps.counter] = 0;
      cmps.heading = atof(strtok(cmps.buffer + 5, ","));
      cmps.counter = 0;
      cmps.difference = cmps.heading - cmps.lastHeading;
      if(cmps.difference> 180.0f) cmps.rotationCount--;
      else if(cmps.difference<180.0f) cmps.rotationCount++;
      cmps.lastHeading = cmps.heading;
      cmps.continuousHeading= cmps.rotationCount * 360.0 + cmps.heading;
    }
  }
}





float Vmax = 1.0;  // 1 m/s (ubah kalau perlu)

void run() {
  float
    s,   // Jarak
    v,   // Linear Velocity
    w,   // angular velocity
    Vx,  // Kecepatan arah X
    Vy;  // Kecepatan arah Y

  float
    alpha,  // heading Error Robot
    TB,     // Target Bearing
    dx,     // Delta x
    dy,     // Delta y
    w1,     // Motor Depan
    w2,     // Motor Kanan
    w3,     // Motor Kiri
    v1,     // Motor Belakang
    v2,     // Motor Kanan
    v3,     // Motor Kiri
    RPM1,   // Motor Belakang
    RPM2,   // Motor Kanan
    RPM3,   // Motor Kiri
    dw;     // Delta Omega

  if (R1 && !alreadyPressedR1) {
    Vmax += 1.0;
    if (Vmax > 7.0) Vmax = 7.0;
    alreadyPressedR1 = true;
  } else if (!R1) {
    alreadyPressedR1 = false;
  }


  if (L1 && !alreadyPressedL1) {
    Vmax -= 1.0;
    if (Vmax < 1.0) Vmax = 1.0;
    alreadyPressedL1 = true;
  } else if (!L1) {
    alreadyPressedL1 = false;
  }

  // --- BACA ANALOG PS2 ---
  int rawLX = stickLX;  // 0..255
  int rawLY = stickLY;  // 0..255

  // --- NORMALISASI ---
  float normLX = rawLX - 128;  // -128 .. +127
  float normLY = rawLY - 128;  // -128 .. +127

  // --- KONVERSI KE Vx, Vy DALAM m/s ---
  Vx = (normLX / 128.0) * Vmax;
  Vy = (normLY / 128.0) * Vmax;

  float normRX = stickRX - 128;
  w = (normRX / 128.0) * 3.0;  // 3 rad/s max

  w1 = (-0.67 * Vx) + (0 * Vy) + (0.33 * w);
  w2 = (0.33 * Vx) + (0.58 * Vy) + (0.33 * w);
  w3 = (0.33 * Vx) + (-0.58 * Vy) + (0.33 * w);

  v1 = w1;
  v2 = w2;
  v3 = w3;

  float D = 0.12;

  // --- KONVERSI KE RPM ---
  RPM1 = (60 * v1) / (M_PI * D);
  RPM2 = (60 * v2) / (M_PI * D);
  RPM3 = (60 * v3) / (M_PI * D);

  // --- OUTPUT KE MOTOR ---
  sendToSlave(RPM3, RPM2, RPM1);

  // Serial.println(Vmax);
  // Serial.printf("kiri=%f.2 kanan=%f.2 belakang=%f.2\n", RPM3, RPM2, RPM1);
}


#define LED 2

void TaskBluetoothSend(void *pvParameters) {
  (void)pvParameters;
  const TickType_t delayTime = pdMS_TO_TICKS(1);

  while (true) {
    BP32.update();
    processControllers();
    digitalWrite(LED, up);
    // sendToSlave();
    vTaskDelay(delayTime);  // Delay antar kirim
  }
}

void TaskProcess(void *pvParameters) {
  (void)pvParameters;
  const TickType_t delayTime = pdMS_TO_TICKS(1);

  while (true) {
    run();
    updateCMPS();
    // Serial.print(cmps.heading);
    // Serial.printf(" %d, %d, %d\n", EncKanan.pulseCnt,  EncKiri.pulseCnt,EncDepan.pulseCnt);
    vTaskDelay(delayTime);  // Delay antar kirim
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  slaveSerial.begin(115200, SERIAL_8N1, 25, 26);  // Serial ke slave
  GY25Serial.begin(115200, SERIAL_8N1, 27, 14);
  BP32.setup(&onConnectedController, &onDisconnectedController);

  encoder_init();
  cmps_init();

  xTaskCreatePinnedToCore(
    TaskBluetoothSend,
    "BluetoothTask",
    4096,
    NULL,
    1,
    NULL,
    0  // Core 0
  );
  xTaskCreatePinnedToCore(
    TaskProcess,
    "ProcessTask",
    4096,
    NULL,
    1,
    NULL,
    1  // Core 0
  );
}

void loop() {
}
