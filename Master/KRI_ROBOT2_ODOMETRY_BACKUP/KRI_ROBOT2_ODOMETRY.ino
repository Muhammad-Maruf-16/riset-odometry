////////////////////////////////////
//library
////////////////////////////////////
// #include <Bluepad32.h>


/* pin serial*/
#define slaveSerial Serial1
#define GY25Serial Serial2

/* PIN ENCODER */
#define encDepan_CHA 39
#define encDepan_CHB 36
#define encKanan_CHA 32
#define encKanan_CHB 33
#define encKiri_CHA 34
#define encKiri_CHB 35

/* ODOMETRY */
#define MATH_PI 3.14286
#define MATH_ROOT_OF_3 1.73205
#define WHEEL_RADIUS 2.9
#define WHEEL_BASE 9.40
#define ENC_RESOLUTION 2400.0
#define MIN_POS_RES 3.0
#define MIN_DEG_RES 5.0

/* Expansi Macro */
#define POS_CONV_FACTOR 0.00261905  // ((2 * MATH_PI) / ENC_RESOLUTION)
#define MATH_SIN(x) sin(radians(x))
#define MATH_COS(x) cos(radians(x))



////////////////////////////////////
//variable kontroller
////////////////////////////////////
// ControllerPtr myController = nullptr;
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
  char buffer[50];
  int counter;
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
/* setup gy25 */
void cmps_init() {
  // Kalibrasi tilt
  GY25Serial.write(0xA5);
  GY25Serial.write(0x54);
  delay(1000);
  resetCMPS();
}
/* update gy25 */
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
      if (cmps.difference > 180.0f) cmps.rotationCount--;
      else if (cmps.difference < 180.0f) cmps.rotationCount++;
      cmps.lastHeading = cmps.heading;
      cmps.continuousHeading = cmps.rotationCount * 360.0 + cmps.heading;
    }
  }
}


/////////////////////////////////////////////
//ODOMETRY
/////////////////////////////////////////////
struct POS {
  float X, Y, T;
} currentPos, lastPos;
int count = 0;
bool Move = false;

void odometry_init(void) {
  currentPos.X = lastPos.X = currentPos.Y = lastPos.Y = currentPos.T = 0;
  EncKanan.pulseCnt = EncDepan.pulseCnt = EncKiri.pulseCnt = 0;
  EncKanan.lastPulse = EncDepan.lastPulse = EncKiri.lastPulse = 0;
}
void fullResetENC(void) {
  odometry_init();
}
void updateOdometry(void) {
  updateCMPS();
  long lTime;
  float _x, _y, _t;
  // kecepatan rotasi posisi dan heading
  float rvPos1, rvPos2, rvPos3;

  long int
    // delta pulse
    dKiriENC,
    dKananENC,
    dBelakangENC,
    // pulsa sekarang
    currentBePC,
    currentKaPC,
    currentKiPC;

  currentBePC = EncDepan.pulseCnt;
  currentKaPC = EncKanan.pulseCnt;
  currentKiPC = EncKiri.pulseCnt;

  dBelakangENC = currentBePC - EncDepan.lastPulse;
  dKananENC = currentKaPC - EncKanan.lastPulse;
  dKiriENC = currentKiPC - EncKiri.lastPulse;

  /* POS ENC X AND Y */
  rvPos1 = dBelakangENC * POS_CONV_FACTOR;
  rvPos2 = dKananENC * POS_CONV_FACTOR;
  rvPos3 = dKiriENC * POS_CONV_FACTOR;

  _x = WHEEL_RADIUS * ((2 / 3.0 * rvPos1) - (1 / 3.0 * rvPos2) - (1 / 3.0 * rvPos3));
  _y = WHEEL_RADIUS * ((1 / MATH_ROOT_OF_3 * rvPos2) - (1 / MATH_ROOT_OF_3 * rvPos3));
  _t = cmps.heading;

  currentPos.X = lastPos.X - ((MATH_COS(_t) * _x) - (MATH_SIN(_t) * _y));
  currentPos.Y = lastPos.Y + ((MATH_SIN(_t) * _x) + (MATH_COS(_t) * _y));
  currentPos.T = _t;


  EncDepan.lastPulse = currentBePC;
  EncKanan.lastPulse = currentKaPC;
  EncKiri.lastPulse = currentKiPC;

  lastPos.X = currentPos.X;
  lastPos.Y = currentPos.Y;
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

bool goXYT(int x, int y, int t) {
  updateOdometry();
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
    w1,     // Motor Belakang
    w2,     // Motor Kanan
    w3,     // Motor Kiri
    v1,     // Motor Belakang
    v2,     // Motor Kanan
    v3,     // Motor Kiri
    RPM1,   // Motor Belakang
    RPM2,   // Motor Kanan
    RPM3,   // Motor Kiri
    dw;     // Delta Omega

  v1 = v2 = v3 = 0;
  w1 = w2 = w3 = 0;
  v = 0;
  w = 0;

  // Delta Position
  dx = abs(currentPos.X - x);
  dy = abs(currentPos.Y - y);

  // Target Bearing
  TB = degrees(atan2((y - currentPos.Y), (x - currentPos.X)));

  // heading Error Robot
  alpha = TB - currentPos.T;

  s = sqrt(pow(dx, 2) + pow(dy, 2));
  dw = t - currentPos.T;

  if (s <= MIN_POS_RES) {
    v = 0;
    if (s <= MIN_POS_RES && abs(dw) <= MIN_DEG_RES && Move == true) {
      count++;
      if (count == 2) {
        sendToSlave(0, 0, 0);
      }
    }
    if (s <= MIN_POS_RES && abs(dw) <= MIN_DEG_RES && Move == false) {
      count = 0;
      sendToSlave(0, 0, 0);
    }
  }

  if (s > MIN_POS_RES) {
    v = FuzzyLinear(s);
  }

  if (abs(dw) > MIN_DEG_RES) {
    w = FuzzyOmega(abs(dw));
    if (dw < 0) {
      w *= -1;
    }
  }

  // Hitung kecepatan arah sumbu X dan Y (Vektor)
  Vx = v * cos(radians(alpha));
  Vy = v * sin(radians(alpha));

  // angular Motor [rad/s]
  w1 = (0.67 * Vx) + (0 * Vy) + (0.33 * w);
  w2 = (-0.33 * Vx) + (0.58 * Vy) + (0.33 * w);
  w3 = (-0.33 * Vx) + (-0.58 * Vy) + (0.33 * w);

  // Linear Motor [m/s];
  v1 = float(0.058 * w1);  //0.029
  v2 = float(0.058 * w2);  //0.029
  v3 = float(0.058 * w3);  //0.029

  // convert To RPM
  RPM1 = (60 * v1) / (2 * MATH_PI * 0.075);  // 0.075 is radius motor
  RPM2 = (60 * v2) / (2 * MATH_PI * 0.075);
  RPM3 = (60 * v3) / (2 * MATH_PI * 0.075);

  sendToSlave(-RPM3, -RPM2, -RPM1);


  Serial.printf("x %.4f, y %.4f, t %.4f\n", RPM3, RPM2, RPM1);

  return false;
}





#define AVLOW_SPEED 10
#define ALOW_SPEED 20
#define AMEDIUM_SPEED 30
#define AFAST_SPEED 50
#define AVFAST_SPEED 75

// Linear
#define LVLOW_SPEED 20
#define LLOW_SPEED 40
#define LMEDIUM_SPEED 50
#define LFAST_SPEED 60
#define LVFAST_SPEED 190

float FuzzyOmega(float wSpeed) {
  float Omega[6] = { 0, 10, 75, 130, 180, 230 };
  float Speed[5];
  float output = 0;
  /* SPEED */
  // Very Near
  if (wSpeed > Omega[0] && wSpeed <= Omega[1]) {
    // Speed[0] = (Omega[1] - wSpeed) / (Omega[1] - Omega[0]);
    Speed[0] = 1;
  } else if (wSpeed > Omega[1] && wSpeed <= Omega[2]) {
    Speed[0] = (Omega[2] - wSpeed) / (Omega[2] - Omega[1]);
  } else {
    Speed[0] = 0;
  }
  // Near
  if (wSpeed <= Omega[1]) {
    Speed[1] = 0;
  } else if (wSpeed > Omega[1] && wSpeed <= Omega[2]) {
    Speed[1] = (wSpeed - Omega[1]) / (Omega[2] - Omega[1]);
  } else if (wSpeed > Omega[2] && wSpeed <= Omega[3]) {
    Speed[1] = (Omega[3] - wSpeed) / (Omega[3] - Omega[2]);
  } else {
    Speed[1] = 0;
  }
  // Midle
  if (wSpeed <= Omega[2]) {
    Speed[2] = 0;
  } else if (wSpeed > Omega[2] && wSpeed <= Omega[3]) {
    Speed[2] = (wSpeed - Omega[2]) / (Omega[3] - Omega[2]);
  } else if (wSpeed > Omega[3] && wSpeed <= Omega[4]) {
    Speed[2] = (Omega[4] - wSpeed) / (Omega[4] - Omega[3]);
  } else {
    Speed[2] = 0;
  }
  // Far
  if (wSpeed <= Omega[3]) {
    Speed[3] = 0;
  } else if (wSpeed > Omega[3] && wSpeed <= Omega[4]) {
    Speed[3] = (wSpeed - Omega[3]) / (Omega[4] - Omega[3]);
  } else if (wSpeed > Omega[4] && wSpeed <= Omega[5]) {
    Speed[3] = (Omega[5] - wSpeed) / (Omega[5] - Omega[4]);
  } else {
    Speed[3] = 0;
  }
  // Very Far
  if (wSpeed <= Omega[4]) {
    Speed[4] = 0;
  } else if (wSpeed > Omega[4] && wSpeed <= Omega[5]) {
    Speed[4] = (wSpeed - Omega[4]) / (Omega[5] - Omega[4]);
  } else {
    Speed[4] = 1;
  }
  output = ((Speed[0] * AVLOW_SPEED) + (Speed[1] * ALOW_SPEED) + (Speed[2] * AMEDIUM_SPEED) + (Speed[3] * AFAST_SPEED) + (Speed[4] * AVFAST_SPEED)) / (Speed[0] + Speed[1] + Speed[2] + Speed[3] + Speed[4]);
  return output;
}

float FuzzyLinear(float vSpeed) {
  float output = 0;
  float fSpeed[6] = { 0, 35, 80, 130, 180, 230 };
  float Speed[5];

  /* SPEED */
  // Very Near
  if (vSpeed > fSpeed[0] && vSpeed <= fSpeed[1]) {
    // Speed[0] = (fSpeed[1] - vSpeed) / (fSpeed[1] - fSpeed[0]);
    Speed[0] = 1;
  } else if (vSpeed > fSpeed[1] && vSpeed <= fSpeed[2]) {
    Speed[0] = (fSpeed[2] - vSpeed) / (fSpeed[2] - fSpeed[1]);
  } else {
    Speed[0] = 0;
  }
  // Near
  if (vSpeed <= fSpeed[1]) {
    Speed[1] = 0;
  } else if (vSpeed > fSpeed[1] && vSpeed <= fSpeed[2]) {
    Speed[1] = (vSpeed - fSpeed[1]) / (fSpeed[2] - fSpeed[1]);
  } else if (vSpeed > fSpeed[2] && vSpeed <= fSpeed[3]) {
    Speed[1] = (fSpeed[3] - vSpeed) / (fSpeed[3] - fSpeed[2]);
  } else {
    Speed[1] = 0;
  }
  // Midle
  if (vSpeed <= fSpeed[2]) {
    Speed[2] = 0;
  } else if (vSpeed > fSpeed[2] && vSpeed <= fSpeed[3]) {
    Speed[2] = (vSpeed - fSpeed[2]) / (fSpeed[3] - fSpeed[2]);
  } else if (vSpeed > fSpeed[3] && vSpeed <= fSpeed[4]) {
    Speed[2] = (fSpeed[4] - vSpeed) / (fSpeed[4] - fSpeed[3]);
  } else {
    Speed[2] = 0;
  }
  // Far
  if (vSpeed <= fSpeed[3]) {
    Speed[3] = 0;
  } else if (vSpeed > fSpeed[3] && vSpeed <= fSpeed[4]) {
    Speed[3] = (vSpeed - fSpeed[3]) / (fSpeed[4] - fSpeed[3]);
  } else if (vSpeed > fSpeed[4] && vSpeed <= fSpeed[5]) {
    Speed[3] = (fSpeed[5] - vSpeed) / (fSpeed[5] - fSpeed[4]);
  } else {
    Speed[3] = 0;
  }
  // Very Far
  if (vSpeed <= fSpeed[4]) {
    Speed[4] = 0;
  } else if (vSpeed > fSpeed[4] && vSpeed <= fSpeed[5]) {
    Speed[4] = (vSpeed - fSpeed[4]) / (fSpeed[5] - fSpeed[4]);
  } else {
    Speed[4] = 1;
  }
  output = ((Speed[0] * LVLOW_SPEED) + (Speed[1] * LLOW_SPEED) + (Speed[2] * LMEDIUM_SPEED) + (Speed[3] * LFAST_SPEED) + (Speed[4] * LVFAST_SPEED)) / (Speed[0] + Speed[1] + Speed[2] + Speed[3] + Speed[4]);
  return output;
}

char IR64;
bool MoveGoal = false;
bool writeODM = false;
struct data {
  int16_t
    p1,
    p2,
    p3;
} targetXYT;

// Move goal To goal
int ZoneArr[100][3]{
  { -150, 150, 0 },
  { -50, 0, 0 },
  { 0, 0, 0 },
};

void controlCompute() {
  updateOdometry();
  if (writeODM) {
    if (goXYT(targetXYT.p1, targetXYT.p2, targetXYT.p3)) {
      writeODM = false;
    }
  }
  if (MoveGoal) {
    switch (count) {
      case 0:
        goXYT(ZoneArr[0][0], ZoneArr[0][1], ZoneArr[0][2]);
        break;
      case 1:
        goXYT(ZoneArr[1][0], ZoneArr[1][1], ZoneArr[1][2]);
        break;
      case 2:
        goXYT(ZoneArr[2][0], ZoneArr[2][1], ZoneArr[2][2]);
        break;
      default:
        sendToSlave(0, 0, 0);
        count = 2;  // Last Count
        break;
    }
  }

  if (slaveSerial.available()) {
    IR64 = slaveSerial.read();
    Serial.println(IR64);
    if (IR64 == 'O') {  // START
      writeODM = false;
      MoveGoal = true;
      Move = true;
      count = 0;
    }
    if (IR64 == 'W') {  // L3
      writeODM = true;
      MoveGoal = false;
      Move = false;
      targetXYT.p1 = 0;
      targetXYT.p2 = 200;
      targetXYT.p3 = 0;
    }
    if (IR64 == 'S') {  // R3
      writeODM = true;
      MoveGoal = false;
      Move = false;
      targetXYT.p1 = 0;
      targetXYT.p2 = -100;
      targetXYT.p3 = 0;
    }
    if (IR64 == 'A') {  // Triangle
      writeODM = true;
      MoveGoal = false;
      Move = false;
      targetXYT.p1 = -200;
      targetXYT.p2 = 0;
      targetXYT.p3 = 0;
    }
    if (IR64 == 'X') {  // Cross
      writeODM = true;
      MoveGoal = false;
      Move = false;
      targetXYT.p1 = 0;
      targetXYT.p2 = 0;
      targetXYT.p3 = 0;
    }
  }
}

#define LED 2

void TaskBluetoothSend(void *pvParameters) {
  (void)pvParameters;
  const TickType_t delayTime = pdMS_TO_TICKS(1);

  while (true) {
    // BP32.update();
    // processControllers();
    // digitalWrite(LED, up);
    // sendToSlave();
    // if (slaveSerial.available()) {
    //   String s = slaveSerial.readStringUntil('\n');
    //   Serial.println(s);
    // }
    vTaskDelay(delayTime);  // Delay antar kirim
  }
}

void TaskProcess(void *pvParameters) {
  (void)pvParameters;
  const TickType_t delayTime = pdMS_TO_TICKS(1);
  while (true) {
    // run();
    // updateOdometry();
    // if (square_) {
    //   fullResetENC();
    // }
    controlCompute();
    // Serial.printf("x %.2f, y %.2f, t %.2f\n", currentPos.X, currentPos.Y, currentPos.T);
    // Serial.printf(" %d, %d, %d\n", EncKanan.pulseCnt,  EncKiri.pulseCnt,EncDepan.pulseCnt);
    vTaskDelay(delayTime);  // Delay antar kirim
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  slaveSerial.begin(115200, SERIAL_8N1, 13, 12);  // Serial ke slave
  GY25Serial.begin(115200, SERIAL_8N1, 27, 14);
  // BP32.setup(&onConnectedController, &onDisconnectedController);

  encoder_init();
  cmps_init();
  odometry_init();

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
