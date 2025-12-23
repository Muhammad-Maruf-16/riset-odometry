#define slaveSerial Serial3
#define cmpSerial Serial2
#define nanoSerial Serial1

/*COMMAND*/
#define AT_RESET "RESET"
#define AT_RPS "RPS"

/* ENCODER */
#define encBelakang_CHA 27
#define encBelakang_CHB 29
#define encKanan_CHA 31
#define encKanan_CHB 33
#define encKiri_CHA 35
#define encKiri_CHB 37

/* ODOMETRY */
#define MATH_PI 3.14286
#define MATH_ROOT_OF_3 1.73205
#define WHEEL_RADIUS 2.9
#define WHEEL_BASE 9.40
#define ENC_RESOLUTION 2400.0
#define MIN_POS_RES 1.0
#define MIN_DEG_RES 3.0

/* Expansi Macro */
#define POS_CONV_FACTOR ((2 * MATH_PI) / ENC_RESOLUTION)
#define MATH_SIN(x) sin(radians(x))
#define MATH_COS(x) cos(radians(x))

/* Fuzzy */
// omega
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

float headingContinuous = 0;
float lastHeading = 0;


/* MOVE GOAL TO GOAL */
int count = 0;
bool Move = false;
/* MOVE GOAL TO GOAL */

/* KOORDINAT X, Y, DAN HEADING */
struct POS {
  float X, Y, T;
} currentPOS, lastPOS;

/* PENYIMPANAN PULSA ENCODER */
struct ENC {
  volatile long int pulseCnt;
  long int lastpulse;
} EncBelakang, EncKanan, EncKiri;

/*COMPAS GY 25*/
struct cmps {
  char buffer[50];
  int counter;
  float heading;
} cmps;

/* Fungsi Compas */
void resetCMPS() {
  // kalibrasi Heading
  cmpSerial.write(0xA5);
  cmpSerial.write(0x55);
  delay(100);
  cmpSerial.write(0xA5);
  cmpSerial.write(0x53);
}

void initCMPS() {
  // Kalibrasi tilt
  cmpSerial.write(0xA5);
  cmpSerial.write(0x54);
  delay(1000);
  resetCMPS();
}

// void updateCMPS() {
//   char tmp;
//   while (cmpSerial.available()) {
//     tmp = cmpSerial.read();
//     cmps.buffer[cmps.counter++] = tmp;
//     if (tmp == '\n') {
//       cmps.buffer[cmps.counter] = 0;
//       cmps.heading = atof(strtok(cmps.buffer + 5, ","));
//       cmps.counter = 0;
//     }
//   }
// }

void updateCMPS() {
  char tmp;
  while (cmpSerial.available()) {
    tmp = cmpSerial.read();
    cmps.buffer[cmps.counter++] = tmp;

    if (tmp == '\n') {
      cmps.buffer[cmps.counter] = 0;

      // Parse heading normal 0–360
      cmps.heading = atof(strtok(cmps.buffer + 5, ","));

      // --- Tambahan continuous heading ---
      float raw = cmps.heading;
      float diff = raw - lastHeading;

      // Koreksi wrap-around (misal dari 359 → 0 atau 0 → 359)
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;

      headingContinuous -= diff;


      while (headingContinuous > 180) headingContinuous -= 360;
      while (headingContinuous <= -180) headingContinuous += 360;


      lastHeading = raw;
      // ------------------------------------

      cmps.counter = 0;
    }
  }
}


void setup() {
  Serial.begin(115200);
  cmpSerial.begin(115200);
  slaveSerial.begin(115200);
  // fullResetENC();
  nanoSerial.begin(115200);

  pinMode(encBelakang_CHA, INPUT_PULLUP);
  pinMode(encBelakang_CHB, INPUT_PULLUP);
  pinMode(encKanan_CHA, INPUT_PULLUP);
  pinMode(encKanan_CHB, INPUT_PULLUP);
  pinMode(encKiri_CHA, INPUT_PULLUP);
  pinMode(encKiri_CHB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encBelakang_CHA), encBelakang_INTT_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBelakang_CHB), encBelakang_INTT_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKanan_CHA), encKanan_INTT_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKanan_CHB), encKanan_INTT_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKiri_CHA), encKiri_INTT_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKiri_CHB), encKiri_INTT_B, CHANGE);
  delay(1000);

  /* RESET */
  fullResetENC();
  initCMPS();
  initOdometry();

  /* Fungsi Utama */
  controlCompute();

  /* Fungsi Trial */
  // trialEncMaster();
}

void loop() {
  // Dead Loop();
}

void trialEncMaster() {
  while (1) {
    updateOdometry();
    Serial.print("BE: ");
    Serial.print(EncBelakang.pulseCnt);
    Serial.print("\tKA: ");
    Serial.print(EncKanan.pulseCnt);
    Serial.print("\tKI: ");
    Serial.print(EncKiri.pulseCnt);
    Serial.print("\tX: ");
    Serial.print(currentPOS.X);
    Serial.print("\tY: ");
    Serial.print(currentPOS.Y);
    Serial.print("\tT: ");
    Serial.println(currentPOS.T);
    delay(1);
  }
}

/* Odometry */
void initOdometry(void) {
  currentPOS.X = lastPOS.X = currentPOS.Y = lastPOS.Y = currentPOS.T = 0;
  EncKanan.pulseCnt = EncBelakang.pulseCnt = EncKiri.pulseCnt = 0;
  EncKanan.lastpulse = EncBelakang.lastpulse = EncKiri.lastpulse = 0;
}

void fullResetENC(void) {
  initOdometry();
  slaveSerial.println(AT_RESET);
}

void updateOdometry(void) {
  updateCMPS();

  long ltime;
  float _x, _y, _t;
  // kecepatan rotasi posisi dan heading
  float rvPos1, rvPos2, rvPos3;

  long int
    // delta pulse
    dKiriENC,
    dKananENC, dBelakangENC,
    // pulsa sekarang
    currentBePC, currentKaPC, currentKiPC;

  currentBePC = EncBelakang.pulseCnt;
  currentKaPC = EncKanan.pulseCnt;
  currentKiPC = EncKiri.pulseCnt;

  dBelakangENC = currentBePC - EncBelakang.lastpulse;
  dKananENC = currentKaPC - EncKanan.lastpulse;
  dKiriENC = currentKiPC - EncKiri.lastpulse;

  /* POS ENC X AND Y */
  rvPos1 = dBelakangENC * POS_CONV_FACTOR;
  rvPos2 = dKananENC * POS_CONV_FACTOR;
  rvPos3 = dKiriENC * POS_CONV_FACTOR;

  _x = WHEEL_RADIUS * ((2 / 3.0 * rvPos1) - (1 / 3.0 * rvPos2) - (1 / 3.0 * rvPos3));
  _y = WHEEL_RADIUS * ((1 / MATH_ROOT_OF_3 * rvPos2) - (1 / MATH_ROOT_OF_3 * rvPos3));
  // _t = cmps.heading;
  _t = headingContinuous;

  currentPOS.X = lastPOS.X + ((MATH_COS(_t) * _x) - (MATH_SIN(_t) * _y));
  currentPOS.Y = lastPOS.Y + ((MATH_SIN(_t) * _x) + (MATH_COS(_t) * _y));
  currentPOS.T = _t;

  EncBelakang.lastpulse = currentBePC;
  EncKanan.lastpulse = currentKaPC;
  EncKiri.lastpulse = currentKiPC;

  lastPOS.X = currentPOS.X;
  lastPOS.Y = currentPOS.Y;
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
  dx = abs(x - currentPOS.X);
  dy = abs(y - currentPOS.Y);
  s = sqrt(dx * dx + dy * dy);

  // PENTING: Kecepatan linear SELALU ke arah target (global frame)
  if (s > MIN_POS_RES) {
    v = FuzzyLinear(s);
    // Velocity dalam global frame
    Vx = v * (dx / s);  // Komponen X menuju target
    Vy = v * (dy / s);  // Komponen Y menuju target
  } else {
    v = 0;
    Vx = 0;
    Vy = 0;
  }

  // ROTASI: Beda heading dengan target sudut t
  dw = t - currentPOS.T;
  // Normalisasi ke -180 sampai 180
  while (dw > 180) dw -= 360;
  while (dw < -180) dw += 360;

  // if (abs(dw) > MIN_DEG_RES) {
  //   w = FuzzyOmega(abs(dw));
  //   if (dw < 0) w *= -1;
  // } else {
  //   w = 0;
  // }

  // Cek sudah sampai
  if (s <= MIN_POS_RES && abs(dw) <= MIN_DEG_RES) {
    if (Move == true) {
      count++;
      if (count >= 2) setRPM(0, 0, 0);
    } else {
      count = 0;
      setRPM(0, 0, 0);
    }
  }

  // INVERSE KINEMATICS (Vx, Vy dalam GLOBAL frame!)
  // Konversi ke robot frame
  float Vx_robot = Vx * cos(radians(currentPOS.T)) + Vy * sin(radians(currentPOS.T));
  float Vy_robot = -Vx * sin(radians(currentPOS.T)) + Vy * cos(radians(currentPOS.T));

  // Hitung RPM
  w1 = (0.67 * Vx_robot) + (0 * Vy_robot) + (0.33 * w);
  w2 = (-0.33 * Vx_robot) + (0.58 * Vy_robot) + (0.33 * w);
  w3 = (-0.33 * Vx_robot) + (-0.58 * Vy_robot) + (0.33 * w);

  // Linear Motor [m/s];
  v1 = float(0.01 * w1);
  v2 = float(0.01 * w2);
  v3 = float(0.01 * w3);

  // convert To RPM
  RPM1 = (60 * v1) / (2 * MATH_PI * 0.075);  // 0.075 is radius motor
  RPM2 = (60 * v2) / (2 * MATH_PI * 0.075);
  RPM3 = (60 * v3) / (2 * MATH_PI * 0.075);

  setRPM(RPM1, RPM2, RPM3);


  Serial.print("\tdw: ");
  Serial.print(dw);
  Serial.print("\t Vx: ");
  Serial.print(Vx);
  Serial.print("\t Vy: ");
  Serial.print(Vy);
  Serial.print("\tw: ");
  Serial.println(w);

  return false;
}

void setRPM(int lRPM, int rRPM, int bRPM) {
  int rpm4 = 0;
  uint8_t packet[15];

  packet[0] = 0xAA;  // Header

  // Slave 1 data
  packet[1] = 0x00;                // D1H = CMD
  packet[2] = (bRPM >> 8) & 0xFF;  // D1M = RPM high byte
  packet[3] = bRPM & 0xFF;         // D1L = RPM low byte

  // Slave 2 data
  packet[4] = 0x00;                // D2H = CMD
  packet[5] = (rRPM >> 8) & 0xFF;  // D2M = RPM high byte
  packet[6] = rRPM & 0xFF;         // D2L = RPM low byte

  // Slave 3 data
  packet[7] = 0x00;                // D3H = CMD
  packet[8] = (lRPM >> 8) & 0xFF;  // D3M = RPM high byte
  packet[9] = lRPM & 0xFF;         // D3L = RPM low byte

  // Slave 4 data
  packet[10] = 0x00;                // D4H = CMD
  packet[11] = (rpm4 >> 8) & 0xFF;  // D4M = RPM high byte
  packet[12] = rpm4 & 0xFF;         // D4L = RPM low byte

  uint8_t checksum = 0;
  for (int i = 1; i <= 12; i++) {
    checksum ^= packet[i];
  }
  packet[13] = checksum;

  packet[14] = 0x55;
  slaveSerial.write(packet, 15);  // Slave 1
}

void encBelakang_INTT_A() {
  if (digitalRead(encBelakang_CHA) == HIGH) {
    if (digitalRead(encBelakang_CHB) == LOW) {
      EncBelakang.pulseCnt--;
    } else {
      EncBelakang.pulseCnt++;
    }
  } else {
    if (digitalRead(encBelakang_CHB) == HIGH) {
      EncBelakang.pulseCnt--;
    } else {
      EncBelakang.pulseCnt++;
    }
  }
}
void encBelakang_INTT_B() {
  if (digitalRead(encBelakang_CHA) == HIGH) {
    if (digitalRead(encBelakang_CHB) == HIGH) {
      EncBelakang.pulseCnt--;
    } else {
      EncBelakang.pulseCnt++;
    }
  } else {
    if (digitalRead(encBelakang_CHB) == LOW) {
      EncBelakang.pulseCnt--;
    } else {
      EncBelakang.pulseCnt++;
    }
  }
}
void encKanan_INTT_A() {
  if (digitalRead(encKanan_CHA) == HIGH) {
    if (digitalRead(encKanan_CHB) == LOW) {
      EncKanan.pulseCnt--;
    } else {
      EncKanan.pulseCnt++;
    }
  } else {
    if (digitalRead(encKanan_CHB) == HIGH) {
      EncKanan.pulseCnt--;
    } else {
      EncKanan.pulseCnt++;
    }
  }
}
void encKanan_INTT_B() {
  if (digitalRead(encKanan_CHA) == HIGH) {
    if (digitalRead(encKanan_CHB) == HIGH) {
      EncKanan.pulseCnt--;
    } else {
      EncKanan.pulseCnt++;
    }
  } else {
    if (digitalRead(encKanan_CHB) == LOW) {
      EncKanan.pulseCnt--;
    } else {
      EncKanan.pulseCnt++;
    }
  }
}
void encKiri_INTT_A() {
  if (digitalRead(encKiri_CHA) == HIGH) {
    if (digitalRead(encKiri_CHB) == LOW) {
      EncKiri.pulseCnt--;
    } else {
      EncKiri.pulseCnt++;
    }
  } else {
    if (digitalRead(encKiri_CHB) == HIGH) {
      EncKiri.pulseCnt--;
    } else {
      EncKiri.pulseCnt++;
    }
  }
}
void encKiri_INTT_B() {
  if (digitalRead(encKiri_CHA) == HIGH) {
    if (digitalRead(encKiri_CHB) == HIGH) {
      EncKiri.pulseCnt--;
    } else {
      EncKiri.pulseCnt++;
    }
  } else {
    if (digitalRead(encKiri_CHB) == LOW) {
      EncKiri.pulseCnt--;
    } else {
      EncKiri.pulseCnt++;
    }
  }
}



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



void controlCompute() {
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

  while (1) {
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
          setRPM(0, 0, 0);
          count = 2;  // Last Count
          break;
      }
    }

    if (nanoSerial.available()) {
      IR64 = nanoSerial.read();
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
        targetXYT.p2 = 254;
        targetXYT.p3 = 0;
      }
      if (IR64 == 'S') {  // R3
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = -254;
        targetXYT.p3 = 0;
      }
      if (IR64 == 'D') {  // Triangle
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 100;
        targetXYT.p2 = 0;
        targetXYT.p3 = 0;
      }
      if (IR64 == 'A') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = -100;
        targetXYT.p2 = 0;
        targetXYT.p3 = 0;
      }
      if (IR64 == 'X') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = 0;
        targetXYT.p3 = 180;
      }
      if (IR64 == 'C') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = 0;
        targetXYT.p3 = 0;
      }
      if (IR64 == 'Z') {  // Cross
        writeODM = false;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = 0;
        targetXYT.p3 = 180;
        setRPM(0, 0, 0);
      }
      if (IR64 == 'R') {  // Cross
        writeODM = false;
        MoveGoal = false;
        Move = false;
        setRPM(0, 0, 0);
        fullResetENC();
      }
    }
    static unsigned long l_print = 0;
    if (millis() - l_print > 5) {
      l_print = millis();
      nanoSerial.print("\t_X: ");
      nanoSerial.print(targetXYT.p1);
      nanoSerial.print("\t_Y: ");
      nanoSerial.print(targetXYT.p2);
      nanoSerial.print("\t_T: ");
      nanoSerial.print(targetXYT.p3);
      nanoSerial.print("\tX: ");
      nanoSerial.print(currentPOS.X);
      nanoSerial.print("\tY: ");
      nanoSerial.print(currentPOS.Y);
      nanoSerial.print("\tT: ");
      nanoSerial.println(currentPOS.T);
    }
  }
}