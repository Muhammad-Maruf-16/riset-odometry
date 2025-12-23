
  GLOBAL VELOCITY full VERSI FINAL
  PERBEDAAN DENGAN VERSI SEBELUMNYA FUZZY DIGANTIKAN PAKAI PID

*/

#define slaveSerial Serial3
#define cmpSerial Serial2
#define nanoSerial Serial1

/*COMMAND*/
#define AT_RESET "RESET"
#define AT_RPS "RPS"

/* ENCODER */
#define encBelakang_CHA 45
#define encBelakang_CHB 43
#define encKanan_CHA 49
#define encKanan_CHB 47
#define encKiri_CHA 53
#define encKiri_CHB 51

// #define encKanan_CHA 33
// #define encKanan_CHB 31
// #define encKiri_CHA 37
// #define encKiri_CHB 35

/* ODOMETRY */
#define MATH_PI 3.14286
#define MATH_ROOT_OF_3 1.73205
#define WHEEL_RADIUS 2.9
#define WHEEL_BASE 9.40
#define ENC_RESOLUTION 1600.0
#define MIN_POS_RES 3.0
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

// ===== PID PARAMETERS =====
// PID Linear
float KP_LINEAR = 1.7;
float KI_LINEAR = 0.0;
float KD_LINEAR = 0.6;
float MAX_V_OUTPUT = 250.0;

// PID Angular
float KP_ANGULAR = 1.0;
float KI_ANGULAR = 0.0;
float KD_ANGULAR = 0.5;
float MAX_W_OUTPUT = 75.0;

// ===== PID STATE =====
float error_prev_linear = 0;
float integral_linear = 0;
unsigned long lastTime_linear = 0;

float error_prev_angular = 0;
float integral_angular = 0;
unsigned long lastTime_angular = 0;





float headingContinuous = 0;
float lastHeading = 0;

int RPM_BE, RPM_KA, RPM_KI;


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

float sin_t;
float cos_t;

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
  SerialUSB.begin(115200);
  cmpSerial.begin(115200);
  slaveSerial.begin(115200);
  // fullResetENC();
  nanoSerial.begin(115200);

  pinMode(encBelakang_CHA, INPUT);
  pinMode(encBelakang_CHB, INPUT);
  pinMode(encKanan_CHA, INPUT);
  pinMode(encKanan_CHB, INPUT);
  pinMode(encKiri_CHA, INPUT);
  pinMode(encKiri_CHB, INPUT);
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
    SerialUSB.print("BE: ");
    SerialUSB.print(EncBelakang.pulseCnt);
    SerialUSB.print("\tKA: ");
    SerialUSB.print(EncKanan.pulseCnt);
    SerialUSB.print("\tKI: ");
    SerialUSB.print(EncKiri.pulseCnt);
    SerialUSB.print("\tX: ");
    SerialUSB.print(currentPOS.X);
    SerialUSB.print("\tY: ");
    SerialUSB.print(currentPOS.Y);
    SerialUSB.print("\tT: ");
    SerialUSB.println(currentPOS.T);
    delay(5);
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

  // _t = cmps.heading;
  _t = headingContinuous;
  float headingRad = radians(_t);
  sin_t = sin(headingRad);
  cos_t = cos(headingRad);

  /*forward kinematics v1*/
  // _x = WHEEL_RADIUS * ((2 / 3.0 * rvPos1) - (1 / 3.0 * rvPos2) - (1 / 3.0 * rvPos3));
  // _y = WHEEL_RADIUS * ((1 / MATH_ROOT_OF_3 * rvPos2) - (1 / MATH_ROOT_OF_3 * rvPos3));

  /*forward kinematics v2*/
  // _x = WHEEL_RADIUS * 0.5 * (rvPos2 - rvPos3);  // FLIP dari usulan awal
  // _y = WHEEL_RADIUS * (rvPos1 - 0.866025404 * (rvPos2 + rvPos3));

  /*forward kinematics v3*/
  // _x = WHEEL_RADIUS * (0.577350269) * (rvPos2 - rvPos3);
  // _y = WHEEL_RADIUS * (0.666666667) * (rvPos1 - 0.333333333 * (rvPos2 + rvPos3));

  /*forward kinematics v4*/
  // _x = WHEEL_RADIUS * (0.66667 * rvPos1 - 0.33333 * (rvPos2 + rvPos3));
  // _y = WHEEL_RADIUS * 0.57735 * (rvPos2 - rvPos3);

  /*forward kinematics v5*/
  _x = WHEEL_RADIUS * 0.57735 * (rvPos2 - rvPos3);
  _y = WHEEL_RADIUS * (0.66667 * rvPos1 - 0.33333 * (rvPos2 + rvPos3));

  /*TRANSFORMASI LOKAL KE GLOBAL V1*/
  // currentPOS.X = lastPOS.X + ((cos(radians(_t)) * _x) - (sin(radians(_t)) * _y));
  // currentPOS.Y = lastPOS.Y + ((sin(radians(_t)) * _x) + (cos(radians(_t)) * _y));

  /*TRANSFORMASI LOKAL KE GLOBAL V2*/
  // currentPOS.X = lastPOS.X + ((sin(radians(_t)) * _x) + (cos(radians(_t)) * _y));
  // currentPOS.Y = lastPOS.Y + ((cos(radians(_t)) * _x) - (sin(radians(_t)) * _y));

  /*TRANSFORMASI LOKAL KE GLOBAL V3*/
  currentPOS.X = lastPOS.X + (sin_t * _x + cos_t * _y);
  currentPOS.Y = lastPOS.Y + (cos_t * _x - sin_t * _y);
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
  dx = x - currentPOS.X;
  dy = y - currentPOS.Y;

  s = sqrt(dx * dx + dy * dy);
  dw = t - currentPOS.T;

  if (s <= MIN_POS_RES) {
    v = 0;
    if (s <= MIN_POS_RES && abs(dw) <= MIN_DEG_RES && Move == true) {
      count++;
      if (count == 2) {
        setRPM(0, 0, 0);
      }
    }
    if (s <= MIN_POS_RES && abs(dw) <= MIN_DEG_RES && Move == false) {
      count = 0;
      setRPM(0, 0, 0);
    }
  }

  // if (s > MIN_POS_RES) {
  //   v = FuzzyLinear(s);
  // }

  if (abs(dw) > MIN_DEG_RES) {
    w = 1.5 * FuzzyOmega(abs(dw));
    if (dw < 0) {
      w *= -1;
    }
  }

  if (s > MIN_POS_RES) {
    v = PIDLinear(s);
  } else {
    v = 0;
    integral_linear = 0;  // reset integral
  }

  // if (abs(dw) > MIN_DEG_RES) {
  //   // w = PIDAngular(abs(dw));  // dw sudah ada tanda +/-
  //   // if (dw < 0) {
  //   //   w *= -1;
  //   // }

  //   w = PIDAngular(dw);  // dw sudah ada tanda +/-
  // } else {
  //   w = 0;
  //   integral_angular = 0;  // reset integral
  // }

  // Hitung kecepatan arah sumbu X dan Y (Vektor)
  if (s > 0.0) {
    Vx = v * dx / s;
    Vy = v * dy / s;
  } else {
    Vx = 0;
    Vy = 0;
  }


  /*TRANSFORMASI GLOBAL TO LOCAL v1*/
  // float Vx_local = (sin_t * Vx) + (cos_t * Vy);
  // float Vy_local = (cos_t * Vx) - (sin_t * Vy);

  /*TRANSFORMASI GLOBAL TO LOCAL v2*/
  float Vx_local = (cos_t * Vx) - (sin_t * Vy);
  float Vy_local = (sin_t * Vx) + (cos_t * Vy);

  /*angular Motor [rad/s] v1 GLOBAL VELOCITY*/
  // w1 = (0.67 * Vx) + (0 * Vy) + (0.33 * w);
  // w2 = (-0.33 * Vx) + (0.58 * Vy) + (0.33 * w);
  // w3 = (-0.33 * Vx) + (-0.58 * Vy) + (0.33 * w);

  /*angular Motor [rad/s] v2 LOCAL VELOCITY*/
  w1 = (0.667 * Vx_local) + (0.33 * w);
  w2 = (-0.333 * Vx_local) + (0.577 * Vy_local) + (0.333 * w);
  w3 = (-0.333 * Vx_local) + (-0.577 * Vy_local) + (0.333 * w);

  // w1 = (0.667 * Vx_local);
  // w2 = (-0.333 * Vx_local) + (0.577 * Vy_local);
  // w3 = (-0.333 * Vx_local) + (-0.577 * Vy_local);

  // Linear Motor [m/s];
  v1 = float(0.029 * w1);
  v2 = float(0.029 * w2);
  v3 = float(0.029 * w3);

  // v1 = float(0.01 * w1);
  // v2 = float(0.01 * w2);
  // v3 = float(0.01 * w3);

  // convert To RPM
  RPM1 = (60 * v1) / (2 * MATH_PI * 0.075);  // 0.075 is radius motor
  RPM2 = (60 * v2) / (2 * MATH_PI * 0.075);
  RPM3 = (60 * v3) / (2 * MATH_PI * 0.075);

  setRPM(RPM1, RPM2, RPM3);

  RPM_BE = RPM1;
  RPM_KI = RPM2;
  RPM_KA = RPM3;

  return false;
}


void setRPM(int bRPM, int lRPM, int rRPM) {
  int rpm4 = 0;
  uint8_t packet[15];

  packet[0] = 0xAA;  // Header

  // Slave 1 data
  packet[1] = 0x00;                // D1H = CMD
  packet[2] = (bRPM >> 8) & 0xFF;  // D1M = RPM high byte
  packet[3] = bRPM & 0xFF;         // D1L = RPM low byte

  // Slave 2 data
  packet[4] = 0x00;                // D2H = CMD
  packet[5] = (lRPM >> 8) & 0xFF;  // D2M = RPM high byte
  packet[6] = lRPM & 0xFF;         // D2L = RPM low byte

  // Slave 3 data
  packet[7] = 0x00;                // D3H = CMD
  packet[8] = (rRPM >> 8) & 0xFF;  // D3M = RPM high byte
  packet[9] = rRPM & 0xFF;         // D3L = RPM low byte

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
  // Serial.write(packet, 15);  // Slave 1
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


// ===== PID LINEAR =====
float PIDLinear(float error) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime_linear) / 1000.0;

  if (lastTime_linear == 0 || dt > 1.0) {
    dt = 0.02;
  }
  lastTime_linear = currentTime;

  // P
  float P = KP_LINEAR * error;

  // I
  integral_linear += error * dt;
  integral_linear = constrain(integral_linear, -50.0, 50.0);
  float I = KI_LINEAR * integral_linear;

  // D
  float D = KD_LINEAR * (error - error_prev_linear) / dt;
  error_prev_linear = error;

  // Output
  float output = P + I + D;
  return constrain(output, 0, MAX_V_OUTPUT);
}

// ===== PID ANGULAR =====
float PIDAngular(float error) {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime_angular) / 1000.0;

  if (lastTime_angular == 0 || dt > 1.0) {
    dt = 0.02;
  }
  lastTime_angular = currentTime;

  // P
  float P = KP_ANGULAR * error;

  // I
  integral_angular += error * dt;
  integral_angular = constrain(integral_angular, -30.0, 30.0);
  float I = KI_ANGULAR * integral_angular;

  // D
  float D = KD_ANGULAR * (error - error_prev_angular) / dt;
  error_prev_angular = error;

  // Output
  float output = P + I + D;
  return constrain(output, -MAX_W_OUTPUT, MAX_W_OUTPUT);
}

// ===== RESET PID =====
void resetPID() {
  error_prev_linear = 0;
  integral_linear = 0;
  lastTime_linear = 0;

  error_prev_angular = 0;
  integral_angular = 0;
  lastTime_angular = 0;
}

unsigned long lTime10 = 0;

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
    // { 0, 250, 0 },
    // { 0, 400, 0 },
    // { 0, 0, 0 },
    { 0, 250, 0 },
    { 0, 400, 0 },
    { 0, 550, 0 },
    { 200, 550, 0 },
    { -80, 550, 0 },
    { -80, 700, 0 },
    { 0, 400, 0 },
    { 0, 0, 0 },
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
          SerialUSB.println("\t case 0 \t");
          lTime10 = millis();
          break;
        case 1:
          if (millis() - lTime10 > 1000) count = 2;
          SerialUSB.println(lTime10 - millis() + "\t case 1 \t");
          break;
        case 2:
          goXYT(ZoneArr[1][0], ZoneArr[1][1], ZoneArr[1][2]);
          SerialUSB.println("\t case 2 \t");
          lTime10 = millis();
          break;
        case 3:
          if (millis() - lTime10 > 1000) count = 4;
          SerialUSB.println(lTime10 - millis() + "\t case 3 \t");
          break;
        case 4:
          goXYT(ZoneArr[2][0], ZoneArr[2][1], ZoneArr[2][2]);
          SerialUSB.println("\t case 4 \t");
          lTime10 = millis();
          break;
        case 5:
          if (millis() - lTime10 > 1000) count = 6;
          SerialUSB.println(lTime10 - millis() + "\t case 5 \t");
          break;
        case 6:
          goXYT(ZoneArr[3][0], ZoneArr[3][1], ZoneArr[3][2]);
          SerialUSB.println("\t case 6 \t");
          lTime10 = millis();
          break;
        case 7:
          if (millis() - lTime10 > 1000) count = 8;
          SerialUSB.println(lTime10 - millis() + "\t case 7 \t");
          break;
        case 8:
          goXYT(ZoneArr[4][0], ZoneArr[4][1], ZoneArr[4][2]);
          SerialUSB.println("\t case 8 \t");
          lTime10 = millis();
          break;
        case 9:
          if (millis() - lTime10 > 1000) count = 10;
          SerialUSB.println(lTime10 - millis() + "\t case 9 \t");
          break;
        case 10:
          goXYT(ZoneArr[5][0], ZoneArr[5][1], ZoneArr[5][2]);
          SerialUSB.println("\t case 10 \t");
          lTime10 = millis();
          break;
        case 11:
          if (millis() - lTime10 > 1000) count = 12;
          SerialUSB.println(lTime10 - millis() + "\t case 11 \t");
          break;
        case 12:
          goXYT(ZoneArr[6][0], ZoneArr[6][1], ZoneArr[6][2]);
          SerialUSB.println("\t case 12 \t");
          lTime10 = millis();
          break;
        case 13:
          if (millis() - lTime10 > 1000) count = 14;
          SerialUSB.println(lTime10 - millis() + "\t case 13 \t");
          break;
        case 14:
          goXYT(ZoneArr[7][0], ZoneArr[7][1], ZoneArr[7][2]);
          SerialUSB.println("\t case 14 \t");
          lTime10 = millis();
          break;
        case 15:
          if (millis() - lTime10 > 1000) count = 16;
          SerialUSB.println(lTime10 - millis() + "\t case 15 \t");
          break;
        case 16:
          goXYT(ZoneArr[8][0], ZoneArr[8][1], ZoneArr[8][2]);
          SerialUSB.println("\t case 16 \t");
          lTime10 = millis();
          break;
        case 17:
          if (millis() - lTime10 > 1000) count = 18;
          SerialUSB.println(lTime10 - millis() + "\t case 17 \t");
          break;
        case 18:
          goXYT(ZoneArr[9][0], ZoneArr[9][1], ZoneArr[9][2]);
          SerialUSB.println("\t case 18 \t");
          lTime10 = millis();
          break;
        default:
          setRPM(0, 0, 0);
          SerialUSB.println("\t done \t");
          count = 18;  // Last Count
          break;
      }
    }

    if (nanoSerial.available()) {
      IR64 = nanoSerial.read();
      SerialUSB.println(IR64);
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
        resetPID();
      }
      if (IR64 == 'S') {  // R3
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = -254;
        targetXYT.p3 = 0;
        resetPID();
      }
      if (IR64 == 'D') {  // Triangle
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 100;
        targetXYT.p2 = 0;
        targetXYT.p3 = 0;
        resetPID();
      }
      if (IR64 == 'A') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = -100;
        targetXYT.p2 = 0;
        targetXYT.p3 = 0;
        resetPID();
      }
      if (IR64 == 'X') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = 0;
        targetXYT.p3 = 180;
        resetPID();
      }
      if (IR64 == 'C') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = 0;
        targetXYT.p3 = 0;
        resetPID();
      }
      if (IR64 == 'Z') {  // Cross
        writeODM = false;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = 0;
        targetXYT.p3 = 180;
        resetPID();
        setRPM(0, 0, 0);
      }

      if (IR64 == 'U') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = -100;
        targetXYT.p2 = 100;
        targetXYT.p3 = 0;
        resetPID();
      }

      if (IR64 == 'I') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 100;
        targetXYT.p2 = 100;
        targetXYT.p3 = 0;
        resetPID();
      }

      if (IR64 == 'J') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = -100;
        targetXYT.p2 = -100;
        targetXYT.p3 = 0;
        resetPID();
      }

      if (IR64 == 'K') {  // Cross
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 100;
        targetXYT.p2 = -100;
        targetXYT.p3 = 0;
        resetPID();
      }

      if (IR64 == 'R') {  // Cross
        writeODM = false;
        MoveGoal = false;
        Move = false;
        setRPM(0, 0, 0);
        fullResetENC();
        resetPID();
        // initOdometry();
      }
      if (IR64 == 'T') {  // Cross
        writeODM = false;
        MoveGoal = false;
        Move = false;
        setRPM(0, 0, 0);
        resetCMPS();
        resetPID();
        // initOdometry();
      }
      if (IR64 == 'M') {  // L3
        writeODM = true;
        MoveGoal = false;
        Move = false;
        targetXYT.p1 = 0;
        targetXYT.p2 = 200;
        targetXYT.p3 = 120;
        resetPID();
      }
    }
    static unsigned long l_print = 0;
    if (millis() - l_print > 20) {
      l_print = millis();
      nanoSerial.print("\t\tDEPAN ");
      nanoSerial.print(RPM_BE);
      nanoSerial.print("\t\tKANAN ");
      nanoSerial.print(RPM_KA);
      nanoSerial.print("\t\tKIRI ");
      nanoSerial.print(RPM_KI);
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