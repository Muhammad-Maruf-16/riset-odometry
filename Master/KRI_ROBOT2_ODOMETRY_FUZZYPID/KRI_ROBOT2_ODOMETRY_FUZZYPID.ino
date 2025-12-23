////////////////////////////////////
//library
////////////////////////////////////
#include <Bluepad32.h>


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
// ========== GANTI BAGIAN INI SAJA (SEBELUM ODOMETRY) ==========

// Struct untuk gains
struct FuzzyGains {
  float Kp, Ki, Kd, FF;
} linearGains, angularGains;

// PID variables
float integral_linear = 0, last_error_linear = 0;
float integral_angular = 0, last_error_angular = 0;
unsigned long last_time_linear = 0, last_time_angular = 0;

// Base gains (bisa di-tune via serial)
float BASE_KP_LINEAR = 0.5;
float BASE_KI_LINEAR = 0.0;
float BASE_KD_LINEAR = 0.6;
float BASE_FF_LINEAR = 10.0;

float BASE_KP_ANGULAR = 0.5;
float BASE_KI_ANGULAR = 0.0;
float BASE_KD_ANGULAR = 0.6;
float BASE_FF_ANGULAR = 5.0;

// ========== KONFIGURASI SOFT START & SOFT BRAKE ==========
// [EDITABLE] - Atur jarak tahapan sesuai kebutuhan
#define VERY_FAR_DISTANCE   100.0  // >100cm: Full speed
#define FAR_DISTANCE        70.0   // 70-100cm: High speed  
#define MEDIUM_DISTANCE     40.0   // 40-70cm: Medium speed
#define NEAR_DISTANCE       20.0   // 20-40cm: Slow speed
#define VERY_NEAR_DISTANCE  10.0   // 10-20cm: Very slow
#define CRITICAL_DISTANCE   5.0    // 5-10cm: Ultra slow
// MIN_POS_RES = 3cm (stopping)

#define VERY_FAST_RATE      40.0   // Error rate >40: Very fast
#define FAST_RATE           25.0   // 25-40: Fast
#define MEDIUM_RATE         15.0   // 15-25: Medium
#define SLOW_RATE           8.0    // 8-15: Slow
#define VERY_SLOW_RATE      3.0    // <8: Very slow

// ========== FUZZY MEMBERSHIP FUNCTIONS ==========
// Distance membership functions
float fuzzy_very_far(float error) {
  if (error >= VERY_FAR_DISTANCE) return 1.0;
  if (error <= FAR_DISTANCE) return 0.0;
  return (error - FAR_DISTANCE) / (VERY_FAR_DISTANCE - FAR_DISTANCE);
}

float fuzzy_far(float error) {
  if (error <= MEDIUM_DISTANCE || error >= VERY_FAR_DISTANCE) return 0.0;
  if (error >= FAR_DISTANCE && error <= FAR_DISTANCE + 10) return 1.0;
  if (error < FAR_DISTANCE) return (error - MEDIUM_DISTANCE) / (FAR_DISTANCE - MEDIUM_DISTANCE);
  return (VERY_FAR_DISTANCE - error) / (VERY_FAR_DISTANCE - FAR_DISTANCE);
}

float fuzzy_medium(float error) {
  if (error <= NEAR_DISTANCE || error >= FAR_DISTANCE) return 0.0;
  if (error >= MEDIUM_DISTANCE && error <= MEDIUM_DISTANCE + 10) return 1.0;
  if (error < MEDIUM_DISTANCE) return (error - NEAR_DISTANCE) / (MEDIUM_DISTANCE - NEAR_DISTANCE);
  return (FAR_DISTANCE - error) / (FAR_DISTANCE - MEDIUM_DISTANCE);
}

float fuzzy_near(float error) {
  if (error <= VERY_NEAR_DISTANCE || error >= MEDIUM_DISTANCE) return 0.0;
  if (error >= NEAR_DISTANCE && error <= NEAR_DISTANCE + 5) return 1.0;
  if (error < NEAR_DISTANCE) return (error - VERY_NEAR_DISTANCE) / (NEAR_DISTANCE - VERY_NEAR_DISTANCE);
  return (MEDIUM_DISTANCE - error) / (MEDIUM_DISTANCE - NEAR_DISTANCE);
}

float fuzzy_very_near(float error) {
  if (error <= CRITICAL_DISTANCE || error >= NEAR_DISTANCE) return 0.0;
  if (error >= VERY_NEAR_DISTANCE && error <= VERY_NEAR_DISTANCE + 3) return 1.0;
  if (error < VERY_NEAR_DISTANCE) return (error - CRITICAL_DISTANCE) / (VERY_NEAR_DISTANCE - CRITICAL_DISTANCE);
  return (NEAR_DISTANCE - error) / (NEAR_DISTANCE - VERY_NEAR_DISTANCE);
}

float fuzzy_critical(float error) {
  if (error >= VERY_NEAR_DISTANCE) return 0.0;
  if (error <= CRITICAL_DISTANCE) return 1.0;
  return (VERY_NEAR_DISTANCE - error) / (VERY_NEAR_DISTANCE - CRITICAL_DISTANCE);
}

// Error rate membership functions  
float fuzzy_very_fast(float error_rate) {
  float rate = abs(error_rate);
  if (rate >= VERY_FAST_RATE) return 1.0;
  if (rate <= FAST_RATE) return 0.0;
  return (rate - FAST_RATE) / (VERY_FAST_RATE - FAST_RATE);
}

float fuzzy_fast(float error_rate) {
  float rate = abs(error_rate);
  if (rate <= MEDIUM_RATE || rate >= VERY_FAST_RATE) return 0.0;
  if (rate >= FAST_RATE && rate <= FAST_RATE + 5) return 1.0;
  if (rate < FAST_RATE) return (rate - MEDIUM_RATE) / (FAST_RATE - MEDIUM_RATE);
  return (VERY_FAST_RATE - rate) / (VERY_FAST_RATE - FAST_RATE);
}

float fuzzy_medium_rate(float error_rate) {
  float rate = abs(error_rate);
  if (rate <= SLOW_RATE || rate >= FAST_RATE) return 0.0;
  if (rate >= MEDIUM_RATE && rate <= MEDIUM_RATE + 3) return 1.0;
  if (rate < MEDIUM_RATE) return (rate - SLOW_RATE) / (MEDIUM_RATE - SLOW_RATE);
  return (FAST_RATE - rate) / (FAST_RATE - MEDIUM_RATE);
}

float fuzzy_slow(float error_rate) {
  float rate = abs(error_rate);
  if (rate <= VERY_SLOW_RATE || rate >= MEDIUM_RATE) return 0.0;
  if (rate >= SLOW_RATE && rate <= SLOW_RATE + 2) return 1.0;
  if (rate < SLOW_RATE) return (rate - VERY_SLOW_RATE) / (SLOW_RATE - VERY_SLOW_RATE);
  return (MEDIUM_RATE - rate) / (MEDIUM_RATE - SLOW_RATE);
}

float fuzzy_very_slow(float error_rate) {
  float rate = abs(error_rate);
  if (rate >= SLOW_RATE) return 0.0;
  if (rate <= VERY_SLOW_RATE) return 1.0;
  return (SLOW_RATE - rate) / (SLOW_RATE - VERY_SLOW_RATE);
}

// ========== FUZZY LINEAR CONTROLLER DENGAN SOFT BRAKING ==========
void fuzzy_linear_controller(float error, float error_rate) {
  // Calculate all membership values
  float very_far = fuzzy_very_far(error);
  float far = fuzzy_far(error);
  float medium = fuzzy_medium(error);
  float near_val = fuzzy_near(error);
  float very_near = fuzzy_very_near(error);
  float critical = fuzzy_critical(error);
  
  float very_fast_rate = fuzzy_very_fast(error_rate);
  float fast_rate = fuzzy_fast(error_rate);
  float medium_rate = fuzzy_medium_rate(error_rate);
  float slow_rate = fuzzy_slow(error_rate);
  float very_slow_rate = fuzzy_very_slow(error_rate);

  float kp_scale = 0, kd_scale = 0, ff_scale = 0, total_weight = 0;

  // ========== FUZZY RULES - 15 RULES COMPREHENSIVE ==========
  
  // Rule 1-2: Very Far + Very Fast/Fast -> MAXIMUM AGGRESSIVE
  float w1 = min(very_far, very_fast_rate);
  kp_scale += w1 * 1.8;  // High P for fast response
  kd_scale += w1 * 0.2;  // Low D to avoid oscillation
  ff_scale += w1 * 1.5;  // High FF for maximum speed
  total_weight += w1;
  
  float w2 = min(very_far, fast_rate);
  kp_scale += w2 * 1.7;
  kd_scale += w2 * 0.3;
  ff_scale += w2 * 1.4;
  total_weight += w2;

  // Rule 3-4: Far + Fast/Medium -> AGGRESSIVE
  float w3 = min(far, fast_rate);
  kp_scale += w3 * 1.5;
  kd_scale += w3 * 0.5;
  ff_scale += w3 * 1.3;
  total_weight += w3;
  
  float w4 = min(far, medium_rate);
  kp_scale += w4 * 1.4;
  kd_scale += w4 * 0.7;
  ff_scale += w4 * 1.2;
  total_weight += w4;

  // Rule 5-6: Medium + Medium/Slow -> MODERATE (CRUISING)
  float w5 = min(medium, medium_rate);
  kp_scale += w5 * 1.2;
  kd_scale += w5 * 1.0;
  ff_scale += w5 * 1.0;
  total_weight += w5;
  
  float w6 = min(medium, slow_rate);
  kp_scale += w6 * 1.1;
  kd_scale += w6 * 0.9;
  ff_scale += w6 * 0.9;
  total_weight += w6;

  // Rule 7-8: Near + Slow/Very Slow -> SOFT BRAKING START
  float w7 = min(near_val, slow_rate);
  kp_scale += w7 * 0.8;
  kd_scale += w7 * 1.3;
  ff_scale += w7 * 0.6;  // Reduced FF for braking
  total_weight += w7;
  
  float w8 = min(near_val, very_slow_rate);
  kp_scale += w8 * 0.7;
  kd_scale += w8 * 1.1;
  ff_scale += w8 * 0.5;
  total_weight += w8;

  // Rule 9-10: Very Near + Any -> STRONG BRAKING
  float w9 = min(very_near, medium_rate);
  kp_scale += w9 * 0.5;
  kd_scale += w9 * 1.5;
  ff_scale += w9 * 0.3;
  total_weight += w9;
  
  float w10 = min(very_near, slow_rate);
  kp_scale += w10 * 0.4;
  kd_scale += w10 * 1.8;
  ff_scale += w10 * 0.2;
  total_weight += w10;

  // Rule 11-12: Critical + Any -> ULTRA BRAKING
  float w11 = min(critical, medium_rate);
  kp_scale += w11 * 0.3;
  kd_scale += w11 * 2.0;
  ff_scale += w11 * 0.1;
  total_weight += w11;
  
  float w12 = min(critical, very_slow_rate);
  kp_scale += w12 * 0.2;
  kd_scale += w12 * 2.2;
  ff_scale += w12 * 0.05;
  total_weight += w12;

  // Rule 13-15: Safety rules for overshoot
  float w13 = min(very_near, very_fast_rate);
  kp_scale += w13 * 0.3;
  kd_scale += w13 * 2.5;  // Very high D to brake hard
  ff_scale += w13 * 0.05;
  total_weight += w13;

  float w14 = min(critical, fast_rate);
  kp_scale += w14 * 0.1;
  kd_scale += w14 * 3.0;
  ff_scale += w14 * 0.0;  // No FF
  total_weight += w14;

  float w15 = min(critical, very_fast_rate);
  kp_scale += w15 * 0.05;
  kd_scale += w15 * 3.5;
  ff_scale += w15 * 0.0;
  total_weight += w15;

  // Normalize
  if (total_weight > 0.01) {
    kp_scale /= total_weight;
    kd_scale /= total_weight;
    ff_scale /= total_weight;
  } else {
    // Default moderate gains
    kp_scale = 1.0;
    kd_scale = 1.0;
    ff_scale = 1.0;
  }

  // Apply SOFT BRAKING by reducing gains near target
  float brake_factor = 1.0;
  if (error < VERY_NEAR_DISTANCE) {
    // Progressive braking when very close
    brake_factor = max(0.1, error / VERY_NEAR_DISTANCE);
  }
  
  linearGains.Kp = BASE_KP_LINEAR * kp_scale * brake_factor;
  linearGains.Ki = BASE_KI_LINEAR;
  linearGains.Kd = BASE_KD_LINEAR * kd_scale;
  linearGains.FF = BASE_FF_LINEAR * ff_scale * brake_factor; // FF reduction is key for soft brake!
}

// ========== FUZZY ANGULAR CONTROLLER ==========
void fuzzy_angular_controller(float error, float error_rate) {
  float error_abs = abs(error);
  float rate_abs = abs(error_rate);
  
  // Simplified angular fuzzy for stability
  float kp_scale = 1.0, kd_scale = 1.0, ff_scale = 1.0;
  
  if (error_abs > 45.0) {
    // Large error - aggressive
    kp_scale = 1.4;
    kd_scale = 0.4;
    ff_scale = 1.3;
  } else if (error_abs > 20.0) {
    // Medium error - moderate
    kp_scale = 1.1;
    kd_scale = 0.8;
    ff_scale = 1.0;
  } else if (error_abs > 8.0) {
    // Small error - precise
    kp_scale = 0.8;
    kd_scale = 1.2;
    ff_scale = 0.6;
  } else {
    // Very small error - ultra precise with braking
    kp_scale = 0.5;
    kd_scale = 1.5;
    ff_scale = 0.3;
  }
  
  // Additional damping based on rate
  if (rate_abs > 25.0) {
    kd_scale *= 1.3; // More damping if rotating too fast
  }

  angularGains.Kp = BASE_KP_ANGULAR * kp_scale;
  angularGains.Ki = BASE_KI_ANGULAR;
  angularGains.Kd = BASE_KD_ANGULAR * kd_scale;
  angularGains.FF = BASE_FF_ANGULAR * ff_scale;
}

// ========== IMPROVED PID CALCULATION ==========
float pid_linear(float error) {
  // Proper derivative with time
  unsigned long current_time = millis();
  float delta_time = (current_time - last_time_linear) / 1000.0;
  if (delta_time <= 0) delta_time = 0.01;
  
  float error_rate = (error - last_error_linear) / delta_time;
  last_error_linear = error;
  last_time_linear = current_time;

  // Fuzzy adaptive gains
  fuzzy_linear_controller(error, error_rate);

  // Improved integral with anti-windup
  if (abs(error) < 50.0) { // Only integrate when error is reasonable
    integral_linear += error * delta_time;
  }
  
  // Clamp integral
  if (integral_linear > 100) integral_linear = 100;
  if (integral_linear < -100) integral_linear = -100;

  // PID calculation
  float output = linearGains.Kp * error + 
                 linearGains.Ki * integral_linear + 
                 linearGains.Kd * error_rate +
                 linearGains.FF;

  return output;
}

float last_angle_error ;
float pid_angular(float error) {
  // Proper derivative with time
  unsigned long current_time = millis();
  float delta_time = (current_time - last_time_angular) / 1000.0;
  if (delta_time <= 0) delta_time = 0.01;
  
  float error_rate = (error - last_angle_error) / delta_time;
  last_angle_error = error;
  last_time_angular = current_time;

  fuzzy_angular_controller(error, error_rate);

  // Improved integral
  if (abs(error) < 30.0) {
    integral_angular += error * delta_time;
  }
  
  if (integral_angular > 50) integral_angular = 50;
  if (integral_angular < -50) integral_angular = -50;

  float output = angularGains.Kp * error + 
                 angularGains.Ki * integral_angular + 
                 angularGains.Kd * error_rate +
                 angularGains.FF;

  return output;
}

// ========== IMPROVED goXYT DENGAN VELOCITY PROFILING ==========
bool goXYT(int x, int y, int t) {
  updateOdometry();
  float s, v, w, Vx, Vy;
  float alpha, TB, dx, dy;
  float w1, w2, w3, v1, v2, v3;
  float RPM1, RPM2, RPM3, dw;

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
    integral_linear = 0;
    integral_angular = 0;
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

  // ========== FUZZY-PID LINEAR DENGAN SOFT BRAKING ==========
  if (s > MIN_POS_RES) {
    float pid_output = pid_linear(s);
    
    // Additional velocity profiling for extra smoothness
    float max_velocity = 150.0;
    if (s < VERY_NEAR_DISTANCE) {
      max_velocity = 50.0; // Ultra slow near target
    } else if (s < NEAR_DISTANCE) {
      max_velocity = 80.0; // Very slow
    } else if (s < MEDIUM_DISTANCE) {
      max_velocity = 120.0; // Slow
    }
    
    v = pid_output + linearGains.FF;
    
    // Soft limiting
    if (v > max_velocity) v = max_velocity;
    if (v < -max_velocity) v = -max_velocity;
    
    // Minimum velocity to prevent stalling
    if (abs(v) < 10.0 && s > 15.0) {
      v = (v >= 0) ? 10.0 : -10.0;
    }
  }

  // ========== FUZZY-PID ANGULAR ==========
  if (abs(dw) > MIN_DEG_RES) {
    float pid_output = pid_angular(abs(dw));
    w = pid_output + angularGains.FF;
    if (dw < 0) w *= -1;
    
    // Angular velocity limiting
    float max_angular = 80.0;
    if (abs(dw) < 10.0) {
      max_angular = 30.0; // Slower rotation for fine adjustment
    }
    if (w > max_angular) w = max_angular;
    if (w < -max_angular) w = -max_angular;
  }

  // Hitung kecepatan arah sumbu X dan Y (Vektor)
  Vx = v * cos(radians(alpha));
  Vy = v * sin(radians(alpha));

  // angular Motor [rad/s]
  w1 = (0.67 * Vx) + (0 * Vy) + (0.33 * w);
  w2 = (-0.33 * Vx) + (0.58 * Vy) + (0.33 * w);
  w3 = (-0.33 * Vx) + (-0.58 * Vy) + (0.33 * w);

  // Linear Motor [m/s];
  v1 = float(0.01 * w1);
  v2 = float(0.01 * w2);
  v3 = float(0.01 * w3);

  // convert To RPM
  RPM1 = (60 * v1) / (2 * MATH_PI * 0.075);
  RPM2 = (60 * v2) / (2 * MATH_PI * 0.075);
  RPM3 = (60 * v3) / (2 * MATH_PI * 0.075);

  sendToSlave(-RPM3, -RPM2, -RPM1);

  // Debug info
  Serial.printf("s:%.1f v:%.1f Kp:%.2f FF:%.1f dw:%.1f w:%.1f\n", 
                s, v, linearGains.Kp, linearGains.FF, dw, w);

  return false;
}


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

  if (cross) {  // START
    writeODM = false;
    MoveGoal = true;
    Move = true;
    count = 0;
  }
  if (up) {  // L3
    writeODM = true;
    MoveGoal = false;
    Move = false;
    targetXYT.p1 = 0;
    targetXYT.p2 = 100;
    targetXYT.p3 = 0;
  }
  if (down) {  // Cross
    writeODM = true;
    MoveGoal = false;
    Move = false;
    targetXYT.p1 = 0;
    targetXYT.p2 = -100;
    targetXYT.p3 = 0;
  }
  if (R1) {  // R3
    writeODM = true;
    MoveGoal = false;
    Move = false;
    targetXYT.p1 = -100;
    targetXYT.p2 = 0;
    targetXYT.p3 = 0;
  }
  if (L1) {  // Triangle
    writeODM = true;
    MoveGoal = false;
    Move = false;
    targetXYT.p1 = 100;
    targetXYT.p2 = 0;
    targetXYT.p3 = 0;
  }
  if (circle) {  // Cross
    writeODM = true;
    MoveGoal = false;
    Move = false;
    targetXYT.p1 = 0;
    targetXYT.p2 = 0;
    targetXYT.p3 = 0;
  }
  if (triangle) {
    writeODM = true;
    MoveGoal = false;
    Move = false;
    targetXYT.p1 = 0;
    targetXYT.p2 = 0;
    targetXYT.p3 = 0;
    fullResetENC();
  }
}
// controlCompute() dan fungsi lainnya TETAP SAMA...

// ========== CONTROL COMPUTE TETAP SAMA PERSIS ==========
// (Pakai yang dari kode asli kamu, TIDAK DIUBAH)

// ========== SERIAL TUNING (OPSIONAL) ==========
void parseBaseFuzzyGains(String serialBuffer) {
  float vals[8];
  int idx = 0, start = 0;
  for (int i = 0; i < serialBuffer.length(); i++) {
    if (serialBuffer[i] == ',' || i == serialBuffer.length() - 1) {
      String part = (i == serialBuffer.length() - 1) ? serialBuffer.substring(start) : serialBuffer.substring(start, i);
      vals[idx++] = part.toFloat();
      start = i + 1;
      if (idx >= 8) break;
    }
  }
  if (idx == 8) {
    BASE_KP_LINEAR = vals[0];
    BASE_KI_LINEAR = vals[1];
    BASE_KD_LINEAR = vals[2];
    BASE_FF_LINEAR = vals[3];
    BASE_KP_ANGULAR = vals[4];
    BASE_KI_ANGULAR = vals[5];
    BASE_KD_ANGULAR = vals[6];
    BASE_FF_ANGULAR = vals[7];
    Serial.printf("BASE OK: %.2f %.2f %.2f %.1f | %.2f %.2f %.2f %.1f\n",
                  vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7]);
  }
}


// Serial.printf("%.2f, %.2f, %.2f, %.2f\n", RPM1,RPM2,RPM3, cmps.heading);

#define LED 2
bool state = false;

void TaskBluetoothSend(void *pvParameters) {
  (void)pvParameters;
  const TickType_t delayTime = pdMS_TO_TICKS(1);

  String serialBuffer = "";  // Deklarasi di sini

  while (true) {
    BP32.update();
    processControllers();
    digitalWrite(LED, up);
    // sendToSlave();

    // while (slaveSerial.available()) {
    //   char c = slaveSerial.read();

    //   if (c != '\n' && c != '\r') {
    //     serialBuffer += c;
    //   }

    //   if (c == '\n') {

    //     // Pecah jadi 8 bagian
    //     float vals[8];
    //     int idx = 0;
    //     int start = 0;

    //     for (int i = 0; i < serialBuffer.length(); i++) {
    //       if (serialBuffer[i] == ',' || i == serialBuffer.length() - 1) {

    //         String part;
    //         if (i == serialBuffer.length() - 1)
    //           part = serialBuffer.substring(start);
    //         else
    //           part = serialBuffer.substring(start, i);

    //         vals[idx++] = part.toFloat();
    //         start = i + 1;

    //         if (idx >= 8) break;
    //       }
    //     }

    //     if (idx == 8) {

    //       Kp_linear = vals[0];
    //       Ki_linear = vals[1];
    //       Kd_linear = vals[2];
    //       feedforward_linear = vals[3];
    //       Kp_angular = vals[4];
    //       Ki_angular = vals[5];
    //       Kd_angular = vals[6];
    //       feedforward_angular = vals[7];

    //       Serial.printf("OK: %.3f %.3f %.3f %.3f | %.3f %.3f %.3f %.3f\n",
    //                     vals[0], vals[1], vals[2], vals[3],
    //                     vals[4], vals[5], vals[6], vals[7]);
    //     } else {
    //       Serial.println("Format salah! 8 data diperlukan");
    //     }

    //     serialBuffer = "";
    //   }
    // }

    if (R2) {
      state = true;
      writeODM = true;
      MoveGoal = false;
      Move = false;
      targetXYT.p1 = 0;
      targetXYT.p2 = 0;
      targetXYT.p3 = 0;
      fullResetENC();
    }
    if (L2) state = false;

    vTaskDelay(delayTime);  // Delay antar kirim
  }
}

void TaskProcess(void *pvParameters) {
  (void)pvParameters;
  const TickType_t delayTime = pdMS_TO_TICKS(1);
  while (true) {
    if (state) {
      run();
    } else {
      if (square_) {
        fullResetENC();
      }
      controlCompute();
    }
    // updateOdometry();
    // Serial.printf("x %.2f, y %.2f, t %.2f\n", currentPos.X, currentPos.Y, currentPos.T);
    // Serial.printf(" %d, %d, %d\n", EncKanan.pulseCnt,  EncKiri.pulseCnt,EncDepan.pulseCnt);
    vTaskDelay(delayTime);  // Delay antar kirim
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  slaveSerial.begin(115200, SERIAL_8N1, 13, 26);  // Serial ke slave
  GY25Serial.begin(115200, SERIAL_8N1, 27, 14);
  BP32.setup(&onConnectedController, &onDisconnectedController);

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
