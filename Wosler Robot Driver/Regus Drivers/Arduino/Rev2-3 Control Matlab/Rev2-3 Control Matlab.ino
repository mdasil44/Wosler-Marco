//#define DEBUG

// CONFIG PARAMETERS
#define MAP(x, a, b, c, d) (c + (d - c) * (float(x) - a) / (b - a))
#define CONSTRAIN(x, a, b) (x <= a ? a : (x >= b ? b : x))

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define MIN_PRECISION_INT16 -32768
#define MAX_PRECISION_INT16 32767
#define OVER_MARGIN 10
#define PULSES_PER_MM 7.04545

// X Axis
#define XA_PIN 2
#define XB_PIN 3
#define X_HOME_PIN 7
#define X_MIN_READ 0
#define X_MAX_READ 1022
#define X_HOME_READ 0
#define X_MIN_POS -72
#define X_MAX_POS 72

// Y Axis
#define YA_PIN 0
#define YB_PIN 1
#define Y_HOME_PIN 6
#define Y_MIN_READ 0
#define Y_MAX_READ 779
#define Y_HOME_READ 0
#define Y_MIN_POS -56
#define Y_MAX_POS 56

// Z Axis
#define ZA_PIN 4
#define ZB_PIN 5
#define Z_HOME_PIN 8
#define Z_MIN_READ 0
#define Z_MAX_READ 930
#define Z_HOME_READ 930
#define Z_MIN_POS -66
#define Z_MAX_POS 66

// Wrist1
#define W1_PIN A7
#define W1_MIN_READ 823
#define W1_MAX_READ 2340
#define W1_MIN_POS 85.86
#define W1_MAX_POS -47.46

//Wrist2
#define W2_PIN A8
#define W2_MIN_READ 810
#define W2_MAX_READ 3350
#define W2_MIN_POS -111.44
#define W2_MAX_POS 111.44

//Wrist3
#define W3_PIN A9
#define W3_MIN_READ 0
#define W3_MAX_READ 4096
#define W3_MIN_POS -180.00
#define W3_MAX_POS 180.00

const uint8_t axisPins[2][6] = {{XA_PIN, YA_PIN, ZA_PIN, W1_PIN, W2_PIN, W3_PIN}, {XB_PIN, YB_PIN, ZB_PIN, 0, 0, 0}};
const int16_t readMins[6] = {X_MIN_READ, Y_MIN_READ, Z_MIN_READ, W1_MIN_READ, W2_MIN_READ, W3_MIN_READ};
const int16_t readMaxs[6] = {X_MAX_READ, Y_MAX_READ, Z_MAX_READ, W1_MAX_READ, W2_MAX_READ, W3_MAX_READ};
const float posMins[6] = {X_MIN_POS, Y_MIN_POS, Z_MIN_POS, W1_MIN_POS, W2_MIN_POS, W3_MIN_POS};
const float posMaxs[6] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS, W1_MAX_POS, W2_MAX_POS, W3_MAX_POS};
const uint8_t homePins[3] = {X_HOME_PIN, Y_HOME_PIN, Z_HOME_PIN};
const int16_t homes[3] = {X_HOME_READ, Y_HOME_READ, Z_HOME_READ};

const uint8_t HEART = 13;
uint8_t sendAgain = 0;
bool change = 0;
volatile int16_t count[3] = {(X_MIN_READ + X_MAX_READ) / 2, (Y_MIN_READ + Y_MAX_READ) / 2, (Z_MIN_READ + Z_MAX_READ) / 2};
int16_t Positions[6] = {count[0], count[1], count[2], 0, 0, 0};
int16_t prevPositions[6] = {count[0], count[1], count[2], 0, 0, 0};

void setup() {
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(axisPins[0][i], INPUT_PULLUP);
    if (i < 3) {
      pinMode(axisPins[1][i], INPUT_PULLUP);
      pinMode(homePins[i], INPUT_PULLUP);
    }
  }

  attachInterrupt(axisPins[0][0], tickXA, RISING);
  attachInterrupt(axisPins[0][1], tickYA, RISING);
  attachInterrupt(axisPins[0][2], tickZA, RISING);

  pinMode(HEART, OUTPUT);
  digitalWrite(HEART, HIGH);
  Serial.begin(115200);

#ifdef DEBUG
  while (!Serial);
#else
  EstablishConnection();
#endif

  digitalWrite(HEART, LOW);
  analogReadResolution(12);
}

void loop() {
#ifdef DEBUG
  while (1) {
    getPositions();
    if (change) {
      for (uint8_t i = 0; i < 6; i++) {
        Serial.print(Positions[i]); Serial.print(F("\t"));
      }
      Serial.println();
      delay(100);
    }
  }
#endif

  digitalWrite(HEART, sendAgain);

  if (Serial.available() > 0) {
    sendAgain = Serial.read();
    if (sendAgain == 5)
      WRITE_RESTART(0x5FA0004);
    flushData();
  }

  if (sendAgain) {
    getPositions();
    sendPositions();
    sendAgain = 0;
  }
}

void flushData() {
  while (Serial.available()) {
    Serial.read();
  }
}

void getPositions() {
  checkHomes();

  change = 0;
  for (uint8_t i = 0; i < 6; i++) {
    float val = 0;
    if (i < 3) {
      val = CONSTRAIN(count[i], readMins[i], readMaxs[i]);
      val = MAP(val, readMins[i], readMaxs[i], posMins[i], posMaxs[i]);
      Positions[i] = MAP(val, posMins[i], posMaxs[i], MIN_PRECISION_INT16, MAX_PRECISION_INT16);
    }
    else {
      val = MAP(analogRead(axisPins[0][i]), readMins[i], readMaxs[i], posMins[i], posMaxs[i]);
      Positions[i] = MAP(val, -180, 180, MIN_PRECISION_INT16, MAX_PRECISION_INT16);
    }

    if (Positions[i] != prevPositions[i])
      change = 1;
    prevPositions[i] = Positions[i];
  }
}

void sendPositions() {
  if (change) {
    for (uint8_t i = 0; i < 6; i++) {
      Serial.write(Positions[i] & 0xff); Serial.write((Positions[i] >> 8) & 0xff);
    }
  }
}

void EstablishConnection() {
  byte outByte = 1;
  while (1) {
    if (Serial.available() > 0) {
      flushData();
      Serial.write(outByte);
      break;
    }
  }
}

// X Encoder
void tickXA() {
  digitalRead(XA_PIN) != digitalRead(XB_PIN) ? --count[0] : ++count[0];
}

// Y Encoder
void tickYA() {
  digitalRead(YA_PIN) != digitalRead(YB_PIN) ? ++count[1] : --count[1];
}

// Z Encoder
void tickZA() {
  digitalRead(ZA_PIN) != digitalRead(ZB_PIN) ? ++count[2] : --count[2];
}

void checkHomes() {
  for (uint8_t i = 0; i < 3; i++) {
    if (!digitalRead(homePins[i]))
      count[i] = homes[i];
  }
}
