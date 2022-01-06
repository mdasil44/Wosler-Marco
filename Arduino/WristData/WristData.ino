//  In this rev3 design, the teensy is only used for its ADC: to obtain the position of the wrist.
//  The rest is handled by the raspberry pi (where both the odrive and the teensy are connected.


#define MAP(x, a, b, c, d) (c + (d - c) * (float(x) - a) / (b - a))
#define CONSTRAIN(x, a, b) (x <= a ? a : (x >= b ? b : x))

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

// Wrist1
#define W1_MIN_READ 823
#define W1_MAX_READ 2340
#define W1_MIN_POS 85.86
#define W1_MAX_POS -47.46

//Wrist2
#define W2_MIN_READ 810
#define W2_MAX_READ 3350
#define W2_MIN_POS -111.44
#define W2_MAX_POS 111.44

//Wrist3
#define W3_MIN_READ 0
#define W3_MAX_READ 4096
#define W3_MIN_POS -180.00
#define W3_MAX_POS 180.00

uint8_t wPins[3] = {A7, A8, A9};
int16_t wReadMins[3] = {W1_MIN_READ, W2_MIN_READ, W3_MIN_READ};
int16_t wReadMaxs[3] = {W1_MAX_READ, W2_MAX_READ, W3_MAX_READ};
float w[3] = {0, 0, 0};
float wPosMins[3] = {W1_MIN_POS, W2_MIN_POS, W3_MIN_POS};
float wPosMaxs[3] = {W1_MAX_POS, W2_MAX_POS, W3_MAX_POS};

const uint8_t HEART = 13;
uint8_t sendAgain = 0;
bool change = 0;
int16_t Positions[3] = {0, 0, 0};
int16_t prevPositions[6] = {0, 0, 0};

void setup() {
  pinMode(HEART, OUTPUT);
  digitalWrite(HEART, HIGH);
  Serial.begin(115200);
  //while (!Serial);
  EstablishConnection();
  digitalWrite(HEART, LOW);
  analogReadResolution(12);
}

void loop() {
//  while (1) {
//    getPositions();
//    if (change) {
//      for (uint8_t i = 0; i < 3; i++) {
//        Serial.print(Positions[i]); Serial.print(F("\t"));
//      }
//      Serial.println();
//    }
//    delay(100);
//  }

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
  change = 0;
  for (uint8_t i = 0; i < 3; i++) {
    w[i] = MAP(analogRead(wPins[i]), wReadMins[i], wReadMaxs[i], wPosMins[i], wPosMaxs[i]);
    Positions[i] = MAP(w[i], -180, 180, -32768, 32767);
    if (Positions[i] != prevPositions[i])
      change = 1;
    prevPositions[i] = Positions[i];
  }
}

void sendPositions() {
  if (change) {
    for (uint8_t i = 0; i < 3; i++) {
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
