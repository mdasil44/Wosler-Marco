const uint8_t xA = 2; const uint8_t xB = 3;
const uint8_t yA = 0; const uint8_t yB = 1;
const uint8_t zA = 4; const uint8_t zB = 5;
const uint8_t w1A = 6; const uint8_t w1B = 7;
const uint8_t w2A = 8; const uint8_t w2B = 9;
//const uint8_t w3A = 8; const uint8_t w2B = 9;

const uint8_t homeX = 11;
const uint8_t homeY = 10;
const uint8_t homeZ = 12;

const uint8_t DeadManSwitch = 19;

#define xMAX 1955
#define yMAX 1595
#define zMAX 746
#define w1MIN -48
#define w2MIN -48
#define w3MIN -48
#define w1MAX 48
#define w2MAX 48
#define w3MAX 48

#define xMaxLength 280
#define yMaxLength 228
#define zMaxLength 90

#define w1MinAngle -180
#define w1MaxAngle 180
#define w2MinAngle -180
#define w2MaxAngle 180
#define w3MinAngle -180
#define w3MaxAngle 180

volatile int xCount = xMAX / 2;
volatile int yCount = yMAX / 2;
volatile int zCount = zMAX / 2;
volatile int w1Count = 0;
volatile int w2Count = 0;
volatile int w3Count = 0;
bool change = 0;

#define MAP(x, a, b, c, d) (c + (float(d) - c) * (x - a) / (float(b) - a))
#define CONSTRAIN(x, a, b) (x <= a ? a : (x >= b ? b : x))
#define EASYMAP(x,a,b) (float(b)*x/float(a))

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

const uint8_t HEART = 13;
uint8_t sendAgain = 0;
int16_t Positions[6] = {0, 0, 0, 0, 0, 0};
int16_t prevPositions[6] = {0, 0, 0, 0, 0, 0};

void setup() {
  pinMode(xA, INPUT_PULLUP); pinMode(xB, INPUT_PULLUP);
  pinMode(yA, INPUT_PULLUP); pinMode(yB, INPUT_PULLUP);
  pinMode(zA, INPUT_PULLUP); pinMode(zB, INPUT_PULLUP);
  pinMode(w1A, INPUT_PULLUP); pinMode(w1B, INPUT_PULLUP);
  pinMode(w2A, INPUT_PULLUP); pinMode(w2B, INPUT_PULLUP);
  // pinMode(w3A, INPUT_PULLUP); pinMode(w3B, INPUT_PULLUP);

  pinMode(homeX, INPUT_PULLUP);
  pinMode(homeY, INPUT_PULLUP);
  pinMode(homeZ, INPUT_PULLUP);

  pinMode(DeadManSwitch, INPUT_PULLUP);

  attachInterrupt(xA, tickXA, RISING);
  attachInterrupt(yA, tickYA, RISING);
  attachInterrupt(zA, tickZA, RISING);
  attachInterrupt(w1A, tickW1A, CHANGE); attachInterrupt(w1B, tickW1B, CHANGE);
  attachInterrupt(w2A, tickW2A, CHANGE); attachInterrupt(w2B, tickW2B, CHANGE);
  //attachInterrupt(w2A,tickW3A,CHANGE); attachInterrupt(w2B,tickW3B,CHANGE);

  attachInterrupt(homeX, Xhomed, FALLING);
  attachInterrupt(homeY, Yhomed, FALLING);
  attachInterrupt(homeZ, Zhomed, FALLING);

  pinMode(HEART, OUTPUT);

  digitalWrite(HEART, HIGH);
  Serial.begin(115200);
  while (!Serial);
  EstablishConnection();
  digitalWrite(HEART, LOW);
}

void loop() {
  //  while (1) {
  //    getPositions();
  //    if (change) {
  //      for (uint8_t i = 0; i < 6; i++) {
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
    else if (sendAgain == 8)
      homeAll();
    flushData();
  }

  if (sendAgain) { //&& digitalRead(DeadManSwitch)
    getPositions();
    sendPositions();
    sendAgain = 0;
  }
}

void flushData() {
  while (Serial.available()) {
    byte trash = Serial.read();
  }
}

void getPositions() {
  Positions[0] = EASYMAP(xCount, xMAX, xMaxLength);
  Positions[1] = EASYMAP(yCount, yMAX, yMaxLength);
  Positions[2] = EASYMAP(zCount, zMAX, zMaxLength);
  Positions[3] = MAP(w1Count, w1MIN, w1MAX , w1MinAngle, w1MaxAngle);
  Positions[4] = MAP(w2Count, w2MIN, w2MAX, w2MinAngle, w2MaxAngle);
  Positions[5] = MAP(w3Count, w3MIN, w3MAX, w3MinAngle, w3MaxAngle);

  change = 0;
  for (uint8_t i = 0; i < 6; i++) {
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
  uint8_t A = digitalRead(xA); uint8_t B = digitalRead(xB);
  A != B ? --xCount : ++xCount;
}
/*void tickXB(){
  uint8_t A = digitalRead(xA); uint8_t B = digitalRead(xB);
  B != A ? --xCount : ++xCount;
  }*/

// Y Encoder
void tickYA() {
  uint8_t A = digitalRead(yA); uint8_t B = digitalRead(yB);
  A != B ? ++yCount : --yCount;
}
/*void tickYB(){
  uint8_t A = digitalRead(yA); uint8_t B = digitalRead(yB);
  B != A ? --yCount : ++yCount;
  }*/

// Z Encoder
void tickZA() {
  uint8_t A = digitalRead(zA); uint8_t B = digitalRead(zB);
  A != B ? ++zCount : --zCount;
}
/*void tickZB(){
  uint8_t A = digitalRead(zA); uint8_t B = digitalRead(zB);
  B != A ? --zCount : ++zCount;
  }*/

// W1 Encoder
void tickW1A() {
  uint8_t A = digitalRead(w1A); uint8_t B = digitalRead(w1B);
  A != B ? ++w1Count : --w1Count;
}
void tickW1B() {
  uint8_t A = digitalRead(w1A); uint8_t B = digitalRead(w1B);
  B != A ? --w1Count : ++w1Count;
}

// W2 Encoder
void tickW2A() {
  uint8_t A = digitalRead(w2A); uint8_t B = digitalRead(w2B);
  A != B ? ++w2Count : --w2Count;
}
void tickW2B() {
  uint8_t A = digitalRead(w2A); uint8_t B = digitalRead(w2B);
  B != A ? --w2Count : ++w2Count;
}

// W3 Encoder
//void tickW3A() {
//  uint8_t A = digitalRead(w3A); uint8_t B = digitalRead(w3B);
//  A != B ? ++w3Count : --w3Count;
//}
//void tickW3B() {
//  uint8_t A = digitalRead(w3A); uint8_t B = digitalRead(w3B);
//  B != A ? --w3Count : ++w3Count;
//}

void Xhomed() {
  xCount = 0;
}
void Yhomed() {
  yCount = 0;
}
void Zhomed() {
  zCount = zMAX;
}
void W1homed() {
  w1Count = 0;
}
void W2homed() {
  w2Count = 0;
}
void W3homed() {
  w3Count = 0;
}

void homeAll() {
//  Xhomed();
//  Yhomed();
//  Zhomed();
  W1homed();
  W2homed();
  W3homed();
}
