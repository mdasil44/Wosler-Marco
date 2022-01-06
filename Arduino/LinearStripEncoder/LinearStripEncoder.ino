const uint8_t xA = 2; const uint8_t xB = 3;
const uint8_t yA = 0; const uint8_t yB = 1;
const uint8_t zA = 4; const uint8_t zB = 5;

const uint8_t homeX = 7;
const uint8_t homeY = 6;
const uint8_t homeZ = 8;

#define xMAX 1059
#define yMAX 975
#define zMAX 955

#define xMaxLength 280
#define yMaxLength 228
#define zMaxLength 90

volatile int xCount = xMAX / 2;
volatile int yCount = yMAX / 2;
volatile int zCount = zMAX / 2;
bool change = 0;

#define MAP(x, a, b, c, d) (c + (float(d) - c) * (x - a) / (float(b) - a))
#define CONSTRAIN(x, a, b) (x <= a ? a : (x >= b ? b : x))
#define EASYMAP(x,a,b) (float(b)*x/float(a))

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

const uint8_t HEART = 13;
uint8_t sendAgain = 0;
int16_t Positions[3] = {0, 0, 0};
int16_t prevPositions[3] = {0, 0, 0};

void setup() {
  pinMode(xA, INPUT_PULLUP); pinMode(xB, INPUT_PULLUP);
  pinMode(yA, INPUT_PULLUP); pinMode(yB, INPUT_PULLUP);
  pinMode(zA, INPUT_PULLUP); pinMode(zB, INPUT_PULLUP);

  pinMode(homeX, INPUT_PULLUP);
  pinMode(homeY, INPUT_PULLUP);
  pinMode(homeZ, INPUT_PULLUP);

  attachInterrupt(xA, tickXA, RISING);
  attachInterrupt(yA, tickYA, RISING);
  attachInterrupt(zA, tickZA, RISING);

  pinMode(HEART, OUTPUT);

  digitalWrite(HEART, HIGH);
  Serial.begin(115200);
  while (!Serial);
  //EstablishConnection();
  digitalWrite(HEART, LOW);
}

void loop() {
  while (1) {
    getPositions();
    if (change) {
      for (uint8_t i = 0; i < 3; i++) {
        Serial.print(Positions[i]); Serial.print(F("\t"));
      }
      Serial.println();
    }
    delay(100);
  }

  digitalWrite(HEART, sendAgain);

  if (Serial.available() > 0) {
    sendAgain = Serial.read();
    if (sendAgain == 5)
      WRITE_RESTART(0x5FA0004);
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
  checkHomes();
  Positions[0] = xCount; //EASYMAP(xCount, xMAX, xMaxLength);
  Positions[1] = yCount; //EASYMAP(yCount, yMAX, yMaxLength);
  Positions[2] = zCount; //EASYMAP(zCount, zMAX, zMaxLength);

  change = 0;
  for (uint8_t i = 0; i < 3; i++) {
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

// X Encoder
void tickXA() {
  uint8_t A = digitalRead(xA); uint8_t B = digitalRead(xB);
  A != B ? ++xCount : --xCount;
}
/*void tickXB(){
  uint8_t A = digitalRead(xA); uint8_t B = digitalRead(xB);
  B != A ? --xCount : ++xCount;
  }*/

// Y Encoder
void tickYA() {
  uint8_t A = digitalRead(yA); uint8_t B = digitalRead(yB);
  A != B ? --yCount : ++yCount;
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

void checkHomes() {
  if (!digitalRead(homeX))
    xCount = xMAX;

  if (!digitalRead(homeY))
    yCount = yMAX;

  if (!digitalRead(homeZ))
    zCount = zMAX;
}
