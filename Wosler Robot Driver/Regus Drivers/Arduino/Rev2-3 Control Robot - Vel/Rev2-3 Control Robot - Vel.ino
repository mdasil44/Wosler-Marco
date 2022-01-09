//#include <RunningMedian.h>
//RunningMedian samples = RunningMedian(5);

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
#define X_MIN_POS 0
#define X_MAX_POS 144

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
#define W1_MIN_READ 1265
#define W1_MAX_READ 2792
#define W1_MIN_POS -85.86 // negatives swapped
#define W1_MAX_POS 47.46

//Wrist2
#define W2_PIN A8
#define W2_MIN_READ 810
#define W2_MAX_READ 3350
#define W2_MIN_POS -21.44 // added 90deg (was -111 to 111)
#define W2_MAX_POS 201.44

//Wrist3
#define W3_PIN A9
#define W3_MIN_READ 0
#define W3_MAX_READ 4096
#define W3_MIN_POS -180.00
#define W3_MAX_POS 180.00

const uint8_t axisPins[2][6] = {{XA_PIN, YA_PIN, ZA_PIN, W1_PIN, W2_PIN, W3_PIN}, {XB_PIN, YB_PIN, ZB_PIN, 0, 0, 0}};
const int16_t readMins[6] = {X_MIN_READ, Y_MIN_READ, Z_MIN_READ, W1_MIN_READ, W2_MIN_READ, W3_MIN_READ};
const int16_t readMaxs[6] = {X_MAX_READ, Y_MAX_READ, Z_MAX_READ, W1_MAX_READ, W2_MAX_READ, W3_MAX_READ};
float posMins[6] = {X_MIN_POS, Y_MIN_POS, Z_MIN_POS, W1_MIN_POS, W2_MIN_POS, W3_MIN_POS};
float posMaxs[6] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS, W1_MAX_POS, W2_MAX_POS, W3_MAX_POS};
const float range[6] = {X_MAX_POS - X_MIN_POS, Y_MAX_POS - Y_MIN_POS, Z_MAX_POS - Z_MIN_POS, W1_MAX_POS - W1_MIN_POS, W2_MAX_POS - W2_MIN_POS, W3_MAX_POS - W3_MIN_POS};
const float absoluteMins[3] = {0,-500.0,-500.0};
const float absoluteMaxs[3] = {600.0,500.0,500.0};
const uint8_t homePins[3] = {X_HOME_PIN, Y_HOME_PIN, Z_HOME_PIN};
const int16_t homes[3] = {X_HOME_READ, Y_HOME_READ, Z_HOME_READ};

const uint8_t HEART = 13;
uint8_t sendAgain = 0;
volatile int16_t count[3] = {(X_MIN_READ + X_MAX_READ) / 2, (Y_MIN_READ + Y_MAX_READ) / 2, (Z_MIN_READ + Z_MAX_READ) / 2};
float Positions[6] = {(X_MIN_POS + X_MAX_POS) / 2, (Y_MIN_POS + Y_MAX_POS) / 2, (Z_MIN_POS + Z_MAX_POS) / 2, 0, 0, 0};
float prevPositions[6] = {(X_MIN_POS + X_MAX_POS) / 2, (Y_MIN_POS + Y_MAX_POS) / 2, (Z_MIN_POS + Z_MAX_POS) / 2, 0, 0, 0};
double Velocities[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
uint8_t homed[3] = {0, 0, 0};

double RawVelocityRecord[6][5] = {{0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}};
double *ElementPtr = &RawVelocityRecord[0][0];
uint8_t counter = 0;

unsigned long CurrentMillis = 0;
unsigned long LastMillis = 0;
unsigned long CurrentMicros = 0;
unsigned long LastMicros = 0;
const uint8_t FixedWaitTime = 50;
const float LPfilterConst = 1;

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

  delay(1000);
  
  Serial.println("Home Input Device");
  uint8_t allHomed = 0;
  CurrentMillis = millis();
  LastMillis = CurrentMillis;
  while (1) {
    CurrentMillis = millis();
    checkHomes();
    allHomed = homed[0] * homed[1] * homed[2];
    if (allHomed) {
      uint16_t deltaT = CurrentMillis - LastMillis;
      if (deltaT <= 1000)
        Serial.printf("Hold for %1.2f\n", float(1 - deltaT / 1000.0));
      else
        break;
    }
    else{
      LastMillis = CurrentMillis;
      Serial.printf("X: %i, Y: %i, Z: %i, A: %i\n", homed[0], homed[1], homed[2], allHomed);
    }
    delay(50);
  }
  Serial.println(F("READY"));
  //Motion();
  //printPositions(); // get initial position

  delay(1000);

  CurrentMillis = millis();
  LastMillis = CurrentMillis;
}

void loop() {
  CurrentMillis = millis();

  if (CurrentMillis - LastMillis >= FixedWaitTime)
  {
    Motion();
    //printPositions();
    printVelocities();
    LastMillis = CurrentMillis;
  }

  if (Serial.available() > 0) {
    sendAgain = Serial.read();
    if (sendAgain == '1')
      printPositions();
    if (sendAgain == 'R')
      WRITE_RESTART(0x5FA0004);
    flushData();
  }
}

void flushData() {
  while (Serial.available()) {
    Serial.read();
  }
}

void Motion() {
  checkHomes();
  for (uint8_t i = 0; i < 6; i++) {
    float pos = 0.0;
    double vel = 0.0;
    double prevVel = Velocities[i];

    if (i < 3) {
      pos = TraverseWorkspace(i, pos);
    }
    else
      pos = MAP(analogRead(axisPins[0][i]), readMins[i], readMaxs[i], posMins[i], posMaxs[i]);

    Positions[i] = pos;
    vel = 1000.0d * double(Positions[i] - prevPositions[i]) / FixedWaitTime; // mm/us -> mm/s * 10^6

    RawVelocityRecord[i][counter] = vel;
    ElementPtr = &RawVelocityRecord[i][0];
    Velocities[i] = MedianFilter();
    Velocities[i] = Velocities[i] * LPfilterConst + prevVel * (1 - LPfilterConst);

    prevPositions[i] = Positions[i];
  }
  counter = (counter + 1) % 5;
}

void printVelocities() {
  Serial.print(F("v"));
  for (uint8_t i = 0; i < 6; i++) {
    Serial.printf("%3.2f%s", Velocities[i], i == 5 ? "\n" : ",");
  }
}

void printPositions() {
  Serial.print(F("p"));
  for (uint8_t i = 0; i < 6; i++) {
    Serial.printf("%3.2f%s", Positions[i], i == 5 ? "\n" : ",");
  }
}

void EstablishConnection() {
  while (1) {
    if (Serial.available() > 0) {
      if (Serial.read() == 'H') {
        flushData();
        Serial.println('S');
        return;
      }
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
    if (!digitalRead(homePins[i])) {
      count[i] = homes[i];
      homed[i] = 1;
    }
    else
      homed[i] = 0;
  }
}

float TraverseWorkspace(uint8_t i, float pos) {

  uint8_t margin = 2;
  pos = CONSTRAIN(count[i], readMins[i], readMaxs[i]);
  if (pos < readMins[i] + margin) {
    posMins[i] = max(posMins[i] - margin, absoluteMins[i]);
    posMaxs[i] = posMins[i] + range[i];
  }
  else if (pos > readMaxs[i] - margin) {
    posMaxs[i] = min(posMaxs[i] + margin, absoluteMaxs[i]);
    posMins[i] = posMaxs[i] - range[i];

  }

  pos = MAP(pos, readMins[i], readMaxs[i], posMins[i], posMaxs[i]);

  return pos;
}






///////////


double MedianFilter() {
  double newArray[5];
  for (uint8_t i = 0; i < 5; i++) {
    newArray[i] = *(ElementPtr + i);
  }
  mergeSort(newArray, 0, 4);
  return newArray[2];
}

void merge(double myarray[], uint8_t const left, uint8_t const mid, uint8_t const right)
{
  uint8_t const subArrayOne = mid - left + 1;
  uint8_t const subArrayTwo = right - mid;

  // Create temp arrays
  double leftArray[subArrayOne], rightArray[subArrayTwo];

  // Copy data to temp arrays leftArray[] and rightArray[]
  for (uint8_t i = 0; i < subArrayOne; i++)
    leftArray[i] = myarray[left + i];
  for (uint8_t j = 0; j < subArrayTwo; j++)
    rightArray[j] = myarray[mid + 1 + j];

  uint8_t indexOfSubArrayOne = 0, // Initial index of first sub-array
          indexOfSubArrayTwo = 0; // Initial index of second sub-array
  uint8_t indexOfMergedArray = left; // Initial index of merged array

  // Merge the temp arrays back into array[left..right]
  while (indexOfSubArrayOne < subArrayOne && indexOfSubArrayTwo < subArrayTwo) {
    if (leftArray[indexOfSubArrayOne] <= rightArray[indexOfSubArrayTwo]) {
      myarray[indexOfMergedArray] = leftArray[indexOfSubArrayOne];
      indexOfSubArrayOne++;
    }
    else {
      myarray[indexOfMergedArray] = rightArray[indexOfSubArrayTwo];
      indexOfSubArrayTwo++;
    }
    indexOfMergedArray++;
  }
  // Copy the remaining elements of
  // left[], if there are any
  while (indexOfSubArrayOne < subArrayOne) {
    myarray[indexOfMergedArray] = leftArray[indexOfSubArrayOne];
    indexOfSubArrayOne++;
    indexOfMergedArray++;
  }
  // Copy the remaining elements of
  // right[], if there are any
  while (indexOfSubArrayTwo < subArrayTwo) {
    myarray[indexOfMergedArray] = rightArray[indexOfSubArrayTwo];
    indexOfSubArrayTwo++;
    indexOfMergedArray++;
  }
}

// begin is for left index and end is
// right index of the sub-array
// of arr to be sorted */
void mergeSort(double myarray[], uint8_t const firstEl, uint8_t const lastEl)
{
  if (firstEl >= lastEl)
    return; // Returns recursively

  uint8_t mid = firstEl + (lastEl - firstEl) / 2;
  mergeSort(myarray, firstEl, mid);
  mergeSort(myarray, mid + 1, lastEl);
  merge(myarray, firstEl, mid, lastEl);
}
