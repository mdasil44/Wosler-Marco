#define MAP(x,a,b,c,d) ((d-c)*(float(x)-a)/(b-a) + c)

// Wrist1
#define W1_MIN_READ 823
#define W1_MAX_READ 2340
#define W1_MIN_POS -85.86
#define W1_MAX_POS 47.46

//Wrist2
#define W2_MIN_READ 810
#define W2_MAX_READ 3347
#define W2_MIN_POS 111.44
#define W2_MAX_POS -111.44

//Wrist3
#define W3_MIN_READ 0
#define W3_MAX_READ 4096
#define W3_MIN_POS 180.00
#define W3_MAX_POS -180.00

uint8_t wPins[3] = {A7,A8,A9};
float w[3] = {0,0,0};
int16_t wReadMins[3] = {W1_MIN_READ,W2_MIN_READ,W3_MIN_READ};
int16_t wReadMaxs[3] = {W1_MAX_READ,W2_MAX_READ,W3_MAX_READ};
float wPosMins[3] = {W1_MIN_POS,W2_MIN_POS,W3_MIN_POS};
float wPosMaxs[3] = {W1_MAX_POS,W2_MAX_POS,W3_MAX_POS};

void setup() {
Serial.begin(9600);
analogReadResolution(12);
}

void loop() {
delay(10);
for (uint8_t i = 0; i < 3; i++){
int readVal = analogRead(wPins[i]);
w[i] = MAP(readVal,wReadMins[i],wReadMaxs[i],wPosMins[i],wPosMaxs[i]);
Serial.print(w[i]);Serial.print('\t');
}
Serial.println();
}
