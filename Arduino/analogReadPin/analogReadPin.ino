int minRead = 206;
int midRead = 445;
int maxRead = 584;
int mindeg = -180;
int maxdeg = 180;

int deg;
int readVal;
void setup() {
Serial.begin(9600);
analogReadResolution(12);
}

void loop() {
  delay(100);

Serial.print(analogRead(A7));Serial.print('\t');
Serial.print(analogRead(A8));Serial.print('\t');
Serial.println(analogRead(A9));

  

}
