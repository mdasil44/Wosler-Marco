uint8_t lower_buffer[32];
uint8_t upper_buffer[32] DMAMEM;
void setup() {
  uint8_t stack_buffer[32];
  uint8_t *heap_buffer = malloc(32);

  while (!Serial && millis() < 4000) ;
  Serial.begin(115200);
  delay(500);
  Serial.printf("Lower_Buffer: %x\n", (uint32_t)lower_buffer);
  Serial.printf("upper_buffer: %x\n", (uint32_t)upper_buffer);
  Serial.printf("stack buffer: %x\n", (uint32_t)stack_buffer);
  Serial.printf("Heap Buffer: %x\n", (uint32_t)heap_buffer);
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, !digitalRead(13));
  delay(500);
}
