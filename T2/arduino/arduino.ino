const int pinFSR = A0;

void setup() {
  Serial.begin(9600);
  //pinMode(pinFSR, INPUT_PULLUP);  // usa la resistencia interna a 5 V
}

void loop() {
  int raw = analogRead(pinFSR);   // 0..1023
  // Nota: la lectura será ALTA sin presión y BAJA al presionar.
  Serial.println(raw);
  delay(50);
}
