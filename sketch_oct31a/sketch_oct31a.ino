int pos;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void  loop() {
  while (!Serial.available());
  pos = Serial.readString().toInt();
  Serial.print(pos + 1);
}
