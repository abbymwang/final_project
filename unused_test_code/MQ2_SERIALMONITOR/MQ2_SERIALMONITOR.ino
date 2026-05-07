#include<MQ2.h>

#define PIN A0

MQ2 mq2(PIN);

void setup() {
  Serial.begin(9600);
  mq2.begin();

}

void loop() {
  int lpg = mq2.readLPG();
  int co = mq2.readCO();
  int smo = mq2.readSmoke();
  Serial.print("Smoke: ");
  Serial.print(smo);
  Serial.print("  LPG: ");
  Serial.print(lpg);
  Serial.print("  CO2: ");
  Serial.println(co);
  delay(2000);
}
