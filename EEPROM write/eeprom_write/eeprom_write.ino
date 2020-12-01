#include <EEPROM.h>
int ID=1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(500000);
  EEPROM.write(0,ID);
  Serial.println("Write done");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Test :");
  Serial.println(EEPROM.read(0));
  delay(1000000);

}
