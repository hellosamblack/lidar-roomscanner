#include "STTS22HSensor.h"

// Components
STTS22HSensor Temp(&Wire);

void setup() {
  // Initialize serial for output
  Serial.begin(9600);
  // Initlialize Wire instance
  Wire.begin();
  // Initlialize components
  Temp.begin();
  Temp.Enable();
}

void loop() {
  float temperature = 0.0;
  // Read temperature
  Temp.GetTemperature(&temperature);
  // Output data
  Serial.print("Temperature: ");
  Serial.println(temperature);
  delay(1000);
}