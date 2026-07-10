/*
   @file    LSM6DSO16IS_ISPU_Sensor_Fusion.ino
   @author  STMicroelectronics
   @brief   Example to use the LSM6DSO16IS ISPU Sensor Fusion of the accelerometer and gyroscope, configured in high-performance mode at 104 Hz.
 *******************************************************************************
   Copyright (c) 2023, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/

/*
 * You can display the quaternion values with a 3D model connecting for example to this link:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 */

// Includes
#include "LSM6DSO16ISSensor.h"
#include "sensor_fusion.h"

#define INT_1 A5

//Interrupts.
volatile int mems_event = 0;

LSM6DSO16ISSensor sensor(&Wire);
ucf_line_ext_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;
void INT1Event_cb();

union data {
  uint8_t raw_data[16];
  float_t values[4];
};

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  Serial.begin(115200);

  // Initlialize i2c.
  Wire.begin();

  // Initlialize components.
  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();

  // Feed the program to ISPU
  ProgramPointer = (ucf_line_ext_t *)ispu_conf;
  TotalNumberOfLine = sizeof(ispu_conf) / sizeof(ucf_line_ext_t);
  Serial.println("LSM6DSO16IS ISPU Sensor Fusion");
  Serial.print("UCF Number Line=");
  Serial.println(TotalNumberOfLine);

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
    if (ProgramPointer[LineCounter].op == MEMS_UCF_OP_WRITE) {
      if (sensor.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
        Serial.print("Error loading the Program to LSM6DSO16ISSensor at line: ");
        Serial.println(LineCounter);
        while (1) {
          // Led blinking.
          digitalWrite(LED_BUILTIN, HIGH);
          delay(250);
          digitalWrite(LED_BUILTIN, LOW);
          delay(250);
        }
      }
    } else if (ProgramPointer[LineCounter].op == MEMS_UCF_OP_DELAY) {
      delay(ProgramPointer[LineCounter].data);
    }
  }
  Serial.println("Program loaded inside the LSM6DSO16IS ISPU");
  //Interrupts.
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);
}

void loop()
{
  union data quaternions;
  // When the quaternion for the new sample is computed and available in the output registers an interrupt is generated.
  if (mems_event) {
    LSM6DSO16IS_ISPU_Status_t ispu_status;
    mems_event = 0;
    sensor.Get_ISPU_Status(&ispu_status);
    // Check if the ISPU event is from the algo00.
    if (ispu_status.ia_ispu_0) {
      // Read quaternions and print them.
      sensor.Read_ISPU_Output(LSM6DSO16IS_ISPU_DOUT_00_L, &quaternions.raw_data[0], 16);
      Serial.print("Quaternion: ");
      Serial.print(quaternions.values[3], 4);
      Serial.print(", ");
      Serial.print(-quaternions.values[1], 4);
      Serial.print(", ");
      Serial.print(quaternions.values[0], 4);
      Serial.print(", ");
      Serial.println(quaternions.values[2], 4);
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
