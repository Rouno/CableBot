/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>
#define max(a,b) ((a)>(b)?(a):(b))

#if defined(__OPENCM904__)
#define DEVICE_NAME "1" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
#define DEVICE_NAME ""
#endif

#define BAUDRATE  1000000
#define min(a,b) ((a)<(b)?(a):(b))
#define TIMEOUT 1000
#define POSITION_MODE 3
#define SPEED_MODE 1
#define PWM_MODE 16
#define COMPLIANT_MODE 15

#define DXL_COUNT 2
int8_t dynIdx[DXL_COUNT] = {1, 2};

int32_t pos_offset[DXL_COUNT];
int32_t presentPosition[DXL_COUNT];
int32_t presentSpeed[DXL_COUNT];
int32_t presentLoad[DXL_COUNT];
int32_t presentPWM[DXL_COUNT];
int32_t operatingMode[DXL_COUNT];

int32_t goalPosition[DXL_COUNT];
int32_t goalSpeed[DXL_COUNT];
int32_t goalPWM[DXL_COUNT];
int32_t d_load[DXL_COUNT];
int32_t d_speed[DXL_COUNT];

int32_t received_frame[DXL_COUNT];

int32_t loadLimit = 80;
int32_t targetPWM = 150;
int32_t speedThreshold = 10;

bool result = false;

float LoadtoPWM = 1 / 1.12;

DynamixelWorkbench dxl_wb;

void setup()
{
  Serial.begin(57600);
  while (!Serial); // If this line is activated, you need to open Serial Terminal.

  const char *log;
  dxl_wb.begin(DEVICE_NAME, BAUDRATE);
  for (int i = 0; i < DXL_COUNT; i++)
  {
    int32_t op_mode = PWM_MODE;

    presentPosition[i] = 0;
    presentSpeed[i] = 0;
    presentLoad[i] = 0;
    presentPWM[i] = 0;
    operatingMode[i] = op_mode;
    goalPosition[i] = 0;
    goalSpeed[i] = 0;
    goalPWM[i] = 0;
    d_load[i] = 0;
    d_speed[i] = 0;
    received_frame[i] = 0;

    result = dxl_wb.ping(dynIdx[i]);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to ping " + String(dynIdx[i]));
    }

    result = dxl_wb.itemWrite(dynIdx[i], "Torque_Enable", 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to disable torque " + String(dynIdx[i]));
    }

    result = dxl_wb.itemWrite(dynIdx[i], "Operating_Mode", op_mode, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to change Operating Mode " + String(op_mode) + " " + String(dynIdx[i]));
    }
  }
}

void loop()
{
  /*
     Reading values from actuator
  */
  const char *log;

  for (int i = 0; i < DXL_COUNT; i++)
  {
    int32_t read_item;
    result = dxl_wb.itemRead(dynIdx[i], "Present_Position", &read_item, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to read present position");
    } else {
      presentPosition[i] = read_item;
      Serial.print(String(presentPosition[i]) + " ");
    }

    result = dxl_wb.itemRead(dynIdx[i], "Present_Velocity", &read_item, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to read present speed");
    } else {
      presentSpeed[i] = read_item;
      Serial.print(String(presentSpeed[i]) + " ");
    }

    result = dxl_wb.itemRead(dynIdx[i], "Present_Load", &read_item, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to read present load");
    } else {
      presentLoad[i] = read_item;
      if (presentLoad[i] > 32768) presentLoad[i] -= 65536;
      Serial.print(String(presentLoad[i]) + " ");
    }

    result = dxl_wb.itemRead(dynIdx[i], "Present_PWM", &read_item, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to read present PWM");
    } else {
      presentPWM[i] = read_item;
      if (presentPWM[i] > 32768) presentPWM[i] -= 65536;
      Serial.print(String(presentPWM[i]) + " ");
    }
  }
  Serial.println();

  /*
     Read command from Serial store in received_frame buffer
  */
  String actuatorItem = "";
  uint8_t frameIndex = 0;
  boolean fullframe_received = false;
  boolean time_out = false;
  if (SerialUSB.available()) {
    digitalWrite(BOARD_LED_PIN, HIGH);
    do {
      char value = SerialUSB.read();
      int id;
      switch (value) {
        case 'K':
          time_out = true;
          break;
        case 'P': //switch to position operating mode
          fullframe_received = true;
          value = SerialUSB.read(); //actuator id is stored just after 'P' command
          actuatorItem += value;
          id = actuatorItem.toInt();
          dxl_wb.itemWrite(id, "Torque_Enable", 0, &log);
          result = dxl_wb.setPositionControlMode(id, &log);
          operatingMode[actuatorIndexToId(id)] = POSITION_MODE;
          dxl_wb.itemWrite(id, "Torque_Enable", 1, &log);
          actuatorItem = "";
          break;
        case 'S': //switch to speed operating mode
          fullframe_received = true;
          value = SerialUSB.read(); //actuator id is stored just after 'S' command
          actuatorItem += value;
          id = actuatorItem.toInt();
          dxl_wb.itemWrite(id, "Torque_Enable", 0, &log);
          result = dxl_wb.setVelocityControlMode(id, &log);
          operatingMode[actuatorIndexToId(id)] = SPEED_MODE;
          dxl_wb.itemWrite(id, "Torque_Enable", 1, &log);
          actuatorItem = "";
          break;
        case 'W': //switch to PWM operating mode
          fullframe_received = true;
          value = SerialUSB.read(); //actuator id is stored just after 'W' command
          actuatorItem += value;
          id = actuatorItem.toInt();
          dxl_wb.itemWrite(id, "Torque_Enable", 0, &log);
          result = dxl_wb.setPWMControlMode(id, &log);
          operatingMode[actuatorIndexToId(id)] = PWM_MODE;
          dxl_wb.itemWrite(id, "Torque_Enable", 1, &log);
          actuatorItem = "";
          break;
        case 'C': // actuator compliance
          fullframe_received = true;
          value = SerialUSB.read(); //actuator id is stored just after 'C' command
          actuatorItem += value;
          id = actuatorItem.toInt();
          dxl_wb.itemWrite(id, "Torque_Enable", 0, &log);
          result = dxl_wb.setPWMControlMode(id, &log);
          operatingMode[actuatorIndexToId(id)] = COMPLIANT_MODE;
          dxl_wb.itemWrite(id, "Torque_Enable", 1, &log);
          actuatorItem = "";
          break;
        case 'R' : //software reboot of an actuator
          fullframe_received = true;
          value = SerialUSB.read(); //actuator id is stored just after 'C' command
          actuatorItem += value;
          id = actuatorItem.toInt();
          Serial.println("rebooting actuator " + String(id));
          dxl_wb.reboot(id);
          break;
        case '\n':
          fullframe_received = true;
          received_frame[frameIndex] = actuatorItem.toInt();
          break;
        case ' ':
          received_frame[frameIndex] = actuatorItem.toInt();
          actuatorItem = "";
          frameIndex += 1;
          break;
        default:
          actuatorItem += value;
          break;
      }
    } while (!fullframe_received && !time_out);
  }
  digitalWrite(BOARD_LED_PIN, LOW);


  /*
     send commands to actuators
  */

  for (int i = 0; i < DXL_COUNT; i++) {
    switch (operatingMode[i]) {
      case POSITION_MODE :
        changePosition(dynIdx[i], received_frame[i]);
        //Serial.println("actuator " + String(dynIdx[i]) + " position set to : " + String(received_frame[i]));
        break;
      case SPEED_MODE :
        changeSpeed(dynIdx[i], received_frame[i]);
        //Serial.println("actuator " + String(dynIdx[i]) + " speed set to : " + String(received_frame[i]));
        break;
      case PWM_MODE :
        changePWM(dynIdx[i], received_frame[i]);
        //Serial.println("actuator " + String(dynIdx[i]) + " PWM set to : " + String(received_frame[i]));
        break;
      case COMPLIANT_MODE :
        //Serial.println("actuator " + String(dynIdx[i]) + " PWM set to compliance ");
        changeCompliance(i);
        changePWM(dynIdx[i], goalPWM[i]);
        break;
      default :
        Serial.println("operating mode value of actuator " + String(dynIdx[i]) + " is unexpected : " + String(operatingMode[i]));
        break;
    }

  }

}

int actuatorIndexToId(int8_t id) {
  int result = -1;
  for (int i = 0; i < DXL_COUNT; i++) {
    if (dynIdx[i] == id) result = i;
  }
  if (result == -1) Serial.println("Warning: actuator id " + String(id) + " is not initialized");
  return result;
}

void changeOperatingMode(int8_t id, int8_t mode) {
  bool result;
  const char *log;
  result = dxl_wb.itemWrite(id, "Operating_Mode", mode, &log);
  operatingMode[actuatorIndexToId(id)] = mode;
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to change Operating Mode " + String(mode) + " on actuator" + String(id));
  }
}

void changePWM(int8_t id, int32_t pwm) {
  bool result;
  const char *log;
  result = dxl_wb.itemWrite(id, "Goal_PWM", pwm, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to set PWM " + String(pwm) + " on actuator " + String(id));
  }
}

void changePosition(int8_t id, int32_t pos) {
  bool result;
  const char *log;
  result = dxl_wb.itemWrite(id, "Goal_Position", pos, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to set Position " + String(pos) + " on actuator " + String(id));
  }
}

void changeSpeed(int8_t id, int32_t spd) {
  bool result;
  const char *log;
  result = dxl_wb.itemWrite(id, "Goal_Velocity", spd, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to set Speed " + String(spd) + " on actuator " + String(id));
  }
}

void changeCompliance(int i) {
  d_load[i] -= presentLoad[i];
  d_speed[i] -= presentSpeed[i];
  if (presentLoad[i] <=  - loadLimit) {
    goalPWM[i] = targetPWM;
  } else {
    if (d_load[i] < 0 && presentLoad[i] > 0) {
      goalPWM[i] = 0;
    }
    if (presentLoad[i] > loadLimit) {
      goalPWM[i] = -targetPWM;
    }
  }
  d_load[i] = presentLoad[i];
  d_speed[i] = presentSpeed[i];
}
