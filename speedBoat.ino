#include "variable.h"
#include "StorageController.h"
#include "IMUReader.h"
#include "MotionController.h"
#include "UDPConnector.h"
#include "JoystickController.h"
#include "System.h"
#include "RCController.h"
#include "WEBDebugger.h"

void setup() {
  motor_initial();
  interface_initial();
  system_initial();
  storage_initial();
  //  imu_initial();

  if (isRCRemote) {
    rc_control_initial();
  }
  else {
    joystick_initial();
  }

  set_param_heading(25.00 , 0.0025 , 0.00 , 3500);
}

float baseSpeed = 0;

void mainControl() {
  static bool onetime[2];

  if (isRCRemote && readConnectionRC()) {
    if (readButtonRC(PressButton) && readButtonRC(ToggleButton)) {
      if (onetime[0] == false) {
        if (modeSelector < 3) {
          modeSelector++;
        }
        else {
          modeSelector = 0;
        }
        onetime[0] = true;
      }
    }
    else {
      onetime[0] = false;
    }
    if (modeSelector == 0) {
      motorDriverRun((int16_t) (-(readAnalogRC(Accelerator) + readAnalogRC(CircleWheels)) * 2.046)  , 1 , (int16_t)(-(readAnalogRC(Accelerator) - readAnalogRC(CircleWheels)) * 2.046) , 1);
    }
    else if (modeSelector == 1) {

    }
  }
  else {
    joystick_run();
    if (joystickConnected) {


    }
    else {
      motorDriverA(0);
      motorDriverB(0);
    }

  }

  if (isConnected) {
    if (initialDebug == false) {
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      webSerialInitial();
      initialDebug = true;
    }
    OTA_RUNTIME();
  }
  //  Serial.printf("i'm running hahaha \n");

}

void loop() {
  mainControl();
}
