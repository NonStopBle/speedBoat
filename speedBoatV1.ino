#include "variable.h"
#include "StorageController.h"
#include "IMUReader.h"
#include "MotionController.h"
#include "UDPConnector.h"
#include "JoystickController.h"
#include "System.h"
#include "RCController.h"

void setup() {
  motor_initial();
  interface_initial();
  system_initial();
  storage_initial();
  imu_initial();

  if (isRCRemote) {
    rc_control_initial();
  }
  else {
    joystick_initial();
  }
  set_param_heading(30.00 , 0.0025 , 0.00 , 3500);
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
      if (modeSelector == 0) {
        if (joyAxis(ThrottleR , 50) > 0 && joyAxis(ThrottleR , 50) > 0) {
          motorDriverRun((int16_t) (joyAxis(Ry , 50) * MOTOR_NORMAL_GAIN * (joyAxis(ThrottleR , 50) * 0.004) )  , 0.999 , (int16_t)(joyAxis(Ly , 50) * MOTOR_NORMAL_GAIN * (joyAxis(ThrottleL , 50) * 0.004 )) , 0.999);
        }
        else
        {
          motorDriverRun((int16_t) (joyAxis(Ry , 50) )  , 0.999 , (int16_t)(joyAxis(Ly , 50)) , 0.999);
        }
      }
      else if (modeSelector == 1) {
        float calculateAngle = atan2(joyAxis(Lx , 35) , joyAxis(Ly , 35)) * RAD_TO_DEG;

        if (fabs(calculateAngle) > 170 ) {
          baseSpeed = (joyAxis(Ly , 35) * MOTOR_NORMAL_GAIN);
          calculateAngle = 0;
        }
        else {
          baseSpeed = (joyAxis(Ly , 35) * MOTOR_NORMAL_GAIN);
        }

        headingController(calculateAngle , baseSpeed);

        motorDriverA(HeadingSpeed.speedRight);
        motorDriverB(HeadingSpeed.speedLeft);


      }
      else if (modeSelector == 2) {
        float calculateAngle = atan2(joyAxis(Rx , 35) , joyAxis(Ry , 35)) * RAD_TO_DEG;

        if (fabs(calculateAngle) > 170 ) {
          baseSpeed = (joyAxis(Ly , 35) * MOTOR_NORMAL_GAIN);
          calculateAngle = 0;
        }
        else {
          baseSpeed = (joyAxis(Ly , 35) * MOTOR_NORMAL_GAIN);
        }

        headingController(calculateAngle , baseSpeed);
        motorDriverA(HeadingSpeed.speedRight);
        motorDriverB(HeadingSpeed.speedLeft);
      }


    }
    else {
      motorDriverA(0);
      motorDriverB(0);
    }

  }



  //    Serial.printf("%d , %d , %d , %d \n ", modeSelector, joyButton(R1) , HeadingSpeed.speedLeft  ,HeadingSpeed.speedRight);


  OTA_RUNTIME();

}

void loop() {
  mainControl();
}
