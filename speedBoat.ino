#include "variable.h"
#include "StorageController.h"
#include "IMUReader.h"
#include "MotionController.h"
#include "UDPConnector.h"
#include "JoystickController.h"
#include "System.h"


void setup() {
  motor_initial();
  interface_initial();
  storage_initial();
  imu_initial();
  system_initial();
  joystick_initial();

  set_param_heading(20.00 , 0.00 , 0.00 , 3500);
}

float baseSpeed = 0;
void loop() {
  joystick_run();

  float calculateAngle = atan2(joyAxis(Lx , 35) , joyAxis(Ly , 35)) * RAD_TO_DEG;

  if (fabs(calculateAngle) > 170 ) {
    baseSpeed = (joyAxis(Ly , 35) * 2.04600);
    calculateAngle = 0;
  }
  else {
    baseSpeed = (joyAxis(Ly , 35) * 2.04600);
  }


  headingController(calculateAngle , baseSpeed);



  motorDriverA(HeadingSpeed.speedLeft );
  motorDriverB(HeadingSpeed.speedRight);

  //  joyDebug();

  Serial.printf("%f %d %d \n" , calculateAngle ,  HeadingSpeed.speedLeft , HeadingSpeed.speedRight);

  //  Serial.println(joyButtonBasic.asByte , BIN);
  //  Serial.printf("%d %d %f \n", HeadingSpeed.speedLeft , HeadingSpeed.speedRight , imu_readable);
}
