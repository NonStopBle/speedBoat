

void set_param_heading(float kp , float ki , float kd , int32_t maxSumError  = 3500) {
  HeadingController.kp = kp;
  HeadingController.ki = ki;
  HeadingController.kd = kd;
  HeadingController.max_sumError = maxSumError;
  ConfigurationState.HeadingControlState = HeadingController;
  storage_write();
}


void headingController(float setpoint , int16_t motorBaseSpeed) {

  imu_run(false, 10);

  readable_hsetpoint_public = setpoint;

  HeadingController.error  = setpoint - imu_readable;

  if (HeadingController.error < -180) {
    HeadingController.error += 360;
  }
  else if (HeadingController.error > 180) {
    HeadingController.error -= 360;
  }

  HeadingController.sumError += HeadingController.error;

  if (HeadingController.sumError >= HeadingController.max_sumError) {
    HeadingController.sumError = HeadingController.max_sumError;
  }
  else if (HeadingController.sumError < -HeadingController.max_sumError) {
    HeadingController.sumError = -HeadingController.max_sumError;
  }

  HeadingController.output = (HeadingController.kp * HeadingController.error) + (HeadingController.ki * HeadingController.max_sumError) + (HeadingController.kd * (HeadingController.error - HeadingController.prev_error));
  HeadingController.prev_error = HeadingController.error;

  HeadingSpeed.speedLeft = motorBaseSpeed - HeadingController.output ;
  HeadingSpeed.speedRight = motorBaseSpeed + HeadingController.output ;


  HeadingSpeed.speedLeft = constrain(HeadingSpeed.speedLeft , -MOTOR_A_MAX_SPEED , MOTOR_A_MAX_SPEED);
  HeadingSpeed.speedRight= constrain(HeadingSpeed.speedRight , -MOTOR_B_MAX_SPEED , MOTOR_B_MAX_SPEED);

}



void motorDriverA(int16_t _pwm) {
  if (_pwm > 0 && _pwm < MOTOR_A_MAX_SPEED) {
    ledcWrite(DRIVE_A_CHANNEL , _pwm);
    digitalWrite(MOTOR_A_IN1 , HIGH);
    digitalWrite(MOTOR_A_IN2 , LOW);
    //    digitalWrite(MOTOR_STANBY , HIGH);
  }
  else if (_pwm < 0 &&  _pwm > -MOTOR_A_MAX_SPEED) {
    ledcWrite(DRIVE_A_CHANNEL , abs(_pwm));
    digitalWrite(MOTOR_A_IN1 , LOW);
    digitalWrite(MOTOR_A_IN2 , HIGH);
    //    digitalWrite(MOTOR_STANBY , HIGH);
  }
  else {
    digitalWrite(MOTOR_A_IN1 , LOW);
    digitalWrite(MOTOR_A_IN2 , LOW);
    //    digitalWrite(MOTOR_STANBY , LOW);
  }
}

void motorDriverB(int16_t _pwm) {
  if (_pwm > 0 && _pwm < MOTOR_A_MAX_SPEED) {
    ledcWrite(DRIVE_B_CHANNEL , _pwm);
    digitalWrite(MOTOR_B_IN1 , HIGH);
    digitalWrite(MOTOR_B_IN2 , LOW);
    //    digitalWrite(MOTOR_STANBY , HIGH);
  }
  else if (_pwm < 0 &&  _pwm > -MOTOR_A_MAX_SPEED) {
    ledcWrite(DRIVE_B_CHANNEL , abs(_pwm));
    digitalWrite(MOTOR_B_IN1 , LOW);
    digitalWrite(MOTOR_B_IN2 , HIGH);
    //    digitalWrite(MOTOR_STANBY , HIGH);
  }
  else {
    digitalWrite(MOTOR_B_IN1 , LOW);
    digitalWrite(MOTOR_B_IN2 , LOW);
    //    digitalWrite(MOTOR_STANBY , LOW);
  }
}
