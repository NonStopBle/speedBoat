

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

  HeadingSpeed.speedLeft = motorBaseSpeed + HeadingController.output ;
  HeadingSpeed.speedRight = motorBaseSpeed - HeadingController.output ;


  HeadingSpeed.speedLeft = constrain(HeadingSpeed.speedLeft , -MOTOR_A_HEADING_MAX_SPEED , MOTOR_A_HEADING_MAX_SPEED);
  HeadingSpeed.speedRight = constrain(HeadingSpeed.speedRight , -MOTOR_B_HEADING_MAX_SPEED , MOTOR_B_HEADING_MAX_SPEED);


  if (fabs(HeadingController.error <= 1)) {
    HeadingController.sumError = 0;
  }
}

float ema_ramp_value_a = 0;
float EMARampA(float _value , float _filter ) {
  ema_ramp_value_a = ((_value * (1.0f - _filter)) + (ema_ramp_value_a * _filter));
  return ema_ramp_value_a;
}

float ema_ramp_value_b = 0;
float EMARampB(float _value , float _filter ) {
  ema_ramp_value_b = ((_value * (1.0f - _filter)) + (ema_ramp_value_b * _filter));
  return ema_ramp_value_b;
}




void motorDriverA(int16_t dutyCycle , float rampGain = 0.00f) {

  if (dutyCycle > 0) {
    ledcWrite(DRIVE_A_L_CHANNEL , constrain(EMARampA(dutyCycle , rampGain) , 0 , MOTOR_A_MAX_SPEED ));
    ledcWrite(DRIVE_A_R_CHANNEL , 0);
  }
  else if (dutyCycle < 0) {
    ledcWrite(DRIVE_A_L_CHANNEL , 0);
    ledcWrite(DRIVE_A_R_CHANNEL , constrain(abs(EMARampA(dutyCycle , rampGain)) , 0 , MOTOR_A_MAX_SPEED ));
  }
  else {
    //    EMARampA(0,0,true);
    ema_ramp_value_a = 0;
    ledcWrite(DRIVE_A_L_CHANNEL , 0);
    ledcWrite(DRIVE_A_R_CHANNEL , 0);
  }
}

void motorDriverB(int16_t dutyCycle, float rampGain = 0.00f) {
  if (dutyCycle > 0) {
    ledcWrite(DRIVE_B_L_CHANNEL , constrain(EMARampB(dutyCycle , rampGain)  , 0 , MOTOR_B_MAX_SPEED ));
    ledcWrite(DRIVE_B_R_CHANNEL , 0);
  }
  else if (dutyCycle < 0) {
    ledcWrite(DRIVE_B_L_CHANNEL , 0);
    ledcWrite(DRIVE_B_R_CHANNEL , constrain(abs(EMARampB(dutyCycle , rampGain)) , 0 , MOTOR_B_MAX_SPEED ));
  }
  else {
    //    EMARampB(0,0,true);
    ema_ramp_value_b = 0;
    ledcWrite(DRIVE_B_L_CHANNEL , 0);
    ledcWrite(DRIVE_B_R_CHANNEL , 0);
  }

  //  Serial.printf("%d " , dutyCycle);

}
void motorDriverRun(int16_t dutyCycleL , float rampGainL , int16_t dutyCycleR , float rampGainR) {
  static uint32_t prev = 0;
  if (millis() - prev >= 1) {
    motorDriverA(dutyCycleL , rampGainL);
    motorDriverB(dutyCycleR , rampGainR);

    Serial.printf("%d , %d \n" , dutyCycleL , dutyCycleR);
    prev = millis();
  }
}

//
//
//void motorDriverA(int16_t _pwm) {
//  if (_pwm > 0)  {
//    ledcWrite(DRIVE_A_CHANNEL , constrain(_pwm , 0 , MOTOR_A_MAX_SPEED ));
//    digitalWrite(MOTOR_A_IN1 , HIGH);
//    digitalWrite(MOTOR_A_IN2 , LOW);
//    //    digitalWrite(MOTOR_STANBY , HIGH);
//  }
//  else if (_pwm < 0 ) {
//    ledcWrite(DRIVE_A_CHANNEL , constrain(abs(_pwm) , 0 , MOTOR_A_MAX_SPEED));
//    digitalWrite(MOTOR_A_IN1 , LOW);
//    digitalWrite(MOTOR_A_IN2 , HIGH);
//    //    digitalWrite(MOTOR_STANBY , HIGH);
//  }
//  else {
//    digitalWrite(MOTOR_A_IN1 , LOW);
//    digitalWrite(MOTOR_A_IN2 , LOW);
//    //    digitalWrite(MOTOR_STANBY , LOW);
//  }
//}
//
//void motorDriverB(int16_t _pwm) {
//  if (_pwm > 0 ) {
//    ledcWrite(DRIVE_B_CHANNEL , constrain(_pwm , 0 , MOTOR_A_MAX_SPEED ));
//    digitalWrite(MOTOR_B_IN1 , HIGH);
//    digitalWrite(MOTOR_B_IN2 , LOW);
//    //    digitalWrite(MOTOR_STANBY , HIGH);
//  }
//  else if (_pwm < 0 ) {
//    ledcWrite(DRIVE_B_CHANNEL , constrain(abs(_pwm) , 0 , MOTOR_A_MAX_SPEED));
//    digitalWrite(MOTOR_B_IN1 , LOW);
//    digitalWrite(MOTOR_B_IN2 , HIGH);
//    //    digitalWrite(MOTOR_STANBY , HIGH);
//  }
//  else {
//    digitalWrite(MOTOR_B_IN1 , LOW);
//    digitalWrite(MOTOR_B_IN2 , LOW);
//    //    digitalWrite(MOTOR_STANBY , LOW);
//  }
//}
