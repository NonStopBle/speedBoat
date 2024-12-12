//#include "soc/gpio_struct.h"  // For low-level GPIO control
//#include "driver/gpio.h"      // For GPIO configuration
#include <WiFi.h>
/////////////////////////////
//    MOTOR A DRIVE  ////////
/////////////////////////////
#define MOTOR_A_PWM 33
#define MOTOR_A_IN1 25
#define MOTOR_A_IN2 26

#define DRIVE_A_CHANNEL 0
#define DRIVE_A_FREQ 10000
#define DRIVE_A_RESOLUTION 10
/////////////////////////////
#define MOTOR_STANBY 27
/////////////////////////////
//    MOTOR B DRIVE  ////////
/////////////////////////////
#define MOTOR_B_PWM 13
#define MOTOR_B_IN1 14
#define MOTOR_B_IN2 32

#define DRIVE_B_CHANNEL 1
#define DRIVE_B_FREQ 10000
#define DRIVE_B_RESOLUTION 10

float imu_readable = 0.00f;

#define MOTOR_A_MAX_SPEED 1023
#define MOTOR_B_MAX_SPEED 1023

#define BNO_INTERRUPT 19

#define LED_STATUS 2

const char* ssid = "iRAP";
const char* pass = "irapiluvu";


struct MotorController {
  int16_t speedLeft = 0;
  int16_t speedRight = 0;
} HeadingSpeed;

struct PIDController {
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float sumError = 0;
  float max_sumError  = 0;
  float error = 0;
  float prev_error = 0;
  float output = 0;
} HeadingController;


struct SPIFFMemory {
  PIDController HeadingControlState;
} ConfigurationState , ConfigurationLoaded ;

float readable_hsetpoint_public = 0.00f;

void motor_initial() {
  pinMode(MOTOR_A_IN1  , OUTPUT);
  pinMode(MOTOR_A_IN2  , OUTPUT);
  pinMode(MOTOR_A_PWM , OUTPUT);
  pinMode(MOTOR_B_IN1  , OUTPUT);
  pinMode(MOTOR_B_IN2  , OUTPUT);
  pinMode(MOTOR_B_PWM , OUTPUT);
  pinMode(MOTOR_STANBY , OUTPUT);
  pinMode(BNO_INTERRUPT , OUTPUT);
  pinMode(LED_STATUS  , OUTPUT);


  digitalWrite(BNO_INTERRUPT , HIGH);
  digitalWrite(MOTOR_STANBY , HIGH);

  //  GPIO.out_w1ts = (1 << MOTOR_STANBY);

  ledcSetup(DRIVE_A_CHANNEL, DRIVE_A_FREQ, DRIVE_A_RESOLUTION);
  ledcAttachPin(MOTOR_A_PWM  , DRIVE_A_CHANNEL);

  ledcSetup(DRIVE_B_CHANNEL, DRIVE_B_FREQ, DRIVE_B_RESOLUTION);
  ledcAttachPin(MOTOR_B_PWM  , DRIVE_B_CHANNEL);
}


void interface_initial() {
  Serial.begin(115200);
  //  WiFi.setTxPower(WIFI_POWER_19_5dBm);    // Set WiFi RF power output to highest level
  //  WiFi.begin(ssid , pass);
  //  while (!WiFi.status() != WL_CONNECTED) {
  //    Serial.print(".");
  //    delay(10);
  //  };
  //  Serial.println("Connected");

}
