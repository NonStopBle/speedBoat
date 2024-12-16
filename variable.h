//#include "soc/gpio_struct.h"  // For low-level GPIO control
//#include "driver/gpio.h"      // For GPIO configuration

void webSerialInitial();

#include <WiFi.h>

#define MOTOR_A_MAX_SPEED 1023 // 400
#define MOTOR_B_MAX_SPEED 1023 //400

#define MOTOR_A_DEFAULT_SPEED 500
#define MOTOR_B_DEFAULT_SPEED 500

#define MOTOR_BOOST_GAIN 2.04600 // 1023
#define MOTOR_NORMAL_GAIN 1.04600 // 500

#define MOTOR_A_L_PWM 26
#define DRIVE_A_L_CHANNEL 0
#define DRIVE_A_L_FREQ 10000
#define DRIVE_A_L_RESOLUTION 10

#define MOTOR_A_R_PWM 27
#define DRIVE_A_R_CHANNEL 1
#define DRIVE_A_R_FREQ 10000
#define DRIVE_A_R_RESOLUTION 10

#define MOTOR_B_L_PWM 12
#define DRIVE_B_L_CHANNEL 2
#define DRIVE_B_L_FREQ 10000
#define DRIVE_B_L_RESOLUTION 10

#define MOTOR_B_R_PWM 14
#define DRIVE_B_R_CHANNEL 3
#define DRIVE_B_R_FREQ 10000
#define DRIVE_B_R_RESOLUTION 10

bool isRCRemote = false;
float imu_readable = 0.00f;

#define MOTOR_A_HEADING_MAX_SPEED 400
#define MOTOR_B_HEADING_MAX_SPEED 400

#define BNO_INTERRUPT 19

#define LED_STATUS 2

const char* ssid = "iBoat";
const char* pass = "12341234";


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
uint8_t modeSelector = 0;

void motor_initial() {

  pinMode(MOTOR_A_L_PWM , OUTPUT);
  pinMode(MOTOR_A_R_PWM , OUTPUT);
  pinMode(MOTOR_B_L_PWM , OUTPUT);
  pinMode(MOTOR_B_R_PWM , OUTPUT);

  //  pinMode(MOTOR_STANBY , OUTPUT);
  pinMode(BNO_INTERRUPT , OUTPUT);
  pinMode(LED_STATUS  , OUTPUT);


  digitalWrite(BNO_INTERRUPT , HIGH);
  //  digitalWrite(MOTOR_STANBY , HIGH);

  //  GPIO.out_w1ts = (1 << MOTOR_STANBY);

  ledcSetup(DRIVE_A_L_CHANNEL, DRIVE_A_L_FREQ, DRIVE_A_L_RESOLUTION);
  ledcAttachPin(MOTOR_A_L_PWM  , DRIVE_A_L_CHANNEL);

  ledcSetup(DRIVE_A_R_CHANNEL, DRIVE_A_R_FREQ, DRIVE_A_R_RESOLUTION);
  ledcAttachPin(MOTOR_A_R_PWM  , DRIVE_A_R_CHANNEL);


  ledcSetup(DRIVE_B_L_CHANNEL, DRIVE_B_L_FREQ, DRIVE_B_L_RESOLUTION);
  ledcAttachPin(MOTOR_B_L_PWM  , DRIVE_B_L_CHANNEL);

  ledcSetup(DRIVE_B_R_CHANNEL, DRIVE_B_R_FREQ, DRIVE_B_R_RESOLUTION);
  ledcAttachPin(MOTOR_B_R_PWM  , DRIVE_B_R_CHANNEL);
}


void interface_initial() {
  Serial.begin(115200);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);    // Set WiFi RF power output to highest level
  //  WiFi.setMode(WiFi_STA);
  WiFi.softAP(ssid , pass);
  webSerialInitial();
  //  WiFi.begin(ssid , pass);
  //  while (!WiFi.status() != WL_CONNECTED) {
  //    Serial.print(".");
  //    delay(10);
  //  };
  //  Serial.println("Connected");

}
