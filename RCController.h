#define RECEIVER_CH_1 32
#define RECEIVER_CH_2 35
#define RECEIVER_CH_3 34
#define RECEIVER_CH_4 39

enum RCRemoteParam {
  CircleWheels = 0,
  Accelerator = 1,
  ToggleButton = 2,
  PressButton = 3,
};


volatile uint64_t receiver_ch_1_pulseRising = 0;
volatile int32_t receiver_ch_1_pulseWidth = 0;

void IRAM_ATTR ISR_RECEIVER_CH_1() {
  if ((GPIO.in1.val >> (RECEIVER_CH_1 - 32)) & 0x1) {
    receiver_ch_1_pulseRising = micros();
  }
  else {
    receiver_ch_1_pulseWidth = (micros() - receiver_ch_1_pulseRising);
  }
}

volatile uint64_t receiver_ch_2_pulseRising = 0;
volatile int32_t receiver_ch_2_pulseWidth = 0;

void IRAM_ATTR ISR_RECEIVER_CH_2() {
  if ((GPIO.in1.val >> (RECEIVER_CH_2 - 32)) & 0x1) {
    receiver_ch_2_pulseRising = micros();
  }
  else {
    receiver_ch_2_pulseWidth = (micros() - receiver_ch_2_pulseRising);
  }
}


volatile uint64_t receiver_ch_3_pulseRising = 0;
volatile int32_t receiver_ch_3_pulseWidth = 0;

void IRAM_ATTR ISR_RECEIVER_CH_3() {
  if ((GPIO.in1.val >> (RECEIVER_CH_3 - 32)) & 0x1) {
    receiver_ch_3_pulseRising = micros();
  }
  else {
    receiver_ch_3_pulseWidth = (micros() - receiver_ch_3_pulseRising);
  }
}


volatile uint64_t receiver_ch_4_pulseRising = 0;
volatile int32_t receiver_ch_4_pulseWidth = 0;

void IRAM_ATTR ISR_RECEIVER_CH_4() {
  if ((GPIO.in1.val >> (RECEIVER_CH_4 - 32)) & 0x1) {
    receiver_ch_4_pulseRising = micros();
  }
  else {
    receiver_ch_4_pulseWidth = (micros() - receiver_ch_4_pulseRising);
  }
}
void rc_control_initial() {
  // put your setup code here, to run once:
  GPIO.enable_w1tc = (1 << RECEIVER_CH_1);
  GPIO.enable_w1tc = (1 << RECEIVER_CH_2);
  GPIO.enable_w1tc = (1 << RECEIVER_CH_3);
  GPIO.enable_w1tc = (1 << RECEIVER_CH_4);


  attachInterrupt(digitalPinToInterrupt(RECEIVER_CH_1) , ISR_RECEIVER_CH_1 , CHANGE);
  attachInterrupt(digitalPinToInterrupt(RECEIVER_CH_2) , ISR_RECEIVER_CH_2 , CHANGE);
  attachInterrupt(digitalPinToInterrupt(RECEIVER_CH_3) , ISR_RECEIVER_CH_3 , CHANGE);
  attachInterrupt(digitalPinToInterrupt(RECEIVER_CH_4) , ISR_RECEIVER_CH_4 , CHANGE);
}

bool readConnectionRC(){
  return  ((receiver_ch_2_pulseWidth - 1515) >- 510) > 0 ? true : false; 
}

int32_t readAnalogRC(RCRemoteParam _param , int16_t deathZone = 5) {
  if ((receiver_ch_2_pulseWidth - 1515) >- 510) {
    switch (_param) {
      case CircleWheels :
        return abs(receiver_ch_1_pulseWidth -  1465) > deathZone ? (receiver_ch_1_pulseWidth -  1465) : 0;
        break;
      case Accelerator :
        return abs(receiver_ch_2_pulseWidth -  1515) > deathZone ? (receiver_ch_2_pulseWidth -  1515) : 0;
      default :
        return 0;
        break;
    }
  }
  else{
    return 0;
  }

}

bool readButtonRC(RCRemoteParam _param) {
  switch (_param) {
    case ToggleButton :
      if (abs(receiver_ch_3_pulseWidth -  984) > 100) {
        return true;
      }
      else {
        return false;
      }
      break;
    case PressButton :
      if (abs(receiver_ch_4_pulseWidth -  1964) > 100) {
        return true;
      }
      else {
        return false;
      }
      break;
    default :
      return false;
      break;
  }
}



void rc_control_example() {
  //  deltaTime = pulseIn(RECEIVER_CH_1 , HIGH );
//  Serial.printf("ch1: %ld  , ch2: %ld , ch3: %d, ch4:%d \n" , readAnalogRC(CircleWheels) , readAnalogRC(Accelerator) , readButtonRC(ToggleButton)  , readButtonRC(PressButton));
  // Serial.println(digitalRead(RECEIVER_CH_1));
}
