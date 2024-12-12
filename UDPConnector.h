#include <WiFi.h>
#include <WiFiUdp.h>

const char* receiverIp = "192.168.4.2";
uint16_t udpPort = 7445;

WiFiUDP udp;

// udp using port 7445 by default
void udp_initial() {
  udp.begin(udpPort);
}

union FloatConvert {
  float asFloat = 0.00f;
  uint8_t asByte[4];
} current_heading_kp_write , current_heading_ki_write , current_heading_kd_write , current_heading_sum_error_write , current_setpoint_write , current_value_write ,
current_heading_kp_read , current_heading_ki_read , current_heading_kd_read , current_heading_sum_error_read , current_setpoint_read , current_value_read ;


union Int16Convert {
  int16_t asInt = 0;
  uint16_t asByte[2];
} current_mode_read , current_save_action_read , current_motor_speed_left , current_motor_speed_right;


uint8_t datapackage[22] = {35 , 0, 0, 0, 0  , 0, 0, 0, 0 , 0, 0, 0, 0  , 0, 0, 0, 0 , 0, 0, 0, 0 , '\n'};
uint8_t dataReceived[24] = {};
boolean oneticker[5] = {};

void udp_run(uint16_t rateMillis = 10) {
  static uint32_t previousTime = 0;

  if (millis() - previousTime >= rateMillis) {

    current_heading_kp_write.asFloat = HeadingController.kp ;
    current_heading_ki_write.asFloat = HeadingController.ki ;
    current_heading_kd_write.asFloat = HeadingController.kd ;

    current_setpoint_write.asFloat = readable_hsetpoint_public;
    current_value_write.asFloat = imu_readable;

    // #
    // kp
    datapackage[1] = current_heading_kp_write.asByte[0];
    datapackage[2] = current_heading_kp_write.asByte[1];
    datapackage[3] = current_heading_kp_write.asByte[2];
    datapackage[4] = current_heading_kp_write.asByte[3];
    // ki
    datapackage[5] = current_heading_ki_write.asByte[0];
    datapackage[6] = current_heading_ki_write.asByte[1];
    datapackage[7] = current_heading_ki_write.asByte[2];
    datapackage[8] = current_heading_ki_write.asByte[3];
    // kd
    datapackage[9] = current_heading_kd_write.asByte[0];
    datapackage[10] = current_heading_kd_write.asByte[1];
    datapackage[11] = current_heading_kd_write.asByte[2];
    datapackage[12] = current_heading_kd_write.asByte[3];
    // set point
    datapackage[13] = current_setpoint_write.asByte[0];
    datapackage[13] = current_setpoint_write.asByte[1];
    datapackage[14] = current_setpoint_write.asByte[2];
    datapackage[15] = current_setpoint_write.asByte[3];
    // value
    datapackage[16] = current_value_write.asByte[0];
    datapackage[17] = current_value_write.asByte[1];
    datapackage[18] = current_value_write.asByte[2];
    datapackage[19] = current_value_write.asByte[3];

    udp.beginPacket(receiverIp, udpPort);
    udp.write(datapackage , sizeof(datapackage));
    udp.endPacket();

    int packetSize = udp.parsePacket();
    Serial.println(udp.remoteIP());
    Serial.println(packetSize);
    //    if (packetSize >= 22) {
    //      udp.read(dataReceived , sizeof(dataReceived));
    //      if (dataReceived[0] == '#' && dataReceived[29] == '\n') {
    //        current_heading_kp_read.asByte[0] = dataReceived[1];
    //        current_heading_kp_read.asByte[1] = dataReceived[2];
    //        current_heading_kp_read.asByte[2] = dataReceived[3];
    //        current_heading_kp_read.asByte[3] = dataReceived[4];
    //
    //        current_heading_ki_read.asByte[0] = dataReceived[5];
    //        current_heading_ki_read.asByte[1] = dataReceived[6];
    //        current_heading_ki_read.asByte[2] = dataReceived[7];
    //        current_heading_ki_read.asByte[3] = dataReceived[8];
    //
    //        current_heading_kd_read.asByte[0] = dataReceived[9];
    //        current_heading_kd_read.asByte[1] = dataReceived[10];
    //        current_heading_kd_read.asByte[2] = dataReceived[11];
    //        current_heading_kd_read.asByte[3] = dataReceived[12];
    //
    //        current_setpoint_read.asByte[0] = dataReceived[13];
    //        current_setpoint_read.asByte[1] = dataReceived[14];
    //        current_setpoint_read.asByte[2] = dataReceived[15];
    //        current_setpoint_read.asByte[3] = dataReceived[16];
    //
    //        current_value_read.asByte[0] = dataReceived[17];
    //        current_value_read.asByte[1] = dataReceived[18];
    //        current_value_read.asByte[2] = dataReceived[19];
    //        current_value_read.asByte[3] = dataReceived[20];
    //
    //        current_mode_read.asByte[0] = dataReceived[21] ;
    //        current_mode_read.asByte[1] = dataReceived[22];
    //
    //        current_save_action_read.asByte[0] = dataReceived[23];
    //        current_save_action_read.asByte[1] = dataReceived[24];
    //
    //        current_motor_speed_left.asByte[0] = dataReceived[25];
    //        current_motor_speed_left.asByte[1] = dataReceived[26];
    //
    //        current_motor_speed_right.asByte[0] = dataReceived[27];
    //        current_motor_speed_right.asByte[1] = dataReceived[28];
    //
    //
    //      }
    //    }

    //    udp.flush();

    previousTime = millis();
  }

  if (current_save_action_read.asInt == 10 && oneticker[0] == false) {
    set_param_heading(current_heading_kp_read.asFloat , current_heading_ki_read.asFloat , current_heading_kd_read.asFloat);
    storage_write();
    oneticker[0] = true;
  }
  else if (current_save_action_read.asInt == 0) {
    storage_reset();
    oneticker[0] = false;
  }
}
