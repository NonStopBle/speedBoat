#include <ArduinoOTA.h>




void OTAInitial() {
  // หากเชื่อมต่อไม่สำเร็จ รอ 5 วินาทีแล้ว Restart ใหม่
  //  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //    Serial.println("Connection Failed! Rebooting...");
  //  }
  //  else {
  ArduinoOTA
  .onStart([]() {
    String type;          // ประเภทของ OTA ที่เข้ามา
    if (ArduinoOTA.getCommand() == U_FLASH)         // แบบ U_FLASH
      type = "sketch";
    else          // แบบ U_SPIFFS
      type = "filesystem";

    // NOTE: ถ้าใช้เป็นแบบ SPIFFS อาจใช้คำสั่ง SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })

  // เริ่มทำงาน (รับข้อมูลโปรแกรม) พร้อมแสดงความคืบหน้าทาง Serial Monitor
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })

  // แสดงข้อความต่างๆหากเกิด Error ขึ้น
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //  }

  //  Serial.begin(9600);
}

void OTA_RUNTIME() {
  //  if (robot_wifi_connected) {
  ArduinoOTA.handle();
  //  }
}

void system_initial() {
  pinMode(LED_STATUS , OUTPUT);
  storage_read();
  OTAInitial();
}

void system_led_blinking() {

}
void system_runtime() {

}
