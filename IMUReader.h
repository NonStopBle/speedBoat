#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void imu_initial() {
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);

  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void imu_run(bool _debug = false , int16_t rate = 10) {
  static int32_t previousTime = 0;
  if (millis() - previousTime >= rate) {
    sensors_event_t event;
    bno.getEvent(&event);

    imu_readable = event.orientation.x;

    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    if (_debug) {
      Serial.print(F("Orientation: "));
      Serial.print((float)event.orientation.x);
      Serial.print(F(" "));
      Serial.print((float)event.orientation.y);
      Serial.print(F(" "));
      Serial.print((float)event.orientation.z);
      Serial.println(F(""));
//      Serial.print(F("Calibration: "));
//      Serial.print(sys, DEC);
//      Serial.print(F(" "));
//      Serial.print(gyro, DEC);
//      Serial.print(F(" "));
//      Serial.print(accel, DEC);
//      Serial.print(F(" "));
//      Serial.println(mag, DEC);
    }
    previousTime = millis();
  }
}
