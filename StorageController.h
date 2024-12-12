#include "SPIFFS.h"
bool storageState = false;

void storage_initial() {
  if (!SPIFFS.begin(true)) {
    Serial.printf("An Error has occured while mounting SPIFFS \n");
  }
}

void storage_gain_loader() {
  HeadingController.kp = ConfigurationLoaded.HeadingControlState.kp;
  HeadingController.ki = ConfigurationLoaded.HeadingControlState.ki;
  HeadingController.kd = ConfigurationLoaded.HeadingControlState.kd;
  HeadingController.max_sumError = ConfigurationLoaded.HeadingControlState.max_sumError;
}

void storage_read() {
  File FileMemoryReader = SPIFFS.open("/CONFIG.RZ" , FILE_READ);
  if (!FileMemoryReader) {
    Serial.println("There was an error opening the file for reading");
  }

  if (FileMemoryReader.available()) {
    FileMemoryReader.read((uint8_t*)&ConfigurationLoaded , sizeof(ConfigurationLoaded));
  }
  FileMemoryReader.close();

  storage_gain_loader();

  Serial.printf("%f , %f , %f , %f , %f \n" , ConfigurationLoaded.HeadingControlState.kp , ConfigurationLoaded.HeadingControlState.ki , ConfigurationLoaded.HeadingControlState.kd , ConfigurationLoaded.HeadingControlState.max_sumError);
}

void storage_reset() {
  storageState = false;
}
void storage_write() {
  if (storageState == false) {
    File FileMemory = SPIFFS.open("/CONFIG.RZ", FILE_WRITE);

    if (!FileMemory ) {
      Serial.println("There was an error opening the file for writing");
      return;
    }

    if (FileMemory.write((uint8_t*)&ConfigurationState , sizeof(ConfigurationState))) {
      Serial.println("configuration was written");
    } else {
      Serial.println("configuration write failed");
    }

    FileMemory.close();

    storageState = true;
  }
}
