#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int16_t joyAxisRx = 0;
int16_t joyAxisRy = 0;
int16_t joyAxisLx = 0;
int16_t joyAxisLy = 0;

bool joystickConnected = false;

enum JoyButtonControl {
  Cross = 0,
  Circle = 1,
  Rectangle = 2,
  Triangle = 3,
  L1 = 4,
  R1 = 5,
  L2 = 7,
  R2 = 8,

} ;


enum JoyAnalogControl {
  Lx = 0,
  Ly = 1,
  Rx = 2,
  Ry = 3,
} ;
union BoolConverter //Union data for convertion between byte and bit
{
  uint16_t asByte;
  struct asBool
  {
    unsigned int bit0 : 1;
    unsigned int bit1 : 1;
    unsigned int bit2 : 1;
    unsigned int bit3 : 1;
    unsigned int bit4 : 1;
    unsigned int bit5 : 1;
    unsigned int bit6 : 1;
    unsigned int bit7 : 1;
    unsigned int bit8 : 1;
    unsigned int bit9 : 1;
    unsigned int bit10 : 1;
    unsigned int bit11 : 1;
    unsigned int bit12 : 1;
    unsigned int bit13 : 1;
    unsigned int bit14 : 1;
    unsigned int bit15 : 1;

  } asBool;
} joyButtonBasic;

int16_t joyAxis(JoyAnalogControl _param , int16_t deathZone) {
  switch (_param) {
    case Lx :
      if (abs(joyAxisLx) > deathZone) {
        return joyAxisLx;
      }
      else {
        return 0;
      }
      break;
    case Ly :
      if (abs(joyAxisLy) > deathZone) {
        return -joyAxisLy;
      }
      else {
        return 0;
      }
      break;
    case Rx :
      if (abs(joyAxisRx) > deathZone) {
        return joyAxisRx;
      }
      else {
        return 0;
      }
      break;
    case Ry:
      if (abs(joyAxisRy) > deathZone) {
        return -joyAxisRy;
      }
      else {
        return 0;
      }
      break;
  }
}
bool joyButton(JoyButtonControl _param) {
  switch (_param) {
    case Cross :
      return joyButtonBasic.asBool.bit0;
      break;
    case Circle :
      return joyButtonBasic.asBool.bit1;
      break;
    case Rectangle :
      return joyButtonBasic.asBool.bit2;
      break;
    case Triangle :
      return joyButtonBasic.asBool.bit3;
      break;
    case L1 :
      return joyButtonBasic.asBool.bit4;
      break;
    case R1 :
      return joyButtonBasic.asBool.bit5;
      break;
    case L2 :
      return joyButtonBasic.asBool.bit7;
      break;
    case R2 :
      return joyButtonBasic.asBool.bit8;
      break;
    default:
      break;
  }

}

void joyDebug() {
  Serial.printf("Bits: ");
  Serial.printf("%d ", joyButton(Cross));
  Serial.printf("%d ", joyButton(Rectangle));
  Serial.printf("%d ", joyButton(Triangle));
  Serial.printf("%d ", joyButton(Circle));
  Serial.printf("%d ", joyButtonBasic.asBool.bit4);
  Serial.printf("%d ", joyButtonBasic.asBool.bit5);
  Serial.printf("%d ", joyButtonBasic.asBool.bit6);
  Serial.printf("%d ", joyButtonBasic.asBool.bit7);
  Serial.printf("%d ", joyButtonBasic.asBool.bit8);
  Serial.printf("%d ", joyButtonBasic.asBool.bit9);
  Serial.printf("%d ", joyButtonBasic.asBool.bit10);
  Serial.printf("%d ", joyButtonBasic.asBool.bit11);
  Serial.printf("%d ", joyButtonBasic.asBool.bit12);
  Serial.printf("%d ", joyButtonBasic.asBool.bit13);
  Serial.printf("%d ", joyButtonBasic.asBool.bit14);
  Serial.printf("%d ", joyButtonBasic.asBool.bit15);
  Serial.printf("%d  %d  %d  %d" , joyAxis(Lx , 35) , joyAxis(Ly , 35) , joyAxis(Rx , 35) , joyAxis(Ry , 35));
  Serial.println();
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      joystickConnected = true;
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      joystickConnected = false;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  joyAxisRx = ctl->axisRX();
  joyAxisRy = ctl->axisRY();
  joyAxisLx = ctl->axisX();
  joyAxisLy = ctl->axisY();
  joyButtonBasic.asByte  = ctl->buttons();
}



void processControllers() {
  if (myControllers[0] && myControllers[0]->isConnected() && myControllers[0]->hasData()) {
    if (myControllers[0]->isGamepad()) {
      processGamepad(myControllers[0]);

    } else {
      Serial.println("Unsupported controller");
    }
  }

}

void joystick_initial() {
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);


  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

}

void joystick_run() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
}
