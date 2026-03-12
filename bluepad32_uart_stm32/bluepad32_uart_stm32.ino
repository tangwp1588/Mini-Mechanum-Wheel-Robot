//coded by: Wei Peng

#include <Bluepad32.h>
#include <uni.h>
#include <HardwareSerial.h>

// Define a struct to pack controller data
#pragma pack(push, 1)
struct ControllerInput {
  int16_t axisX, axisY, axisRX, axisRY;
  uint16_t dpad;
  uint16_t buttons;
  uint16_t brake, throttle;
};
#pragma pack(pop)

HardwareSerial SerialToSTM32(1); // Use UART1 (adjust pins as needed)

static const char * controller_addr_string = "A0:5A:5D:FE:B8:AB"; //my controller's MAC Address

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

ControllerPtr Controller; 

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;

      // to ensure only can connect to one controller
      if (Controller == nullptr) {
        Controller = ctl;
      }
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
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

// void dumpGamepad(ControllerPtr ctl) {
//   Serial.printf(
//     "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
//     "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
//     ctl->index(),        // Controller Index
//     ctl->dpad(),         // D-pad
//     ctl->buttons(),      // bitmask of pressed buttons
//     ctl->axisX(),        // (-511 - 512) left X Axis
//     ctl->axisY(),        // (-511 - 512) left Y axis
//     ctl->axisRX(),       // (-511 - 512) right X axis
//     ctl->axisRY(),       // (-511 - 512) right Y axis
//     ctl->brake(),        // (0 - 1023): brake button
//     ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
//     ctl->miscButtons(),  // bitmask of pressed "misc" buttons
//     ctl->gyroX(),        // Gyro X
//     ctl->gyroY(),        // Gyro Y
//     ctl->gyroZ(),        // Gyro Z
//     ctl->accelX(),       // Accelerometer X
//     ctl->accelY(),       // Accelerometer Y
//     ctl->accelZ()        // Accelerometer Z
//     );
// }

/*
  Direwolf 2 Controller:
  MAC Address: A0:5A:5D:FE:B8:AB

  Analog:
  Left  Joystick: min_y(up) = -512, max_y(down) = 508, min_x(left) = -512, max_x(right) = 508
  Right Joystick: min_y(up) = -512, max_y(down) = 508, min_x(left) = -512, max_x(right) = 508
  Left  Trigger(brake)   : 0-1020
  Right Trigger(throttle): 0-1020
  
  Digital:
  Dpad   : up(0x01), down(0x02), left(0x08), right(0x04), up+left(0x09), up+right(0x05), down+left(0x0a), down+right(0x06)
  Buttons: X(0x0004), Y(0x0008), A(0x0001), B(0x0002), X+A(0x0005), Y+B(0x000a)
            LS(0x0100), RS(0x0200), LS+RS(0x0300)
            LB(0x0010), RB(0x0020), LB+RB(0x0030)
            M1(none), M2=A=(0x0001)
  MiscButtons: Start(0x04), Select(0x02) 

*/

void processGamepad(ControllerPtr ctl) {
  ControllerInput data;
  data.axisX = ctl->axisX();
  data.axisY = ctl->axisY();
  data.axisRX = ctl->axisRX();
  data.axisRY = ctl->axisRY();
  data.dpad = ctl->dpad();
  data.buttons = ctl->buttons();
  data.brake = ctl->brake();
  data.throttle = ctl->throttle();

  // Debug print (view in Arduino Serial Monitor)
  Serial.printf("Sending: LX=%d, LY=%d, RX=%d, RY=%d, B=%04x\n", data.axisX, data.axisY, data.axisRX, data.axisRY, data.buttons);

  // Send data via UART
  size_t bytesSent = SerialToSTM32.write((uint8_t*)&data, sizeof(data));
  if (bytesSent != sizeof(data)) {
    Serial.println("UART Write Error!");
  }
  //dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  SerialToSTM32.begin(115200, SERIAL_8N1, 16, 17); // RX2 = 16, TX2 = 17
  //To ensure only my controller can connect
  bd_addr_t controller_addr;
  sscanf_bd_addr(controller_addr_string, controller_addr);
  uni_bt_allowlist_add_addr(controller_addr);
  uni_bt_allowlist_set_enabled(true);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time
  if (Controller == nullptr)
  {
    Serial.println("Your controller is not connected");
    return;
  }

  vTaskDelay(1);
}
