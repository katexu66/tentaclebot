// Copyright 2021 - 2023, Ricardo Quesada

#include <Bluepad32.h>
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Servo setup
#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// neutral position for servos; test this -- maybe 0? maybe 90? do we want default to be standing up or to the side?
// int neutralpos;
int neutralpos = 90; // probably need different neutral position for different servos (do!)

// Arduino setup function. Runs in CPU 1
void setup() {
  // Initialize serial
  Serial.begin(9600);

  servo1.attach(5);
  servo2.attach(6);
  servo3.attach(7);
  servo4.attach(8);

  servo1.write(neutralpos);
  servo2.write(neutralpos);
  servo3.write(neutralpos);
  servo4.write(neutralpos);

  Serial.println("Servos set to default position.");

  if (Serial) {
    Serial.println("Serial is connected!");
  }

  delay(1000); // test delay for battery

  String fv = BP32.firmwareVersion();
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  // To get the BD Address (MAC address) call:
  const uint8_t* addr = BP32.localBdAddress();
  Serial.print("BD Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(addr[i], HEX);
    if (i < 5)
      Serial.print(":");
    else
      Serial.println();
  }

  // BP32.pinMode(27, OUTPUT);
  // BP32.digitalWrite(27, 0);

  // This call is mandatory. It sets up Bluepad32 and creates the callbacks.
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();
  Serial.println("Bluetooth keys forgotten.");

}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {

  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.print("CALLBACK: Controller is connected, index=");
      Serial.println(i);
      myControllers[i] = ctl;
      foundEmptySlot = true;

      // Optional, once the gamepad is connected, request further info about the gamepad.
      ControllerProperties properties = ctl->getProperties();
      char buf[80];
      sprintf(buf,
              "BTAddr: %02x:%02x:%02x:%02x:%02x:%02x, VID/PID: %04x:%04x, "
              "flags: 0x%02x",
              properties.btaddr[0], properties.btaddr[1], properties.btaddr[2],
              properties.btaddr[3], properties.btaddr[4], properties.btaddr[5],
              properties.vendor_id, properties.product_id, properties.flags); // I think here is where all the data is coming from; test and tweak this so it's only necessary controller things
      Serial.println(buf);
      break;
    }
  }
  
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Controller connected, but could not found empty slot");
  }
}

// if controller is disconnected
void onDisconnectedController(ControllerPtr ctl) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.print("CALLBACK: Controller is disconnected from index=");
      Serial.println(i);
      myControllers[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr gamepad) {

  // variables for servo control using controllers
  const int joyZone = 16; // define dead zone range for joystick (not sensing anything); maybe create zones for each l/r x/y
  int leftX = gamepad->axisX();
  int leftY = gamepad->axisY();
  int rightX = gamepad->axisRX();
  int rightY = gamepad->axisRY();
  int error = 5;
  int servo1value = servo1.read(); 
  int servo2value = servo2.read();
  int servo3value = servo3.read();
  int servo4value = servo4.read();
  int servo1goal = neutralpos;
  int servo2goal = neutralpos;
  int servo3goal = neutralpos;
  int servo4goal = neutralpos;

  // use left and right joysticks to control servos

  if (abs(leftX) > joyZone) {
    Serial.print("Left joystick moved: ");
    servo1goal = map(leftX, -512, 512, 0, 180);
  } else {
    servo1goal = neutralpos;
  }

  if (abs(servo1value - servo1goal) > error) {
    // increment servo write by small amount and then delay a small amount
    if (servo1value < servo1goal) {
      servo1.write(servo1value + 3);
      delay(80);
    };
    if (servo1value > servo1goal) {
      servo1.write(servo1value - 3);
      delay(80);
    };
  };
  
  if (abs(leftY) > joyZone) {
    Serial.print("Left joystick moved: ");
    servo3goal = map(leftY, -512, 512, 0, 180);
  } else {
    servo3goal = neutralpos;
  }

  if (abs(servo3value - servo3goal) > error) {
    // increment servo write by like 5 and then delay a millisecond
    if (servo3value < servo3goal) {
      servo3.write(servo3value + 5);
      delay(10);
    };
    if (servo3value > servo3goal) {
      servo3.write(servo3value - 5);
      delay(10);
    };
  };

  if (abs(rightX) > joyZone) {
    Serial.print("Right joystick moved: ");
    servo2goal = map(rightX, -512, 512, 0, 180);
  } else {
    servo2goal = neutralpos;
  }

  if (abs(servo2value - servo2goal) > error) {
    // increment servo write by like 5 and then delay a millisecond
    if (servo2value < servo2goal) {
      servo2.write(servo2value + 5);
      delay(10);
    };
    if (servo2value > servo2goal) {
      servo2.write(servo2value - 5);
      delay(10);
    };
  };

  if (abs(rightY) > joyZone) {
    Serial.print("Right joystick moved: ");
    servo4goal = map(rightY, -512, 512, 0, 180);
  } else {
    servo4goal = neutralpos;
  }

  if (abs(servo4value - servo4goal) > error) {
    // increment servo write by like 5 and then delay a millisecond
    if (servo4value < servo4goal) {
      servo4.write(servo4value + 5);
      delay(10);
    };
    if (servo4value > servo4goal) {
      servo4.write(servo4value - 5);
      delay(10);
    };
  };

  // There are different ways to query whether a button is pressed.
  // By query each button individually: a(), b(), x(), y(), l1(), etc...

  // changing color LED using X button (gamepad a)
  if (gamepad->a()) {
    static int colorIdx = 0;
    switch (colorIdx % 3) {
      case 0:
        // Red
        gamepad->setColorLED(255, 0, 0);
        break;
      case 1:
        // Green
        gamepad->setColorLED(0, 255, 0);
        break;
      case 2:
        // Blue
        gamepad->setColorLED(0, 0, 255);
        break;
    }
    colorIdx++;
  }

  // vibrate/rumble using square button (gamepad x)
  if (gamepad->x()) {
    // Duration: 255 is ~2 seconds
    // force: intensity
    gamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
  }

  // Another way to query the buttons, is by calling buttons()
  char buf[256];
  snprintf(buf, sizeof(buf) - 1, // this is how all the info is printed to serial monitor
           "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
           "axis L: %4li, %4li, axis R: %4li, %4li, "
           "brake: %4ld, throttle: %4li "
           //"gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, "
           "battery: %d",
           gamepad->index(),        // Gamepad Index
           gamepad->dpad(),         // DPad
           gamepad->buttons(),      // bitmask of pressed buttons
           gamepad->axisX(),        // (-511 - 512) left X Axis
           gamepad->axisY(),        // (-511 - 512) left Y axis
           gamepad->axisRX(),       // (-511 - 512) right X axis
           gamepad->axisRY(),       // (-511 - 512) right Y axis
           gamepad->brake(),        // (0 - 1023): brake button
           gamepad->throttle(),     // (0 - 1023): throttle (AKA gas) button
           //gamepad->gyroX(),        // Gyro X
           //gamepad->gyroY(),        // Gyro Y
           //gamepad->gyroZ(),        // Gyro Z
           //gamepad->accelX(),       // Accelerometer X
           //gamepad->accelY(),       // Accelerometer Y
           //gamepad->accelZ(),       // Accelerometer Z
           gamepad->battery()       // 0=Unknown, 1=empty, 255=full
  );
  Serial.println(buf);

  // You can query the axis and other properties as well. See Controller.h For all the available functions.
}

// Arduino loop function. Runs in CPU 1
void loop() {
  // This call fetches all the controller info from the NINA (ESP32) module.
  // Call this function in your main loop.
  // The controllers' pointer (the ones received in the callbacks) gets updated automatically.
  BP32.update();

  // It is safe to always do this before using the controller API.
  // This guarantees that the controller is valid and connected.
  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        processGamepad(myController);
    }
  }
  delay(10); // original was 150

}

// put things to control top one on right joystick, put bottom on left joystick
// step values? too jerky/sudden right now, make motion smoother --> set servo map as goal, have servo turn slowly towards goal with error and get slower as the error decreases (like PID)
// neutral pos / straight pos / string tensions