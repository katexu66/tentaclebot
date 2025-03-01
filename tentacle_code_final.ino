// Copyright 2021 - 2023, Ricardo Quesada

#include <Bluepad32.h>
ControllerPtr myControllers[BP32_MAX_CONTROLLERS];

// Servo setup
#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// neutral position for servos; test this -- maybe 0? maybe 90? do we want default to be standing up or to the side? or stay at the last moved place?
// int neutralpos;
int neutralpos = 90;  // probably need different neutral position for different servos (do!)

// Arduino setup function. Runs in CPU 1
void setup() {
  // Initialize serial
  Serial.begin(9600);

  servo1.attach(5);
  servo2.attach(6);
  servo3.attach(7);
  servo4.attach(8);

  servo1.write(neutralpos);
  servo2.write(150); // I think this makes it better? servo2 is the very extreme one
  servo3.write(neutralpos);
  servo4.write(neutralpos);

  Serial.println("Servos set to default position.");

  if (Serial) {
    Serial.println("Serial is connected!");
  }

  delay(1000);  // test delay for battery

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
              properties.vendor_id, properties.product_id, properties.flags);  // I think here is where all the data is coming from; test and tweak this so it's only necessary controller things
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
  // Read joystick inputs
  int leftX = gamepad->axisX();
  int leftY = gamepad->axisY();
  int rightX = gamepad->axisRX();
  int rightY = gamepad->axisRY();

  int error = 3;     // Reduce error threshold for smoother movement
  int stepSize = 3;  // Smaller step size for gradual movement

  // Map joystick values to servo goals; Top half of tentacle is controlled by right joystick, bottom half by left joystick
  int servo1goal = map(rightX, -511, 512, 0, 180);
  int servo2goal = map(0.9*leftX, -511, 512, 0, 180); // to try to make servo2 less fast
  int servo3goal = map(leftY, -511, 512, 0, 180);
  int servo4goal = map(rightY, -512, 512, 0, 180);

  // Gradually move servos toward their goals
  moveServoSmoothly(servo1, servo1goal, stepSize, error);
  moveServoSmoothly(servo2, servo2goal, stepSize, error);
  moveServoSmoothly(servo3, servo3goal, stepSize, error);
  moveServoSmoothly(servo4, servo4goal, stepSize, error);

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
  snprintf(buf, sizeof(buf) - 1,  // this is how all the info is printed to serial monitor
           "idx=%d, dpad: 0x%02x, buttons: 0x%04x, "
           "axis L: %4li, %4li, axis R: %4li, %4li, "
           "brake: %4ld, throttle: %4li "
           //"gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d, "
           "battery: %d",
           gamepad->index(),     // Gamepad Index
           gamepad->dpad(),      // DPad
           gamepad->buttons(),   // bitmask of pressed buttons
           gamepad->axisX(),     // (-511 - 512) left X Axis
           gamepad->axisY(),     // (-511 - 512) left Y axis
           gamepad->axisRX(),    // (-511 - 512) right X axis
           gamepad->axisRY(),    // (-511 - 512) right Y axis
           gamepad->brake(),     // (0 - 1023): brake button
           gamepad->throttle(),  // (0 - 1023): throttle (AKA gas) button
           //gamepad->gyroX(),        // Gyro X
           //gamepad->gyroY(),        // Gyro Y
           //gamepad->gyroZ(),        // Gyro Z
           //gamepad->accelX(),       // Accelerometer X
           //gamepad->accelY(),       // Accelerometer Y
           //gamepad->accelZ(),       // Accelerometer Z
           gamepad->battery()  // 0=Unknown, 1=empty, 255=full
  );
  Serial.println(buf);

  // You can query the axis and other properties as well. See Controller.h For all the available functions.
}

// Function to gradually move servos toward their target positions
void moveServoSmoothly(Servo& servo, int goal, int stepSize, int error) {
  int currentPos = servo.read();

  if (abs(currentPos - goal) > error) {  // Move only if error is significant
    int newPos = currentPos + (goal > currentPos ? stepSize : -stepSize); // Add stepsize if over, subtract if under
    newPos = constrain(newPos, 0, 180);  // Keep within valid servo range
    servo.write(newPos);
  }
}

// Arduino loop function. Runs in CPU 1
void loop() {
  BP32.update();

  for (int i = 0; i < BP32_MAX_CONTROLLERS; i++) {
    ControllerPtr myController = myControllers[i];

    if (myController && myController->isConnected()) {
      if (myController->isGamepad())
        processGamepad(myController);
    }
  }

  delay(5);  // Reduce delay to make it faster/more responsive
}

// step values? too jerky/sudden right now, make motion smoother --> set servo map as goal, have servo turn slowly towards goal with error and get slower as the error decreases (like PID)
// neutral pos / straight pos / string tensions

// make one joystick move bottom part, other joystick move top part (make them the same direction for both)
// Servo 1 and the servo CCW to it (to the right of it, servo 4) is bottom part (left joystick)
// make all four strings move together (add weights/tensions) for each direction to make it smoother