#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>

// ---- Physical wiring on the ESP32-C3 Super Mini ----
#define SDA_PIN       0
#define SCL_PIN       1
#define LEFT_BUTTON        10
#define RIGHT_BUTTON       4
#define PPT_PREVIOUS_BUTTON 6
#define PPT_NEXT_BUTTON     7
#define SCROLL_POT_PIN      3

// ---- Cursor tuning: adjust these after testing ----

struct DebouncedButton {
  uint8_t pin;
  bool stablePressed;
  bool lastReading;
  uint32_t lastChangeMs;
};


// Forward declarations
bool writeMpuRegister(uint8_t reg, uint8_t value);
bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz);
int readScrollPot();
bool mpuBegin();
void calibrateGyro();
void setupBleMouse();
void sendMouseReport(int8_t dx, int8_t dy, int8_t wheel = 0);
void sendKeyboardKey(uint8_t keyCode);
void releaseKeyboardKey();
void startBeepPattern(uint8_t beepCount, uint16_t onMs, uint16_t gapMs = 100);
void updateBuzzer();
bool updateButton(DebouncedButton &button);
float applyDeadzone(float value);

// Balanced motion: gentle turns remain precise; faster deliberate turns
// automatically cover more screen distance without an abrupt speed jump.
float GYRO_SENSITIVITY = 1.0f;
float HORIZONTAL_SENSITIVITY = 10.5f;
float VERTICAL_SENSITIVITY = 10.5f;
float DEADZONE = 2.5f;          // reject normal resting gyro noise so the pointer stays still; deliberate turns remain above this level
float SMOOTHING = 0.22f;        // smooth hand tremor while responding sooner to a deliberate turn
int MAX_CURSOR_SPEED = 24;      // allows faster intentional turns while retaining a safe flick cap
int POT_COUNTS_PER_SCROLL = 40;      // lower number = more scroll steps for the same knob turn
bool INVERT_SCROLL = false;            // set true if turning the knob clockwise scrolls the wrong way

// Default module orientation: MPU6050 Y axis controls mouse X; X controls mouse Y.
// Change SWAP_XY or either INVERT variable if your installed module faces another way.
bool INVERT_X = false;
bool INVERT_Y = true;
bool SWAP_XY = false;

const uint32_t BUTTON_FLASH_MS = 90;

// Active 3.3 V buzzer connected directly to an unused ESP32-C3 pin.
static const uint8_t BUZZER_PIN = 5;

static const uint8_t MPU_ADDRESS = 0x68;
static const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_REG_GYRO_XOUT_H = 0x43;
static const uint16_t CALIBRATION_SAMPLES = 1000;
static const uint32_t SAMPLE_INTERVAL_MS = 10;
static const uint32_t DEBOUNCE_MS = 25;

NimBLEServer *bleServer = nullptr;
NimBLEHIDDevice *hidDevice = nullptr;
NimBLECharacteristic *mouseInput = nullptr;
NimBLECharacteristic *keyboardInput = nullptr;
bool bleConnected = false;
// Do not restart advertising inside the BLE disconnect callback. A short delay
// gives Windows time to finish closing the old link before it sees the mouse again.
uint32_t advertisingRestartDueMs = 0;
const uint32_t ADVERTISING_RESTART_DELAY_MS = 750;
uint32_t buttonFlashUntil = 0;
uint8_t beepsRemaining = 0;
bool buzzerOn = false;
uint32_t buzzerDeadlineMs = 0;
uint16_t currentBeepOnMs = 80;
uint16_t currentBeepGapMs = 100;

float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
float filteredMouseX = 0.0f;
float filteredMouseY = 0.0f;
// Keep sub-pixel movement between samples so gentle turns do not disappear
// just because a single 10 ms sample rounds to zero.
float cursorRemainderX = 0.0f;
float cursorRemainderY = 0.0f;
int lastPotReading = 0;
int potScrollRemainder = 0;
int8_t lastMoveDirectionX = 0;
int8_t lastMoveDirectionY = 0;
bool returningToCenterX = false;
bool returningToCenterY = false;
uint8_t mouseButtons = 0;
uint32_t lastSampleMs = 0;
// Presentation keys are sent as complete, non-overlapping press/release events.
// This prevents a quick second button press from overwriting the first HID report.
bool keyboardKeyHeld = false;
// Keep presentation presses in order. This is a small FIFO queue rather than
// one shared "pending" key, so Previous and Next can never replace each other.
uint8_t keyboardQueue[4] = {0, 0, 0, 0};
uint8_t keyboardQueueHead = 0;
uint8_t keyboardQueueTail = 0;
uint8_t keyboardQueueCount = 0;
uint32_t keyboardReleaseDueMs = 0;
uint32_t keyboardNextPressDueMs = 0;
const uint32_t KEYBOARD_HOLD_MS = 120;
const uint32_t KEYBOARD_RELEASE_GAP_MS = 40;


DebouncedButton leftButton {LEFT_BUTTON, false, false, 0};
DebouncedButton rightButton {RIGHT_BUTTON, false, false, 0};
DebouncedButton pptPreviousButton {PPT_PREVIOUS_BUTTON, false, false, 0};
DebouncedButton pptNextButton {PPT_NEXT_BUTTON, false, false, 0};

class MouseServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    Serial.println("BLE connected.");
    bleConnected = true;
    advertisingRestartDueMs = 0;
    startBeepPattern(2, 80); // two short beeps
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
    // Do not immediately advertise from this callback: Windows may still be
    // closing its old HID link. Restarting after a short quiet period prevents
    // a rapid connect/disconnect loop.
    Serial.printf("BLE disconnected (reason %d). Waiting before advertising again.\n", reason);
    bleConnected = false;
    mouseButtons = 0;
    advertisingRestartDueMs = millis() + ADVERTISING_RESTART_DELAY_MS;
    startBeepPattern(1, 350); // one long beep
  }
};

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_REG_GYRO_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDRESS, (uint8_t)6) != 6) return false;

  gx = (int16_t)((Wire.read() << 8) | Wire.read());
  gy = (int16_t)((Wire.read() << 8) | Wire.read());
  gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

// Average several ADC conversions. This removes small electrical noise while
// preserving every gradual knob movement, so a slow turn still scrolls.
int readScrollPot() {
  uint32_t total = 0;
  for (uint8_t sample = 0; sample < 4; ++sample) {
    total += analogRead(SCROLL_POT_PIN);
  }
  return (int)(total / 4);
}

bool mpuBegin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(50);

  Wire.beginTransmission(MPU_ADDRESS);
  if (Wire.endTransmission() != 0) return false;

  // Wake the MPU6050. The default +/-250 degrees/second gyro range is retained.
  return writeMpuRegister(MPU_REG_PWR_MGMT_1, 0x00);
}

void calibrateGyro() {
  Serial.println("Keep the air mouse completely still: calibrating gyro...");
  int64_t sumX = 0, sumY = 0, sumZ = 0;
  uint16_t collected = 0;

  while (collected < CALIBRATION_SAMPLES) {
    int16_t rawX, rawY, rawZ;
    if (readGyroRaw(rawX, rawY, rawZ)) {
      sumX += rawX;
      sumY += rawY;
      sumZ += rawZ;
      ++collected;
    }
    delay(2);
  }

  gyroOffsetX = (float)sumX / CALIBRATION_SAMPLES;
  gyroOffsetY = (float)sumY / CALIBRATION_SAMPLES;
  gyroOffsetZ = (float)sumZ / CALIBRATION_SAMPLES;
  filteredMouseX = 0.0f;
  filteredMouseY = 0.0f;
  cursorRemainderX = 0.0f;
  cursorRemainderY = 0.0f;

  Serial.printf("Calibration complete. Offsets: X=%.2f Y=%.2f Z=%.2f\n",
                gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

void startBeepPattern(uint8_t beepCount, uint16_t onMs, uint16_t gapMs) {
  beepsRemaining = beepCount;
  currentBeepOnMs = onMs;
  currentBeepGapMs = gapMs;
  buzzerOn = true;
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerDeadlineMs = millis() + currentBeepOnMs;
}

// Non-blocking timing keeps gyro movement and BLE reports responsive while
// an active buzzer sounds its short status beeps.
void updateBuzzer() {
  if (beepsRemaining == 0 || millis() < buzzerDeadlineMs) return;
  if (buzzerOn) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    --beepsRemaining;
    buzzerDeadlineMs = millis() + currentBeepGapMs;
  } else if (beepsRemaining > 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
    buzzerDeadlineMs = millis() + currentBeepOnMs;
  }
}

void setupBleMouse() {
  // Windows expects a HID mouse/keyboard to bond so it can reconnect reliably.
  // No passkey is required: accept secure "Just Works" pairing.
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::init("ESP32 Air Mouse");
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new MouseServerCallbacks());

  hidDevice = new NimBLEHIDDevice(bleServer);
  mouseInput = hidDevice->getInputReport(1);
  keyboardInput = hidDevice->getInputReport(2);

  // One composite BLE HID device: report 1 is a 3-button mouse with wheel;
  // report 2 is a standard 6-key keyboard used for PowerPoint arrow keys.
  static const uint8_t reportMap[] = {
    0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x85, 0x01, 0x09, 0x01, 0xA1, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
    0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
    0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
    0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
    0xC0, 0xC0,
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x02,
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08,
    0x81, 0x01, 0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,
    0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0
  };

  // NimBLE-Arduino 2.x uses setter methods (not the older ESP32 BLE API names).
  hidDevice->setManufacturer("ESP32 Air Mouse");
  hidDevice->setPnp(0x02, 0xE502, 0xA111, 0x0100);
  hidDevice->setHidInfo(0x00, 0x01);
  hidDevice->setReportMap((uint8_t *)reportMap, sizeof(reportMap));
  hidDevice->startServices();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->setAppearance(0x03C2); // HID Mouse
  advertising->addServiceUUID(hidDevice->getHidService()->getUUID());
  advertising->start();
  bleConnected = false;
  Serial.println("BLE mouse advertising as: ESP32 Air Mouse");
}

void sendMouseReport(int8_t dx, int8_t dy, int8_t wheel) {
  if (bleServer->getConnectedCount() == 0) return;
  uint8_t report[4] = {mouseButtons, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel};
  mouseInput->setValue(report, sizeof(report));
  mouseInput->notify();
}

void sendKeyboardKey(uint8_t keyCode) {
  if (bleServer->getConnectedCount() == 0) return;

  // Store every event in order. A single pending value allowed one button's
  // key code to overwrite the other when presses happened close together.
  if (keyboardQueueCount < 4) {
    keyboardQueue[keyboardQueueTail] = keyCode;
    keyboardQueueTail = (keyboardQueueTail + 1) % 4;
    ++keyboardQueueCount;
  }
}

void releaseKeyboardKey() {
  uint32_t now = millis();

  if (keyboardKeyHeld && (int32_t)(now - keyboardReleaseDueMs) >= 0) {
    uint8_t released[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (bleServer->getConnectedCount() != 0) {
      keyboardInput->setValue(released, sizeof(released));
      keyboardInput->notify();
    }
    keyboardKeyHeld = false;
    keyboardNextPressDueMs = now + KEYBOARD_RELEASE_GAP_MS;
  }

  if (!keyboardKeyHeld && keyboardQueueCount > 0 &&
      (int32_t)(now - keyboardNextPressDueMs) >= 0 &&
      bleServer->getConnectedCount() != 0) {
    // Keyboard report: modifier, reserved byte, then six simultaneous key slots.
    uint8_t pressed[8] = {0, 0, keyboardQueue[keyboardQueueHead], 0, 0, 0, 0, 0};
    keyboardInput->setValue(pressed, sizeof(pressed));
    keyboardInput->notify();
    keyboardQueueHead = (keyboardQueueHead + 1) % 4;
    --keyboardQueueCount;
    keyboardKeyHeld = true;
    keyboardReleaseDueMs = now + KEYBOARD_HOLD_MS;
  }
}

bool updateButton(DebouncedButton &button) {
  bool pressedNow = digitalRead(button.pin) == LOW;
  uint32_t now = millis();
  if (pressedNow != button.lastReading) {
    button.lastReading = pressedNow;
    button.lastChangeMs = now;
  }
  if ((now - button.lastChangeMs) >= DEBOUNCE_MS &&
      button.stablePressed != button.lastReading) {
    button.stablePressed = button.lastReading;
    return true;
  }
  return false;
}

float applyDeadzone(float value) {
  return (fabsf(value) < DEADZONE) ? 0.0f : value;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LEFT_BUTTON, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(PPT_PREVIOUS_BUTTON, INPUT_PULLUP);
  pinMode(PPT_NEXT_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  analogReadResolution(12);
  // The potentiometer is powered from 3.3 V, so this input never receives 5 V.
  analogSetPinAttenuation(SCROLL_POT_PIN, ADC_11db);
  lastPotReading = readScrollPot();
  leftButton.lastReading = digitalRead(LEFT_BUTTON) == LOW;
  leftButton.stablePressed = leftButton.lastReading;
  rightButton.lastReading = digitalRead(RIGHT_BUTTON) == LOW;
  rightButton.stablePressed = rightButton.lastReading;
  pptPreviousButton.lastReading = digitalRead(PPT_PREVIOUS_BUTTON) == LOW;
  pptPreviousButton.stablePressed = pptPreviousButton.lastReading;
  pptNextButton.lastReading = digitalRead(PPT_NEXT_BUTTON) == LOW;
  pptNextButton.stablePressed = pptNextButton.lastReading;

  if (!mpuBegin()) {
    Serial.println("MPU6050 not found at 0x68. Check power, SDA/SCL, and AD0.");
    while (true) delay(1000);
  }
  calibrateGyro();
  startBeepPattern(1, 80); // one short power-on beep
  setupBleMouse();
}

void loop() {
  updateBuzzer();
  releaseKeyboardKey();

  // Restart advertising only once the former connection has had time to close.
  // This also lets the same paired Windows/Android device reconnect after a
  // power cycle without repeatedly resetting the BLE radio.
  if (!bleConnected && advertisingRestartDueMs != 0 &&
      (int32_t)(millis() - advertisingRestartDueMs) >= 0) {
    advertisingRestartDueMs = 0;
    NimBLEDevice::startAdvertising();
    Serial.println("BLE advertising restarted.");
  }

  bool buttonsChanged = false;
  if (updateButton(leftButton)) {
    if (leftButton.stablePressed) {
      mouseButtons |= 0x01;
      buttonFlashUntil = millis() + BUTTON_FLASH_MS;
    } else mouseButtons &= ~0x01;
    buttonsChanged = true;
  }
  if (updateButton(rightButton)) {
    if (rightButton.stablePressed) {
      mouseButtons |= 0x02;
      buttonFlashUntil = millis() + BUTTON_FLASH_MS;
    } else mouseButtons &= ~0x02;
    buttonsChanged = true;
  }
  if (updateButton(pptPreviousButton) && pptPreviousButton.stablePressed) {
    sendKeyboardKey(0x50); // HID usage: Left Arrow
    buttonFlashUntil = millis() + BUTTON_FLASH_MS;
    startBeepPattern(1, 80); // short beep: previous slide

  }
  if (updateButton(pptNextButton) && pptNextButton.stablePressed) {
    sendKeyboardKey(0x4F); // HID usage: Right Arrow
    buttonFlashUntil = millis() + BUTTON_FLASH_MS;
    startBeepPattern(1, 80); // short beep: next slide

  }

  uint32_t now = millis();
  if (buttonFlashUntil != 0 && now >= buttonFlashUntil) {
    buttonFlashUntil = 0;
  }
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) {
    if (buttonsChanged) sendMouseReport(0, 0);
    return;
  }

  float dtSeconds = (lastSampleMs == 0) ? (SAMPLE_INTERVAL_MS / 1000.0f)
                                        : ((now - lastSampleMs) / 1000.0f);
  lastSampleMs = now;

  int16_t rawX, rawY, rawZ;
  if (!readGyroRaw(rawX, rawY, rawZ)) return;

  // +/-250 dps uses 131 raw counts per degree/second.
  float gyroX = ((float)rawX - gyroOffsetX) / 131.0f;
  float gyroY = ((float)rawY - gyroOffsetY) / 131.0f;
  float gyroZ = ((float)rawZ - gyroOffsetZ) / 131.0f;

  gyroX = applyDeadzone(gyroX);
  gyroY = applyDeadzone(gyroY);
  gyroZ = applyDeadzone(gyroZ);

  float targetX = SWAP_XY ? gyroX : gyroY;
  float targetY = SWAP_XY ? gyroY : gyroX;
  if (INVERT_X) targetX = -targetX;
  if (INVERT_Y) targetY = -targetY;

  // A gyro's zero point slowly changes with temperature. When every axis is
  // quiet, very slowly learn that resting value so the cursor does not creep.
  // Movement above the dead zone is never used for this correction.
  bool motionIsQuiet = fabsf(gyroX) < DEADZONE &&
                       fabsf(gyroY) < DEADZONE &&
                       fabsf(gyroZ) < DEADZONE;
  if (motionIsQuiet) {
    filteredMouseX = 0.0f;
    filteredMouseY = 0.0f;
    cursorRemainderX = 0.0f;
    cursorRemainderY = 0.0f;
    targetX = 0.0f;
    targetY = 0.0f;
  } else {
    // Exponential smoothing prevents hand tremor from making the pointer jump.
    filteredMouseX += SMOOTHING * (targetX - filteredMouseX);
    filteredMouseY += SMOOTHING * (targetY - filteredMouseY);
  }

  // A mild response curve gives fine control near the centre and only a small
  // boost during a faster deliberate turn. It is capped for predictable motion.
  float xResponse = 1.0f + min(0.5f, fabsf(filteredMouseX) / 120.0f);
  float yResponse = 1.0f + min(0.5f, fabsf(filteredMouseY) / 120.0f);
  cursorRemainderX += filteredMouseX * GYRO_SENSITIVITY * HORIZONTAL_SENSITIVITY * xResponse * dtSeconds;
  cursorRemainderY += filteredMouseY * GYRO_SENSITIVITY * VERTICAL_SENSITIVITY * yResponse * dtSeconds;
  int dx = (int)truncf(cursorRemainderX);
  int dy = (int)truncf(cursorRemainderY);
  cursorRemainderX -= dx;
  cursorRemainderY -= dy;
  dx = constrain(dx, -MAX_CURSOR_SPEED, MAX_CURSOR_SPEED);
  dy = constrain(dy, -MAX_CURSOR_SPEED, MAX_CURSOR_SPEED);

  // The potentiometer is the scroll wheel: turning it sends wheel steps and
  // holding it still sends none. Do not discard small changes: a slow turn of a
  // knob produces many small ADC changes that must be accumulated.
  int potReading = readScrollPot();
  int potChange = potReading - lastPotReading;
  lastPotReading = potReading;
  if (INVERT_SCROLL) potChange = -potChange;

  potScrollRemainder += potChange;
  int wheel = potScrollRemainder / POT_COUNTS_PER_SCROLL;
  potScrollRemainder -= wheel * POT_COUNTS_PER_SCROLL;
  wheel = constrain(wheel, -8, 8);

  if (dx != 0 || dy != 0 || wheel != 0 || buttonsChanged) {
    sendMouseReport((int8_t)dx, (int8_t)dy, (int8_t)wheel);
  }
}