#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include <RTClib.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <QueueArray.h>

// Pin definitions
#define LEFT_BUTTON_PIN 6
#define RIGHT_BUTTON_PIN 7
#define SOLENOID_PIN 4  // D4
#define SENSOR_PIN 8


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define INITIAL_ADDRESS 0

#define SMA_PERIOD_DAYS 3       // Change this to the number of days you want
#define READINGS_PER_DAY 8      // This stays constant as you're reading once per hour between 9 am to 5 pm
#define ONE_HOUR_IN_MS 3600000  // One hour in milliseconds



#define DISPLAY_MODE_DURATION 5000  // This is the duration for each display mode in milliseconds. Adjust this to change how long each mode is displayed.


Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);


// Watering durations in milliseconds
const unsigned long WATERING_DURATION = 60 * 60 * 1000L;      // 1 hour
const unsigned long TEST_WATERING_DURATION = 5 * 60 * 1000L;  // 5 minutes
bool isWatering = false;


class ValveController {
private:
  int valvePin;

public:
  ValveController(int pin)
    : valvePin(pin) {
    pinMode(valvePin, OUTPUT);
    digitalWrite(valvePin, LOW);  // Ensure the valve is closed to start
  }

  void openValve() {
    digitalWrite(valvePin, HIGH);
  }

  void closeValve() {
    digitalWrite(valvePin, LOW);
  }
};


class FlashMemoryController {
private:
  int initialAddress;  // The initial address in the EEPROM to read/write
  int address;         // Current address in EEPROM for read/write

public:
  FlashMemoryController(int a)
    : initialAddress(a), address(a) {}

  // Write a long to EEPROM
  void writeLong(unsigned long value) {
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
      EEPROM.write(address++, *p++);
    }
    address = initialAddress;  // Reset address after write
  }

  // Read a long from EEPROM
  unsigned long readLong() {
    unsigned long value = 0;
    byte* p = (byte*)(void*)&value;
    for (int i = 0; i < sizeof(value); i++) {
      *p++ = EEPROM.read(address++);
    }
    address = initialAddress;  // Reset address after read
    return value;
  }
};

class RTCController {
private:
  RTC_DS3231 rtc;

public:
  RTCController() {
    Serial.println("RTCController constructor entered");
    if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      while (1)
        ;
    }
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(__DATE__, __TIME__));
    }
    Serial.println("RTCController constructor completed");
  }

  String getCurrentDateAndTime() {
    DateTime now = rtc.now();
    char dateAndTime[17];
    sprintf(dateAndTime, "%02d/%02d/%02d %02d:%02d", now.day(), now.month(), now.year() % 100, now.hour(), now.minute());
    return String(dateAndTime);
  }

  bool isDaytime() {
    DateTime now = rtc.now();
    int hour = now.hour();
    return hour >= 9 && hour < 17;
  }

  DateTime getDateTime() {
    return rtc.now();
  }

  unsigned long getCurrentTimeMillis() {
    DateTime now = rtc.now();
    return now.unixtime() * 1000L;
  }
};

class RainSensor {
private:
  RTCController& rtcController;
  bool isRaining;
  unsigned long rainStartTime;
  unsigned long rainEndTime;
  unsigned long rainDuration;
  bool rainEventOverOneHour;  // Flag to indicate if the last rain event lasted longer than one hour

public:
  RainSensor(RTCController& rtc)
    : isRaining(false), rainStartTime(0), rainEndTime(0), rainDuration(0), rainEventOverOneHour(false), rtcController(rtc) {
    pinMode(SENSOR_PIN, INPUT);
  }

  void updateRainStatus() {
    // Read from rain sensor
    int rainValue = digitalRead(SENSOR_PIN);
    bool currentlyRaining = (rainValue == LOW);  // Rain detected when pin is HIGH

    // Check if a rain event has started or stopped
    if (!isRaining && currentlyRaining) {
      rainStartTime = rtcController.getCurrentTimeMillis();
      isRaining = true;
      rainEventOverOneHour = false;  // Reset the flag when a new rain event starts
    } else if (isRaining && !currentlyRaining) {
      rainEndTime = rtcController.getCurrentTimeMillis();
      rainDuration = rainEndTime - rainStartTime;
      // Check if rain event lasted longer than an hour
      if (rainDuration > 60 * 60 * 1000) {
        rainEventOverOneHour = true;  // Set the flag if the rain event lasted longer than one hour
      }
      isRaining = false;
    }
  }

  bool getRainStatus() {
    return isRaining;
  }

  unsigned long getRainDuration() {
    return rainDuration;
  }

  bool isRainEventOverOneHour() {
    return rainEventOverOneHour;
  }
};

class TemperatureController {
private:
  float temperatureSMA;                // Simple Moving Average of the temperatures
  QueueArray<float> lastTemperatures;  // Queue to store last temperature readings
  Adafruit_BME280 bme;
  unsigned long lastUpdateMillis;
  RTCController& rtcController;  // This instance of RTCController will be used to check the current time
  float currentTemperature;      // To store the current temperature value

public:
  TemperatureController(RTCController& rtcController)
    : lastUpdateMillis(0), rtcController(rtcController) {
    temperatureSMA = 0;
    currentTemperature = 0;
    // Initialize BME280 sensor
    if (!bme.begin(0x76)) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1)
        ;
    }
  }

  void updateTemperature() {
    // Get the current temperature from the BME280 sensor
    currentTemperature = bme.readTemperature();
  }

  void updateSMATemperature() {
    // Check if it's daytime and at least one hour has passed since last update
    if (rtcController.isDaytime() && (millis() - lastUpdateMillis >= ONE_HOUR_IN_MS)) {
      lastUpdateMillis = rtcController.getCurrentTimeMillis();
      currentTemperature = bme.readTemperature();

      if (lastTemperatures.count() == SMA_PERIOD_DAYS * READINGS_PER_DAY) {
        // Subtract the oldest temperature from SMA and remove it from the queue
        temperatureSMA -= lastTemperatures.front() / (float)(SMA_PERIOD_DAYS * READINGS_PER_DAY);
        lastTemperatures.pop();
      }

      // Add the new temperature to SMA and the queue
      temperatureSMA += currentTemperature / (float)(SMA_PERIOD_DAYS * READINGS_PER_DAY);
      lastTemperatures.enqueue(currentTemperature);

      Serial.println(temperatureSMA);
    }
  }

  float getTemperature() {
    return currentTemperature;
  }

  float getTemperatureSMA() {
    return temperatureSMA;
  }
};


class IrrigationSystem {
private:
  enum WateringType {
    WATERING_TYPE_WATERING,
    WATERING_TYPE_RAIN,
    WATERING_TYPE_OVERRIDE
  };

  ValveController& valveController;
  unsigned long lastWateringTime;
  unsigned long wateringEndTime;
  unsigned long wateringInterval;  // How often to water, in milliseconds
  TemperatureController& tempController;
  RainSensor& rainSensor;
  RTCController& rtcController;
  FlashMemoryController& flashMemoryController;
  const unsigned long wateringTime = 19 * 60 * 60 * 1000;  // 1930 in milliseconds
  WateringType lastWateringType;                           // Stores the type of the last watering event
  unsigned long nextWateringEvent;                         // next watering event

public:
  IrrigationSystem(ValveController& valve, TemperatureController& temp, RainSensor& rain, RTCController& rtcController, FlashMemoryController& flashMemoryController)
    : valveController(valve), wateringInterval(7 * 24 * 60 * 60 * 1000), wateringEndTime(0), tempController(temp), rainSensor(rain), rtcController(rtcController), flashMemoryController(flashMemoryController), nextWateringEvent(0) {
    lastWateringTime = flashMemoryController.readLong();  // Read the last watering time from the EEPROM
  }
  bool isWatering() {
    unsigned long currentTime = rtcController.getCurrentTimeMillis();
    return currentTime < wateringEndTime;
  }

  void checkWateringSchedule() {
    unsigned long currentTime = rtcController.getCurrentTimeMillis();
    DateTime now = rtcController.getDateTime();

    // If the current temperature is less than 5 degrees, do not water
    if (tempController.getTemperature() < 5.0) {
      return;
    }

    checkWateringIntervalForJuly(now);
    resetTimerIfRained();
    updateWateringIntervalBasedOnTemperature();

    // Check if it's time to water
    if (currentTime - lastWateringTime >= wateringInterval && abs(currentTime % (24 * 60 * 60 * 1000) - wateringTime) < 60 * 1000) {
      startWatering();
      wateringEndTime = currentTime + WATERING_DURATION;
    }

    // Stop watering if it's time
    if (rtcController.getCurrentTimeMillis() >= wateringEndTime) {
      stopWatering();
      // Reset the timer to avoid immediate re-watering after an override
      resetWateringTimer();
    }
  }

  void checkWateringIntervalForJuly(const DateTime& now) {
    // Water daily between July 7 and July 28
    if (now.month() == 7 && now.day() >= 7 && now.day() <= 28) {
      wateringInterval = 24 * 60 * 60 * 1000;
    }
  }

  void resetTimerIfRained() {
    // Reset timer if it has rained for over one hour
    if (rainSensor.isRainEventOverOneHour()) {
      resetWateringTimer();
    }
  }

  void updateWateringIntervalBasedOnTemperature() {
    float temperatureSMA = tempController.getTemperatureSMA();

    // Update watering interval based on temperature SMA
    if (temperatureSMA < 10.0) {
      wateringInterval = 0;  // Never water
    } else if (temperatureSMA < 18.0) {
      wateringInterval = 7 * 24 * 60 * 60 * 1000;
    } else if (temperatureSMA < 25.0) {
      wateringInterval = 4 * 24 * 60 * 60 * 1000;
    } else {
      wateringInterval = 2 * 24 * 60 * 60 * 1000;
    }
  }

  void startWatering() {
    valveController.openValve();
    lastWateringTime = rtcController.getCurrentTimeMillis();
    lastWateringType = WATERING_TYPE_WATERING;  // Update the watering type to Watering
  }

  void stopWatering() {
    valveController.closeValve();
    lastWateringTime = rtcController.getCurrentTimeMillis();
    flashMemoryController.writeLong(lastWateringTime);
  }

  void resetWateringTimer() {
    if (rainSensor.isRainEventOverOneHour()) {
      lastWateringTime = rtcController.getCurrentTimeMillis() - wateringInterval;
      lastWateringType = WATERING_TYPE_RAIN;  // Update the watering type to Rain
    }
  }

  void overrideWatering() {
    startWatering();
    wateringEndTime = rtcController.getCurrentTimeMillis() + WATERING_DURATION;  // Water for the full duration
    lastWateringType = WATERING_TYPE_OVERRIDE;                                   // Update the watering type to Override
  }

  // Modify this method to include the type of the last watering event
  void getLastWateringTimeAndType(char* buffer) {
    sprintf(buffer, "%lu %s", lastWateringTime, wateringTypeToString(lastWateringType));
  }

  void setWateringInterval(unsigned long interval) {
    wateringInterval = interval;
  }

  void calculateNextWateringEvent() {
    nextWateringEvent = lastWateringTime + wateringInterval;
  }

  void getNextWateringEvent(char* buffer) {
    sprintf(buffer, "%lu", nextWateringEvent);
  }

  void getCountdownToNextEvent(char* buffer) {
    unsigned long currentTime = rtcController.getCurrentTimeMillis();
    unsigned long timeToNextEvent = nextWateringEvent - currentTime;
    sprintf(buffer, "%lu", timeToNextEvent);
  }

  const char* wateringTypeToString(WateringType type) {
    switch (type) {
      case WATERING_TYPE_WATERING:
        return "Water";
      case WATERING_TYPE_RAIN:
        return "Rain";
      case WATERING_TYPE_OVERRIDE:
        return "Override";
      default:
        return "Unknown";
    }
  }
};


class LCDController {
private:
  LiquidCrystal_I2C lcd;
  int displayMode = 0;
  String lastPrintedTime = "";
  unsigned long lastModeChangeTime = 0;  // will hold the time when the display mode last changed
  IrrigationSystem& irrigationSystem;
  RTCController& rtcController;
  char lastPrintedContent[32];  // Buffer to store last printed content

public:
  LCDController(IrrigationSystem& system, RTCController& rtc)
    : lcd(0x27, 16, 2), irrigationSystem(system), rtcController(rtc) {
  }

  void begin() {
    lcd.init();
    lcd.backlight();
  }
  // Method to clear the LCD.
  void clear() {
    lcd.clear();
  }

  // Method to set the cursor position.
  void setCursor(int col, int row) {
    lcd.setCursor(col, row);
  }

  // Method to print a string to the LCD.
  void print(const String& s) {
    lcd.print(s);
  }


  // Update the LCD with the current status
  void updateDisplay(TemperatureController& tempController, IrrigationSystem& irrigationSystem, RTCController& rtc) {
    String currentTime = rtc.getCurrentDateAndTime();
    char newContent[32];  // Buffer for storing new content
    char buffer[16];      // Buffer for storing watering event information

    // Check if the time has changed since the last update
    if (currentTime != lastPrintedTime) {
      // Update the last printed time
      lastPrintedTime = currentTime;
    }

    // If the display mode duration has elapsed, update the display mode
    if (millis() - lastModeChangeTime >= DISPLAY_MODE_DURATION) {
      displayMode = (displayMode + 1) % 5;
      lastModeChangeTime = rtcController.getCurrentTimeMillis();
    }

    // Determine the new content based on the mode
    switch (displayMode) {
      case 0:
        sprintf(newContent, "T: %d", (int)tempController.getTemperature());
        break;
      case 1:
        sprintf(newContent, "SMA: %d", (int)tempController.getTemperatureSMA());
        break;
      case 2:
        irrigationSystem.getLastWateringTimeAndType(buffer);
        sprintf(newContent, "LW: %s", buffer);
        break;
      case 3:
        irrigationSystem.getNextWateringEvent(buffer);
        sprintf(newContent, "NW: %s", buffer);
        break;
      case 4:
        irrigationSystem.getCountdownToNextEvent(buffer);
        sprintf(newContent, "CT: %s", buffer);
        break;
    }

    // If the new content is different from the last printed content, update the LCD
    if (strcmp(newContent, lastPrintedContent) != 0) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(currentTime);
      lcd.setCursor(0, 1);
      lcd.print(newContent);
      strcpy(lastPrintedContent, newContent);
    }
  }
};

class TestButton {
private:
  OneButton button;
  IrrigationSystem& irrigationSystem;
  TemperatureController& tempController;
  RainSensor& rainSensor;
  LCDController& lcdController;
  RTCController& rtcController;
  unsigned long pressTime;
  int displayMode = 0;

public:
  TestButton(int pin, IrrigationSystem& system, TemperatureController& temp, RainSensor& rain, LCDController& lcd, RTCController& rtc)
    : button(pin, true, true), irrigationSystem(system), tempController(temp), rainSensor(rain), lcdController(lcd), rtcController(rtc) {
    button.attachClick(clickProxy, static_cast<void*>(this));
    button.attachLongPressStop(longPressStopProxy, static_cast<void*>(this));
  }

  void loop() {
    button.tick();

    if (irrigationSystem.isWatering()) {
      if (rtcController.getCurrentTimeMillis() - pressTime >= 5 * 60 * 1000) {
        irrigationSystem.stopWatering();
      } else {
        lcdController.clear();
        lcdController.setCursor(0, 0);
        lcdController.print("Test mode");
        lcdController.setCursor(0, 1);

        switch (displayMode) {
          case 0:
            lcdController.print("Temp: " + String(tempController.getTemperature()));
            break;
          case 1:
            lcdController.print("Rain: " + String(rainSensor.getRainStatus()));
            break;
        }

        displayMode = (displayMode + 1) % 2;
      }
    }
  }

private:
  static void clickProxy(void* arg) {
    auto* instance = static_cast<TestButton*>(arg);
    instance->onClick();
  }

  static void longPressStopProxy(void* arg) {
    auto* instance = static_cast<TestButton*>(arg);
    instance->onLongPressStop();
  }

  void onClick() {
    if (!irrigationSystem.isWatering()) {
      pressTime = rtcController.getCurrentTimeMillis();
      irrigationSystem.startWatering();
    }
  }

  void onLongPressStop() {
    if (irrigationSystem.isWatering()) {
      irrigationSystem.stopWatering();
    }
  }
};
class OverrideButton {
private:
  OneButton button;
  IrrigationSystem& irrigationSystem;

public:
  OverrideButton(int pin, IrrigationSystem& system)
    : button(pin, true, true), irrigationSystem(system) {
    button.attachClick(clickProxy, static_cast<void*>(this));
  }

  void loop() {
    button.tick();
  }

private:
  static void clickProxy(void* arg) {
    auto* instance = static_cast<OverrideButton*>(arg);
    instance->onClick();
  }

  void onClick() {
    if (!irrigationSystem.isWatering()) {
      irrigationSystem.startWatering();
    } else {
      irrigationSystem.stopWatering();
    }
  }
};



RTCController rtcController;
TemperatureController tempController(rtcController);
RainSensor rainSensor(rtcController);
ValveController valveController(SOLENOID_PIN);
FlashMemoryController flashMemoryController(INITIAL_ADDRESS);
IrrigationSystem irrigationSystem(valveController, tempController, rainSensor, rtcController, flashMemoryController);
LCDController lcdController(irrigationSystem, rtcController);
TestButton testButton(LEFT_BUTTON_PIN, irrigationSystem, tempController, rainSensor, lcdController, rtcController);
OverrideButton overrideButton(RIGHT_BUTTON_PIN, irrigationSystem);

void setup() {
  Serial.begin(9600);
  Serial.println("Serial communication started");

  // Initialize the BME280 sensor
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }
  Serial.println("BME280 sensor initialized");

  // Set the sensor pin as input
  pinMode(SENSOR_PIN, INPUT);
  Serial.println("Rain sensor pin set as input");

  // Use lcdController to initialize the LCD
  //lcdController.begin();

  //lcdController.print("Initializing...");
  //lcdController.clear();

  // Initialize solenoid valve
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);  // Ensure valve is off at start
}



void loop() {
 //testButton.loop();
//overrideButton.loop();
 // tempController.updateTemperature();
  //tempController.updateSMATemperature();
// rainSensor.updateRainStatus();
//irrigationSystem.checkWateringSchedule();
 // lcdController.updateDisplay(tempController, irrigationSystem, rtcController);
}
