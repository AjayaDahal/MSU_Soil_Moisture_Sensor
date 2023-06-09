#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <esp_sleep.h>

#define SD_CS_PIN 5
#define RTC_INTERRUPT_PIN 34
#define ZIGBEE_TX_PIN 16
#define ZIGBEE_RX_PIN 17
#define MEASUREMENT_INTERVAL 1800000

File dataFile;
RTC_DS3231 rtc;
SoftwareSerial zigbeeSerial(ZIGBEE_RX_PIN, ZIGBEE_TX_PIN);

void setup() {
  Serial.begin(9600);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  rtc.enableInterrupts();
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);

  zigbeeSerial.begin(9600, SWSERIAL_8N1, ZIGBEE_RX_PIN, ZIGBEE_TX_PIN, false, 256);

  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), wakeUp, FALLING);

  DateTime now = rtc.now();
  Serial.print("Current time: ");
  printDateTime(now);

  sleep();
}

void loop() {
  // Sleep and wait for interrupt
}

void wakeUp() {
  int soilMoisture = readSoilMoisture();
  saveDataToSD(soilMoisture);
  transmitDataViaZigbee(soilMoisture);
  
  delay(MEASUREMENT_INTERVAL);
  sleep();
}

void sleep() {
  detachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN));

  esp_sleep_enable_ext0_wakeup((gpio_num_t)RTC_INTERRUPT_PIN, LOW);
  esp_deep_sleep_start();

  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), wakeUp, FALLING);
}

int readSoilMoisture() {
  // SD12 protocol implementation for Terros 12 soil sensor
  Serial1.begin(9600); // Assuming the sensor is connected to Serial1

  byte cmd[] = {0x7E, 0x00, 0x10, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7E};

  Serial1.write(cmd, sizeof(cmd));

  byte response[10];
  int timeout = 1000; // Timeout in milliseconds
  unsigned long start = millis();
  int i = 0;

  while ((millis() - start) < timeout) {
    if (Serial1.available()) {
      response[i++] = Serial1.read();
      if (i >= sizeof(response)) {
        break;
      }
    }
  }

  int soilMoisture = (response[7] << 8) | response[8];

  Serial1.end(); // Release Serial1

  return soilMoisture;
}

void saveDataToSD(int soilMoisture) {
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(soilMoisture);
    dataFile.close();
    Serial.println("Data saved to SD card.");
  } else {
    Serial.println("Error opening data.txt file on SD card.");
  }
}

void transmitDataViaZigbee(int soilMoisture) {
  zigbeeSerial.println(soilMoisture);
  Serial.println("Data transmitted via Zigbee.");
}

void printDateTime(DateTime dt) {
  Serial.print(dt.year(), DEC);
  Serial.print('/');
  Serial.print(dt.month(), DEC);
  Serial.print('/');
  Serial.print(dt.day(), DEC);
  Serial.print(' ');
  Serial.print(dt.hour(), DEC);
  Serial.print(':');
  Serial.print(dt.minute(), DEC);
  Serial.print(':');
  Serial.print(dt.second(), DEC);
  Serial.println();
}
