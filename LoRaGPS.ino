#include <TheThingsNetwork.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

const char *appEui = "7***************";
const char *appKey = "******************************";

// Connect the GPS Power pin to 3.3V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 8
//   Connect the GPS RX (receive) pin to Digital 7

#define loraSerial Serial1
#define freqPlan TTN_FP_EU868
#define GPSECHO  false

TheThingsNetwork ttn(loraSerial, Serial, freqPlan);
SoftwareSerial mySerial(8, 7); //TX, RX
Adafruit_GPS GPS(&mySerial);

void setup() {
  Serial.begin(115200);
  loraSerial.begin(57600);
  delay(5000);
  Serial.println("Adafruit GPS and LoRa!");

  Serial.println("-- STATUS");
  ttn.showStatus();

  Serial.println("-- JOIN");
  ttn.join(appEui, appKey);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
}

uint32_t timer = millis();

void loop() {
  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (timer > millis())  timer = millis();

  if (millis() - timer > 10000) {
    timer = millis();

    uint32_t gHour = GPS.hour; // Add 2 hours
    uint32_t gMonth = GPS.month;
    uint32_t gYear = GPS.year; // Add 2000 years
    uint32_t gLat = GPS.latitude * 10000;
    uint32_t gLon = GPS.longitude * 10000;

    //        Serial.print("\nTime: ");
    //        Serial.print(gHour); Serial.print(' ');
    //        Serial.print("Date: ");
    //        Serial.print(gMonth); Serial.print("/");
    //        Serial.println(gYear);
    //
    //        if (GPS.fix) {
    //          Serial.print("Location: ");
    //          Serial.print(gLat); Serial.print(GPS.lat);
    //          Serial.print(", ");
    //          Serial.print(gLon); Serial.println(GPS.lon);
    //        }

    byte payload[10];
    payload[0] = highByte(gHour);
    payload[1] = lowByte(gHour);
    payload[2] = highByte(gMonth);
    payload[3] = lowByte(gMonth);
    payload[4] = highByte(gYear);
    payload[5] = lowByte(gYear);
    payload[6] = highByte(gLat);
    payload[7] = lowByte(gLat);
    payload[8] = highByte(gLon);
    payload[9] = lowByte(gLon);

    ttn.sendBytes(payload, sizeof(payload));
  }
}
