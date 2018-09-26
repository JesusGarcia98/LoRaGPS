#include <TheThingsNetwork.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <math.h>

const char *appEui = "****************";
const char *appKey = "********************************";

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
    double latitude = GPS.latitudeDegrees * 1000000;
    double longitude = GPS.longitudeDegrees * 1000000;
    uint32_t gLat1 = latitude / 10000;
    uint32_t gLat2 = fmod(latitude, 10000);
    uint32_t gLon1 = longitude / 10000;
    uint32_t gLon2 = fmod(longitude, 10000);

    //    Serial.print("\nTime: ");
    //    Serial.print(gHour); Serial.print(' ');
    //    Serial.print("Date: ");
    //    Serial.print(gMonth); Serial.print("/");
    //    Serial.println(gYear);
    //
    //    if (GPS.fix) {
    //      Serial.print("Location: ");
    //      Serial.print(latitude); Serial.print(", "); Serial.println(longitude);
    //      Serial.print(gLat1); Serial.print(gLat2);
    //      Serial.print(", ");
    //      Serial.print(gLon1); Serial.print(gLon2);
    //    }

    byte payload[14];
    payload[0] = highByte(gHour);
    payload[1] = lowByte(gHour);
    payload[2] = highByte(gMonth);
    payload[3] = lowByte(gMonth);
    payload[4] = highByte(gYear);
    payload[5] = lowByte(gYear);
    payload[6] = highByte(gLat1);
    payload[7] = lowByte(gLat1);
    payload[8] = highByte(gLat2);
    payload[9] = lowByte(gLat2);
    payload[10] = highByte(gLon1);
    payload[11] = lowByte(gLon1);
    payload[12] = highByte(gLon2);
    payload[13] = lowByte(gLon2);

    ttn.sendBytes(payload, sizeof(payload));
  }
}
