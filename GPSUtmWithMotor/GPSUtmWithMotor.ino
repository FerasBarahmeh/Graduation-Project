#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

const int motorPin = 9;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3);

const double a = 6378137.0;  // Semi-major axis of the Earth (WGS84)
const double f = 1.0 / 298.257223563;  // Flattening (WGS84)
const double k0 = 0.9996;  // Scale factor for UTM

double targetEasting = 0.0;
double targetNorthing = 0.0;

double degToRad(double deg) {
  return deg * (3.14159265358979323846 / 180.0);
}

void LLtoUTM(double lat, double lon, double *easting, double *northing, int *zone) {
  double latRad = degToRad(lat);
  double lonRad = degToRad(lon);

  int zoneNumber = floor((lon + 180.0) / 6.0) + 1;

  double lonOrigin = (zoneNumber - 1) * 6.0 - 180.0 + 3.0;
  double lonOriginRad = degToRad(lonOrigin);

  double N = a / sqrt(1 - (2 * f - f * f) * sin(latRad) * sin(latRad));
  double T = tan(latRad) * tan(latRad);
  double C = (2 * f - f * f) / (1 - (2 * f - f * f)) * cos(latRad) * cos(latRad);
  double A = cos(latRad) * (lonRad - lonOriginRad);

  double M = a * ((1 - f * (2 - f) / 4 - 3 * f * f * (1 - f) / 64 - 5 * f * f * f * (1 - f) / 256) * latRad
               - (3 * f * (1 - f) / 8 + 3 * f * f * (1 - f) / 32 + 45 * f * f * f * (1 - f) / 1024) * sin(2 * latRad)
               + (15 * f * f * (1 - f) / 256 + 45 * f * f * f * (1 - f) / 1024) * sin(4 * latRad)
               - (35 * f * f * f * (1 - f) / 3072) * sin(6 * latRad));

  *easting = k0 * N * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * (2 * f - f * f)) * pow(A, 5) / 120) + 500000.0;

  *northing = k0 * (M + N * tan(latRad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24
                  + (61 - 58 * T + T * T + 600 * C - 330 * (2 * f - f * f)) * pow(A, 6) / 720));

  if (lat < 0) {
    *northing += 10000000.0;  // False northing for southern hemisphere
  }

  *zone = zoneNumber;
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  pinMode(motorPin, OUTPUT);

  while (!Serial);

  Serial.println("Enter Target Easting:");
  while (Serial.available() == 0) {}
  targetEasting = Serial.parseFloat();
  Serial.print("Target Easting: ");
  Serial.println(targetEasting);
  while (Serial.available() > 0) {
    Serial.read();
  }

  Serial.println("Enter Target Northing:");
  while (Serial.available() == 0) {}
  targetNorthing = Serial.parseFloat();
  Serial.print("Target Northing: ");
  Serial.println(targetNorthing);
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      double latitude = gps.location.lat();
      double longitude = gps.location.lng();
      double altitude = gps.altitude.meters();

      Serial.print("Latitude: ");
      Serial.print(latitude, 6);
      Serial.print("°");

      Serial.print("\tLongitude: ");
      Serial.print(longitude, 6);
      Serial.print("°");

      Serial.print("\tAltitude: ");
      Serial.print(altitude);
      Serial.println(" m");

      double easting, northing;
      int zone;
      LLtoUTM(latitude, longitude, &easting, &northing, &zone);

      Serial.print("Easting: ");
      Serial.print(easting);
      Serial.print("\tNorthing: ");
      Serial.print(northing);
      Serial.print("\tZone: ");
      Serial.println(zone);

      double range = 20;
      double EstDifTar = easting - targetEasting;
      double NorDifTar = northing - targetNorthing;

      Serial.print("Easting difference to target: ");
      Serial.println(EstDifTar);
      Serial.print("Northing difference to target: ");
      Serial.println(NorDifTar);

      bool isValidEasting = (EstDifTar >= -range && EstDifTar <= range);
      bool isValidNorthing = (NorDifTar >= -range && NorDifTar <= range);

      if (isValidEasting && isValidNorthing) {
        digitalWrite(motorPin, LOW);  // Stop motor if in range
      } else {
        digitalWrite(motorPin, HIGH); // Activate motor if out of range
      }

      Serial.print("Motor: ");
      Serial.println(digitalRead(motorPin));
    }
  }
}
