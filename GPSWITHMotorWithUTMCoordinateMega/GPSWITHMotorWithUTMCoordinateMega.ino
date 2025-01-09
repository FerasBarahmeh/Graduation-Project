#include <TinyGPSPlus.h>
// #include <SoftwareSerial.h>
// Easting: 252772.26	Northing: 252772.26	Zone: 36

// Easting: 252776.90	Northing: 252776.90	Zone: 36
// 

const int motorPin = 9; 
TinyGPSPlus gps;

const double a = 6378137.0;  
const double f = 1.0 / 298.257223563;  
const double k0 = 0.9996;  

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
  double C = (2 * f - f * f) * cos(latRad) * cos(latRad);
  double A = cos(latRad) * (lonRad - lonOriginRad);

  *easting = k0 * N * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + pow(T, 2)) * pow(A, 5) / 120);
  *northing = k0 * (N * (latRad - latRad) + N * (A + (1 - T + C) * pow(A, 3) / 6));
  *zone = zoneNumber;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("Mega up");
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
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());

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

      double range = 8;
      double  EstDifTar= easting - targetEasting;
      double NorDifTar = northing - targetNorthing; 

      Serial.print("Esting difference targer =  ");
      Serial.println(EstDifTar);
      Serial.print("northing difference targer =  ");
      Serial.println(NorDifTar);

      bool isValidEasit = (EstDifTar >= -range && EstDifTar <= range);  
      bool isValidNorth = (NorDifTar >= -range && NorDifTar <= range);

      Serial.print("bool isValidEasit :  ");
      Serial.print(isValidEasit);
                   
      Serial.print("   bool isValidNorth :  ");
      Serial.println(isValidNorth);

      if (isValidEasit && isValidNorth) {
        digitalWrite(motorPin, LOW);  
      } else {
        digitalWrite(motorPin, HIGH); 
      }

      Serial.print("Motor: ");
      Serial.println(digitalRead(motorPin));
    }
  }
}
