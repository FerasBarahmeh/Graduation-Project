#include <TinyGPSPlus.h>
TinyGPSPlus gps;
const double a = 6378137.0;
const double f = 1.0 / 298.257223563;
const double k0 = 0.9996;

double targetEasting = 0.0;
double targetNorthing = 0.0;

double range = 0.3;



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




/**
 * For B Motor
 */
int FPWM_Output_MB = 4;
int BPWM_Output_MB = 5;
int PB = 6;  // PWM to Motor B

/**
 * For A Motor (IBT Driver)
 */
int FPWM_Output_MA = 10;
int BPWM_Output_MA = 9;



int Feedback_pot_x;
int Feedback_pot_map_x;
int Feedback_pot_y;
int Feedback_pot_map_y;
const int deadband = 50;

/**
 * Button configuration
 */
int onPin = 2;
int offPin = 3;
int GPSoff = 11;
int GPSon = 12;

bool system_status = true;  // false = OFF, true = ON
bool gps_status = false;
/**
* Voice Configuration
*/
const int IO1 = 7;
const int IO2 = 8;

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);

  while (!Serial)
    ;

  while (gps.location.isUpdated() == false) {
    if (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  }
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();

  double easting, northing;
  int zone;
  LLtoUTM(latitude, longitude, &easting, &northing, &zone);

  targetEasting = easting;
  targetNorthing = northing;

  Serial.print("Target Easting: ");
  Serial.println(targetEasting);
  Serial.print("Target Northing: ");
  Serial.println(targetNorthing);

  pinMode(FPWM_Output_MB, OUTPUT);
  pinMode(BPWM_Output_MB, OUTPUT);

  pinMode(FPWM_Output_MA, OUTPUT);
  pinMode(BPWM_Output_MA, OUTPUT);

  pinMode(PB, OUTPUT);  // Set PB as output

  pinMode(onPin, INPUT_PULLUP);
  pinMode(offPin, INPUT_PULLUP);

  pinMode(IO1, INPUT);
  pinMode(IO2, INPUT);
}

void turnOffMotor() {
  analogWrite(FPWM_Output_MB, 0);
  analogWrite(BPWM_Output_MB, 0);

  analogWrite(FPWM_Output_MA, 0);
  analogWrite(BPWM_Output_MA, 0);

  analogWrite(PB, 0);
}

void setSystemStatus() {
  if (digitalRead(onPin) == LOW) {
    system_status = true;
    // Serial.println("System turned ON");
    delay(200);
  }

  if (digitalRead(offPin) == LOW) {
    system_status = false;
    // Serial.println("System turned OFF");
    delay(200);
  }
}

void setgpsStatus() {
  if (digitalRead(GPSon) == LOW) {
    gps_status = true;
    // Serial.println("System turned ON");
    delay(200);
  }

  if (digitalRead(GPSoff) == LOW) {
    gps_status = false;
    // Serial.println("System turned OFF");
    delay(200);
  }
}


bool getSystemStatus() {
  return system_status;
}

bool getgpsStatus() {
  return gps_status;
}

void turnOffBackForwardMotor() {
  analogWrite(BPWM_Output_MA, 0);
  analogWrite(BPWM_Output_MB, 0);

  analogWrite(PB, 0);
}

void turnOffForwardMotor() {
  analogWrite(FPWM_Output_MA, 0);
  analogWrite(FPWM_Output_MB, 0);

  analogWrite(PB, 0);
}

void setMotorsSpeedF(byte speed) {
  analogWrite(FPWM_Output_MB, speed);
  analogWrite(FPWM_Output_MA, speed);

  analogWrite(PB, 255);

  analogWrite(BPWM_Output_MB, 0);
  analogWrite(BPWM_Output_MA, 0);

  Serial.println("Forward");
}

void setMotorsSpeedB(byte speed) {
  analogWrite(BPWM_Output_MB, speed);
  analogWrite(BPWM_Output_MA, speed);

  analogWrite(PB, 255);

  analogWrite(FPWM_Output_MB, 0);
  analogWrite(FPWM_Output_MA, 0);

  Serial.println("Backward");
}


void setMotorsSpeedRightF(byte speed) {
  analogWrite(FPWM_Output_MB, speed);
  analogWrite(FPWM_Output_MA, speed * 0.5);

  analogWrite(PB, 255);
  analogWrite(BPWM_Output_MB, 0);
  analogWrite(BPWM_Output_MA, 0);

  Serial.println("Forward And Right");
}

void setMotorsSpeedRightB(byte speed) {
  analogWrite(BPWM_Output_MB, speed);
  analogWrite(BPWM_Output_MA, speed * 0.5);

  analogWrite(PB, 255);

  analogWrite(FPWM_Output_MB, 0);
  analogWrite(FPWM_Output_MA, 0);

  Serial.println("Backward And Right");
}

void setMotorsSpeedleftB(byte speed) {
  analogWrite(BPWM_Output_MB, speed * 0.5);
  analogWrite(BPWM_Output_MA, speed);

  analogWrite(PB, 255);

  analogWrite(FPWM_Output_MB, 0);
  analogWrite(FPWM_Output_MA, 0);

  Serial.println("Backward And Left");
}

void setMotorsSpeedleftF(byte speed) {
  analogWrite(FPWM_Output_MB, speed * 0.5);
  analogWrite(FPWM_Output_MA, speed);

  analogWrite(PB, 255);

  analogWrite(BPWM_Output_MB, 0);
  analogWrite(BPWM_Output_MA, 0);

  Serial.println("Forward And Left");
}




/* ...........................loop.......................*/
void loop() {

  setSystemStatus();
  setgpsStatus();


  int stateIO1 = digitalRead(IO1);
  int stateIO2 = digitalRead(IO2);



  if (stateIO1 && stateIO2) {
    Serial.println("                          ^");
    setMotorsSpeedF(100);
    // system_status = true;

    delay(2000);


  } else if (!stateIO1 && !stateIO2) {
    turnOffBackForwardMotor();
    turnOffForwardMotor();
    Serial.println("                      .");
    // system_status = false;


  } else if (stateIO1 && !stateIO2) {
    analogWrite(FPWM_Output_MB, 100);
    analogWrite(FPWM_Output_MA, 50);

    analogWrite(PB, 255);

    analogWrite(BPWM_Output_MB, 0);
    analogWrite(BPWM_Output_MA, 0);
    Serial.println("                    >");
    delay(2000);
  } else if (!stateIO1 && stateIO2) {
    analogWrite(FPWM_Output_MB, 50);
    analogWrite(FPWM_Output_MA, 100);

    analogWrite(PB, 255);

    analogWrite(BPWM_Output_MB, 0);
    analogWrite(BPWM_Output_MA, 0);
    Serial.println("                       <");
    delay(2000);
  }


  if (!getSystemStatus()) turnOffMotor();

  if (getSystemStatus()) {
    // Read potentiometer and map its value
    Feedback_pot_y = analogRead(A0);
    Feedback_pot_map_y = map(Feedback_pot_y, 0, 1023, 0, 255);

    Feedback_pot_x = analogRead(A1);
    Feedback_pot_map_x = map(Feedback_pot_x, 0, 1023, 0, 255);

    //Move Forward
    if (Feedback_pot_map_y >= (128 + deadband)) {
      turnOffBackForwardMotor();

      int speed = map(Feedback_pot_map_y, 128 + deadband, 255, 0, 255);
      speed = constrain(speed, 0, 255);
      // Move righ
      if (Feedback_pot_map_x >= (128 + deadband)) {
        int speed = map(Feedback_pot_map_x, 128 + deadband, 255, 0, 255);
        speed = constrain(speed, 0, 255);
        setMotorsSpeedRightF(speed);
        // Serial.println(speed);
      }

      // Move left
      else if (Feedback_pot_map_x <= (128 - deadband)) {
        turnOffForwardMotor();

        int speed = map(Feedback_pot_map_x, 128 - deadband, 0, 0, 255);
        speed = constrain(speed, 0, 255);
        setMotorsSpeedleftF(speed);
        // Serial.print("Back ");
        // Serial.println(speed);
      } else {
        setMotorsSpeedF(speed);
        // Serial.print("Stop motor ");
        // Serial.println(Feedback_pot_map_x);
      }
      setMotorsSpeedF(speed);
      //  setMotorsSpeed(speed);
      // Serial.print("Forward ");
      // Serial.println(speed);

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move Back
    else if (Feedback_pot_map_y <= (128 - deadband)) {
      turnOffForwardMotor();

      int speed = map(Feedback_pot_map_y, 128 - deadband, 0, 0, 255);
      speed = constrain(speed, 0, 255);


      // Move righ
      if (Feedback_pot_map_x >= (128 + deadband)) {

        int speed = map(Feedback_pot_map_x, 128 + deadband, 255, 0, 255);
        speed = constrain(speed, 0, 255);
        setMotorsSpeedRightB(speed);
        // Serial.print("Forward ");
        // Serial.println(speed);

      }

      // Move left
      else if (Feedback_pot_map_x <= (128 - deadband)) {
        turnOffForwardMotor();

        int speed = map(Feedback_pot_map_x, 128 - deadband, 0, 0, 255);
        speed = constrain(speed, 0, 255);
        setMotorsSpeedB(speed);
        setMotorsSpeedleftB(speed);

        // Serial.print("Back ");
        // Serial.println(speed);
      } else {
        setMotorsSpeedB(speed);
        // Serial.print("Stop motor ");
        // Serial.println(Feedback_pot_map_x);
      }
      setMotorsSpeedB(speed);
      // Serial.print("Back ");
      // Serial.println(speed);
    } else {
      turnOffMotor();
      // Serial.print("Stop motor ");
      // Serial.println(Feedback_pot_map_y);

      // Move righ
      if (Feedback_pot_map_x >= (128 + deadband)) {

        int speed = map(Feedback_pot_map_x, 128 + deadband, 255, 0, 255);
        speed = constrain(speed, 0, 255);
        setMotorsSpeedRightF(speed);
        // Serial.print("Forward ");
        // Serial.println(speed);

      }

      // Move left
      else if (Feedback_pot_map_x <= (128 - deadband)) {
        turnOffForwardMotor();

        int speed = map(Feedback_pot_map_x, 128 - deadband, 0, 0, 255);
        speed = constrain(speed, 0, 255);
        setMotorsSpeedleftF(speed);
        // Serial.print("Back ");
        // Serial.println(speed);
      }
    }
  }

  if (!getgpsStatus()) NULL;
  else if (getgpsStatus()) {

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

        double EstDifTar = easting - targetEasting;
        double NorDifTar = northing - targetNorthing;

        Serial.print("Easting difference target = ");
        Serial.println(EstDifTar);
        Serial.print("Northing difference target = ");
        Serial.println(NorDifTar);

        bool isValidEasit = (EstDifTar >= -range && EstDifTar <= range);
        bool isValidNorth = (NorDifTar >= -range && NorDifTar <= range);

        Serial.print("bool isValidEasit: ");
        Serial.print(isValidEasit);

        Serial.print("   bool isValidNorth: ");
        Serial.println(isValidNorth);


        if (isValidEasit && isValidNorth) {
          turnOffForwardMotor();
          turnOffBackForwardMotor();
          Serial.print("Motor A : ");
          Serial.println(digitalRead(FPWM_Output_MA));
          Serial.print("Motor B : ");
          Serial.println(digitalRead(FPWM_Output_MB));

        } else {
          setMotorsSpeedF(255);
          Serial.print("Motor A : ");
          Serial.println(digitalRead(FPWM_Output_MA));
          Serial.print("Motor B : ");
          Serial.println(digitalRead(FPWM_Output_MB));
          delay(3000);
        }
      }
    }
  }

  delay(50);
}
