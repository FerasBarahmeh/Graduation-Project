/**
* For B Motor
*/
int FPWM_Output_MB = 4;
int BPWM_Output_MB = 5;

/**
* For A Motor
*/
int FPWM_Output_MA = 7;
int BPWM_Output_MA = 8;


int Feedback_pot;
int Feedback_pot_map;

const int deadband = 50;

/**
 * Button configuration
 */
int onPin = 2;
int offPin = 3;
bool system_status = true;  // false = OFF, true = ON

void setup() {
  pinMode(FPWM_Output_MB, OUTPUT);
  pinMode(BPWM_Output_MB, OUTPUT);

  pinMode(FPWM_Output_MA, OUTPUT);
  pinMode(BPWM_Output_MA, OUTPUT);

  pinMode(onPin, INPUT_PULLUP);
  pinMode(offPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void turnOffMotor() {
  analogWrite(FPWM_Output_MB, 0);
  analogWrite(BPWM_Output_MB, 0);

  analogWrite(FPWM_Output_MA, 0);
  analogWrite(BPWM_Output_MA, 0);
}

void setSystemStatus() {
  if (digitalRead(onPin) == LOW) {
    system_status = true;
    Serial.println("System turned ON");
    delay(200);
  }

  if (digitalRead(offPin) == LOW) {
    system_status = false;
    Serial.println("System turned OFF");
    delay(200);
  }
}

bool getSystemStatus() {
  return system_status;
}

void turnOffBackForwardMotor() {
  analogWrite(BPWM_Output_MA, 0);
  analogWrite(BPWM_Output_MB, 0);
}

void turnOffForwardMotor() {
  analogWrite(FPWM_Output_MA, 0);
  analogWrite(FPWM_Output_MB, 0);
}

void setMotorsSpeed(byte speed) {
  analogWrite(BPWM_Output_MA, speed);
  analogWrite(BPWM_Output_MB, speed);
}

void loop() {
  setSystemStatus();

  if (!getSystemStatus()) turnOffMotor();

  if (getSystemStatus()) {
    // Read potentiometer and map its value
    Feedback_pot = analogRead(A0);
    Feedback_pot_map = map(Feedback_pot, 0, 1023, 0, 255);

    // Move Foroward
    if (Feedback_pot_map >= (128 + deadband)) {
      turnOffBackForwardMotor();

      int speed = map(Feedback_pot_map, 128 + deadband, 255, 0, 255);
      speed = constrain(speed, 0, 255);
      setMotorsSpeed(speed);
      Serial.print("Forward ");
      Serial.println(speed);

    } else if (Feedback_pot_map <= (128 - deadband)) {  // Move Back
      turnOffForwardMotor();

      int speed = map(Feedback_pot_map, 128 - deadband, 0, 0, 255);
      speed = constrain(speed, 0, 255);
      setMotorsSpeed(speed);

      Serial.print("Back ");
      Serial.println(speed);
    } else {
      turnOffMotor();
      Serial.print("Stop motor ");
      Serial.println(Feedback_pot_map);
    }
  }

  delay(50);
}