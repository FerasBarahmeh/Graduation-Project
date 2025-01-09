/**
 * For B Motor
 */
int FPWM_Output_MB = 4;
int BPWM_Output_MB = 5;
int PB = 6;  // PWM to Motor B

/**
 * For A Motor (IBT Driver)
 */
int FPWM_Output_MA = 9;
int BPWM_Output_MA = 10;


int Feedback_pot_Y;
int Feedback_pot_map_Y;
const int deadband = 50;

int Feedback_pot_X;
int Feedback_pot_map_X;

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

  pinMode(PB, OUTPUT);  // Set PB as output

  pinMode(onPin, INPUT_PULLUP);
  pinMode(offPin, INPUT_PULLUP);
  Serial.begin(115200);
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

  analogWrite(PB, 0);
}

void turnOffForwardMotor() {
  analogWrite(FPWM_Output_MA, 0);
  analogWrite(FPWM_Output_MB, 0);

  analogWrite(PB, 0);
}

void setMotorsSpeed(byte speed) {
  analogWrite(BPWM_Output_MA, speed);
  analogWrite(BPWM_Output_MB, speed);

  analogWrite(PB, speed);
}



void loop() {
  setSystemStatus();

  if (!getSystemStatus()) turnOffMotor();

  if (getSystemStatus()) {
    // Read potentiometer and map its value
    Feedback_pot_Y = analogRead(A0);
    Feedback_pot_map_Y = map(Feedback_pot_Y, 0, 1023, 0, 255);

    Feedback_pot_X = analogRead(A1);
    Feedback_pot_map_X = map(Feedback_pot_X, 0, 1023, 0, 255);
    // Serial.print("Feedback_pot_map_X ");
    // Serial.println(Feedback_pot_map_X);

    /**
    * Convert Rigth range to be 255 as maximum
    */
    if (Feedback_pot_map_X < 132) {
      int Feedback_pot_map_X_right = map(Feedback_pot_map_X, 132, 0, 0, 255);
      Serial.print("Feedback_pot_map_X_right ");
      Serial.println(Feedback_pot_map_X_right);

      int speed = constrain(Feedback_pot_map_X_right, 0, 255);
      analogWrite(PB, speed);
      // analogWrite(PA, 0);

      analogWrite(BPWM_Output_MB, 0);
      analogWrite(FPWM_Output_MB, speed);


      analogWrite(FPWM_Output_MA, 0);
      analogWrite(FPWM_Output_MB, 0);
    }


    // Move Forward
    if (Feedback_pot_map_Y >= (128 + deadband)) {
      turnOffBackForwardMotor();

      int speed = map(Feedback_pot_map_Y, 128 + deadband, 255, 0, 255);
      speed = constrain(speed, 0, 255);
      setMotorsSpeed(speed);
      // Serial.print("Forward ");
      // Serial.println(speed);

    } else if (Feedback_pot_map_Y <= (128 - deadband)) {  // Move Back
      turnOffForwardMotor();

      int speed = map(Feedback_pot_map_Y, 128 - deadband, 0, 0, 255);
      speed = constrain(speed, 0, 255);
      setMotorsSpeed(speed);

      // Serial.print("Back ");
      // Serial.println(speed);
    } else {
      turnOffMotor();
      // Serial.print("Stop motor ");
      // Serial.println(Feedback_pot_map_Y);
    }
  }

  delay(50);
}