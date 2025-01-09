int FPWM_Output = 4;
int BPWM_Output = 5;
int Feedback_pot;
int Feedback_pot_map;
const int deadband = 50;

/**
 * Button configuration
 */
int onPin = 8;
int offPin = 9;
bool system_status = false; // false = OFF, true = ON

void setup() {
  pinMode(FPWM_Output, OUTPUT);
  pinMode(BPWM_Output, OUTPUT);
  pinMode(onPin, INPUT_PULLUP);
  pinMode(offPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() { 
  if
   (digitalRead(onPin) == LOW) {
    system_status = true;
    Serial.println("System turned ON");
    delay(200);
  }

  if (digitalRead(offPin) == LOW) {
    system_status = false;
    Serial.println("System turned OFF");
    delay(200);
  }

  if (system_status) {
    // Read potentiometer and map its value
    Feedback_pot = analogRead(A0);
    Feedback_pot_map = map(Feedback_pot, 0, 1023, 0, 255);

    // Motor control logic
    if (Feedback_pot_map >= (128 + deadband)) {
      analogWrite(BPWM_Output, 0);
      int speed = map(Feedback_pot_map, 128 + deadband, 255, 0, 255);
      speed = constrain(speed, 0, 255);
      analogWrite(FPWM_Output, speed);
      Serial.print("Forward ");
      Serial.println(speed);
      
    } else if (Feedback_pot_map <= (128 - deadband)) {
      analogWrite(FPWM_Output, 0);
      int speed = map(Feedback_pot_map, 128 - deadband, 0, 0, 255);
      speed = constrain(speed, 0, 255);
      analogWrite(BPWM_Output, speed);
      Serial.print("Back ");
      Serial.println(speed);
    } else {
      analogWrite(FPWM_Output, 0);
      analogWrite(BPWM_Output, 0);
      Serial.print("Stop motor ");
      Serial.println(Feedback_pot_map);
    }
  } else {
    analogWrite(FPWM_Output, 0);
    analogWrite(BPWM_Output, 0);
  }

  delay(50);
}