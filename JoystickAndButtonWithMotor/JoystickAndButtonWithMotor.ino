/*
Dr. Abdel Ilah Alshbatat 20/11/2023
BTS9760 
IBT-2 pin 1 (RPWM) to Arduino Nano pin 3 (PWM)
IBT-2 pin 2 (LPWM) to Arduino Nano pin 5 (PWM)
IBT-2 pins 3 (R_EN), 4 (L_EN), to 5V power supply
IBT-2 pin 8 (GND) to GND not connected
IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected

ledPin = 13
Pot.   = A0
*/

int RPWM_Output = 3;
int LPWM_Output = 5;
const byte ledPin = 13;

int Feedback_pot;
int Feedback_pot_map;
const int deadband = 50;

/**
 * Button configuration
 */
int onPin = 2;
int offPin = 4;
bool system_status = false; // false = OFF, true = ON

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  pinMode(onPin, INPUT_PULLUP);
  pinMode(offPin, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
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

  if (system_status) {
    // Read potentiometer and map its value
    Feedback_pot = analogRead(A0);
    Feedback_pot_map = map(Feedback_pot, 0, 1023, 0, 255);
    Serial.println(Feedback_pot_map);

    // Motor control logic
    if (Feedback_pot_map >= (128 + deadband)) {
      analogWrite(LPWM_Output, 0);
      int speed = map(Feedback_pot_map, 128 + deadband, 255, 0, 255);
      speed = constrain(speed, 0, 255);
      analogWrite(RPWM_Output, speed);
      digitalWrite(ledPin, HIGH);
    } else if (Feedback_pot_map <= (128 - deadband)) {
      analogWrite(RPWM_Output, 0);
      int speed = map(Feedback_pot_map, 128 - deadband, 0, 0, 255);
      speed = constrain(speed, 0, 255);
      analogWrite(LPWM_Output, speed);
      digitalWrite(ledPin, LOW);
    } else {
      analogWrite(RPWM_Output, 0);
      analogWrite(LPWM_Output, 0);
      digitalWrite(ledPin, LOW);
    }
  } else {
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, 0);
    digitalWrite(ledPin, LOW);
  }

  delay(50);
}