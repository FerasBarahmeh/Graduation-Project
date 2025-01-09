// Pin definitions
#define A0 3  // Motor A Direction 1
#define A1 4  // Motor A Direction 2
#define PWM_A 5 // Motor A PWM
#define B0 6  // Motor B Direction 1
#define B1 7  // Motor B Direction 2
#define PWM_B 8 // Motor B PWM

void setup() {
  // Set control pins as output
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(B0, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(PWM_B, OUTPUT);
}

void loop() {
  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);
  analogWrite(PWM_A, 255);

  analogWrite(PWM_A, 0);
 
  digitalWrite(B0, HIGH);
  digitalWrite(B1, LOW);
  analogWrite(PWM_B, 255);

  analogWrite(PWM_B, 0);
}
