#include <Servo.h>

Servo myServo; 
const int IO1 = 10; 
const int IO2 = 11;

void setup() {
  Serial.begin(9600); 
  while (!Serial);

  myServo.attach(10); 
  pinMode(IO1, INPUT); 
  pinMode(IO2, INPUT);
  Serial.println("SpeakUp Click Test Started.");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    checkSpeakUpCommand(command);
    delay(100); 
  }

  int stateIO1 = digitalRead(IO1);
  int stateIO2 = digitalRead(IO2);
  
  Serial.print("IO1 State: ");
  Serial.println(stateIO1 ? "HIGH" : "LOW");
  Serial.print("IO2 State: ");
  Serial.println(stateIO2 ? "HIGH" : "LOW");

  delay(1000);
}

void checkSpeakUpCommand(String command) {
  command.trim();

  if (command.equalsIgnoreCase("right")) {
    Serial.println("Command 'right' recognized.");
    Serial.println("Value: right");
  } 
  else if (command.equalsIgnoreCase("left")) {
    Serial.println("Command 'left' recognized.");
    Serial.println("Value: left");
  } 
  else if (command.equalsIgnoreCase("stop")) {
    Serial.println("Command 'stop' recognized.");
    Serial.println("Value: stop");
  } 
  else {
    Serial.println("Unknown command or SpeakUp Click not responding.");
  }
}
