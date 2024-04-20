#include <

#define enA 5
#define in1 A1
#define in2 A2

#define enB 6
#define in3 A3
#define in4 A4

#define servoPin 9

void writePercent(float value) {
  if (value >= 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, value * 255);
    analogWrite(enB, value * 255);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, -value * 255);
    analogWrite(enB, -value * 255);
  }
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);

}

void loop() {
  // put your main code here, to run repeatedly

  writePercent(.7);

  // analogWrite(enA, 60.5);
  // analogWrite(enB, 255);
}
