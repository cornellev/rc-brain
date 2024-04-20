#define enA 10
#define in1 11
#define in2 12

#define enB 6
#define in3 9
#define in4 8

void writeLeft(float value) {
  analogWrite(enB, value * 255);
}

void writeRight(float value) {
  analogWrite(enA, value * 255);
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

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(enA, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(enB, LOW);

}

void loop() {
  // put your main code here, to run repeatedly

  writeLeft(.3);
  writeRight(.3);

  // analogWrite(enA, 60.5);
  // analogWrite(enB, 255);
}
