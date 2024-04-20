#define enA 9
#define in1 10
#define in2 11

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(enA, HIGH);
  Serial.write("hello");

}
