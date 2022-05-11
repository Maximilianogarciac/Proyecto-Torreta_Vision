#include <Servo.h>
Servo ServoC;
Servo ServoB;
#define led 8
int SC = 0, SB = 0;
int pos, P1 = 90, P2 = 90;
int val1, val2, L;
String cad, cad1, cad2;
String inputString = "";
bool stringComplete = false;


void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
  delay(10);
  ServoC.attach(5);
  ServoB.attach(6);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  ServoC.write(P1);
  ServoB.write(P2);

}

void loop() {
  if (stringComplete) {
    int idx = inputString.indexOf(',');
    val1 = atoi(inputString.substring(0, idx).c_str());
    int idx2 = inputString.indexOf(',', idx+1);
    val2 = atoi(inputString.substring(idx + 1, idx2).c_str());
    int idx3 = inputString.indexOf(';', idx2+1);
    int L = atoi(inputString.substring(idx2 + 1, idx3).c_str());

    if ( L == 1) {
      laser();
    } else {
      digitalWrite(led, HIGH);
    }
    
    P1 = P1 + val1;
    P2 = P2 + val2;

    if (P1 >= 180) {
      P1 = 180;
    } else if (P1 <= 0) {
      P1 = 0;
    }
    if (P2 >= 180) {
      P2 = 180;
    } else if (P2 <= 0) {
      P2 = 0;
    }
    ServoC.write(P1);
    ServoB.write(P2);
    Serial.println(val1);
    Serial.println(val2);
    Serial.println(L);
    
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == ';') {
      stringComplete = true;
    }
  }
}

void laser() {
  digitalWrite(led, LOW);
  delay(10);
  digitalWrite(led, HIGH);
  delay(10);

}
