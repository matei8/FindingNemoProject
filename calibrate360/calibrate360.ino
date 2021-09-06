#include <EEPROM.h>

#define TRIG_PIN D8
#define ECHO_PIN D10

#define TIME360 0
#define LOWSPEED 250

//Motors initialization

//motor nr1
#define DIRA D4
#define DIRB D3

// motor nr2
#define DIRA2 D6
#define DIRB2 D5

int a, pos = 0;

long distance() {
   // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHO_PIN, INPUT);
  long duration = pulseIn(ECHO_PIN, HIGH);
  // Convert the time into a distance
  Serial.println((duration/2) / 29.1);
  return (duration/2) / 29.1;
}


//Only one action is requierd since the robot needs to make (in this stage) only a 360 turn
void turnLeft() {
  analogWrite(DIRA,LOWSPEED);
  analogWrite(DIRB,LOW);
  analogWrite(DIRA2,LOW);
  analogWrite(DIRB2,LOWSPEED);

  Serial.print(distance());
  Serial.println(" cm");
}

unsigned long returnTime() {
  unsigned long rTime = millis();
  
  while(distance() > 5) {
    turnLeft();
  }

  Serial.print(millis());
  Serial.println(" millisec");

  analogWrite(DIRA,LOW);
  analogWrite(DIRB,LOW);
  analogWrite(DIRA2,LOW);
  analogWrite(DIRB2,LOW);
    
  return millis() - rTime;
}


void setup() {
  unsigned long t360;
  Serial.begin(74880);

  //Motors pins setup
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  pinMode(DIRA2,OUTPUT);
  pinMode(DIRB2,OUTPUT);

  //SR04 distance sensor pins setup
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input

  delay(100);

  EEPROM.begin(512);
  EEPROM.put(255, returnTime()); //Remember the time of the spin in EEPROM so the value can be used in Rescue_Nemo.ino
  EEPROM.commit();
  EEPROM.end();

  //Check the value (for safety)
  Serial.print("eeprom value: ");
  EEPROM.begin(512);
  EEPROM.get(255, t360);
  EEPROM.end();  

  Serial.println(t360);
}

void loop() {
  delay(20);
}