#include <EEPROM.h>

#include <arduinoFFT.h>

#define SAMPLES 2048            //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 8192 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
 
arduinoFFT FFT = arduinoFFT();
 
unsigned int samplingPeriod;
unsigned long microSeconds;
 
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values

typedef struct SI {
  double frequency;
  double amplitude;
} SoundInfo;

SoundInfo getSoundInfo() {
  SoundInfo info;
  info.amplitude = 0;

    /*Sample SAMPLES times*/
    for(int i=0; i<SAMPLES; i++)
    {
        microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
     
        vReal[i] = analogRead(0); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
        vImag[i] = 0; //Makes imaginary term 0 always
        info.amplitude = max(info.amplitude, vReal[i]);
        /*remaining wait time between samples if necessary*/
        while(micros() < (microSeconds + samplingPeriod)){
          
        }
    }
 
    /*Perform FFT on samples*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    /*Find peak frequency and print peak*/
    //info.frequency = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    delay(10);
    double sound[2];
    FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY, &info.frequency, &info.amplitude);
    Serial.print("peak ");     //Print out the most dominant frequency.
    Serial.print(info.amplitude);
    Serial.print(" ");
    Serial.println(info.frequency);     //Print out the most dominant frequency
  return info;
}
// end FFT


#define TRIG_PIN D8
#define ECHO_PIN D10

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
  return (duration/2) / 29.1;
}

double waitForSignal() {
  int notFound = 15;
  SoundInfo sound;
  
  while(notFound) {
    sound = getSoundInfo();
    if(sound.frequency > 3000 && sound.frequency < 3500) {
      notFound--;
    } else {
      notFound = 15;
    }
    delay(20);
  }
  return sound.frequency;
}

// motor nr1
#define DIRA D4
#define DIRB D3
// motor nr2
#define DIRA2 D6
#define DIRB2 D5

#define TIME360 0
#define LOWSPEED 300

void turnLeft() {
  analogWrite(DIRA,LOWSPEED);
  analogWrite(DIRB,LOW);
  analogWrite(DIRA2,LOW);
  analogWrite(DIRB2,LOWSPEED);
}

void turnRight() {
  analogWrite(DIRA,LOW);
  analogWrite(DIRB,LOWSPEED);
  analogWrite(DIRA2,LOWSPEED);
  analogWrite(DIRB2,LOW);
}

void moveForward(){
  analogWrite(DIRA,LOW);
  analogWrite(DIRB,LOWSPEED);
  analogWrite(DIRA2,LOW);
  analogWrite(DIRB2,LOWSPEED);
}

void stopMotors() {
  analogWrite(DIRA,LOW);
  analogWrite(DIRB,LOW);
  analogWrite(DIRA2,LOW);
  analogWrite(DIRB2,LOW);
  delay(3000);
}

void findDirection() { // finding the direction from where the sound comes from so we can move twards the robot nr2
  
   Serial.println("FINDING THE DIRECTION");
   unsigned long initialTime;
   unsigned long t360; // the time that robot nr1 needs to make a 360 turn
   EEPROM.begin(512);
   EEPROM.get(255, t360);
   EEPROM.end();
   Serial.print("rotation time: ");
   Serial.println(t360);
   int maxAmp = 0;
   unsigned long maxAmpTime;
   SoundInfo sound;
   turnLeft();
   initialTime = millis();
   do { 
      sound = getSoundInfo(); 
      int amp = sound.amplitude;
      Serial.print("amplitude: ");
      Serial.println(amp);
      if(amp > maxAmp) {
        maxAmp = amp;
        maxAmpTime = millis();
      }
      delay(10);
   } while((millis() - initialTime) < t360);
   stopMotors();
   
   Serial.print("max amp time: ");
   Serial.println(maxAmpTime);
   Serial.print("max amplitude: ");
   Serial.println(maxAmp);

   unsigned long finalRotationTime = initialTime + t360 - maxAmpTime;
   initialTime = millis();
   turnRight();
   do {
    delay(10);
   } while((millis() - initialTime) < finalRotationTime);
   stopMotors();  
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  Serial.begin(74880);
  Serial.println(waitForSignal());
  findDirection();
}

void loop() {
  // put your main code here, to run repeatedly:

}
