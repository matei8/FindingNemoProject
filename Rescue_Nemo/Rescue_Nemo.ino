#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define STASSID "TENDA_5C3400"
#define STAPSK  "austria1"

#include <algorithm> // std::min

/*
    SERIAL_LOOPBACK
    0: normal serial operations
    1: RX-TX are internally connected (loopback)
*/

#define SERIAL_LOOPBACK 1

#define BAUD_SERIAL 115200
#define BAUD_LOGGER 115200
#define RXBUFFERSIZE 1024

////////////////////////////////////////////////////////////

#if SERIAL_LOOPBACK
#undef BAUD_SERIAL
#define BAUD_SERIAL 3000000
#include <esp8266_peri.h>
#endif

#define logger (&Serial1)

#define STACK_PROTECTOR  512 // bytes

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 2

const char* ssid = STASSID;
const char* password = STAPSK;
const int port = 23;

WiFiServer server(port);
WiFiClient serverClients[MAX_SRV_CLIENTS];
extern MDNSResponder MDNS;

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
    for(int i=0; i<SAMPLES; i++) {
    
        microSeconds = micros();    //Returns the number of microseconds since the board began running the current script. 
     
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
    //Serial.print(info.amplitude);
    //Serial.print(" ");
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
  int notFound = 5;
  SoundInfo sound;
  while(notFound) {
    sound = getSoundInfo();
    if(sound.frequency > 3000 && sound.frequency < 3500) {
      notFound--;
    } else {
      notFound = 15;
    }
    handleWifi();
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

void moveForward(int speed){
  analogWrite(DIRA,LOW);
  analogWrite(DIRB,speed);
  analogWrite(DIRA2,LOW);
  analogWrite(DIRB2,speed + 65);
}

void stopMotors() {
  analogWrite(DIRA,LOW);
  analogWrite(DIRB,LOW);
  analogWrite(DIRA2,LOW);
  analogWrite(DIRB2,LOW);
  delay(3000);
}

void stepBack(int speed) {
  analogWrite(DIRA,speed);
  analogWrite(DIRB,LOW);
  analogWrite(DIRA2,speed + 60);
  analogWrite(DIRB2,LOW);
}

void findDirection() { // finding the direction from where the sound comes from so we can move twards the robot nr2
  
   Serial.println("FINDING THE DIRECTION");
   Serial.println(" ");
   unsigned long initialTime;
   long t360; // the time that robot nr1 needs to make a 360 turn
   EEPROM.begin(512);
   EEPROM.get(255, t360);
   EEPROM.end();
   t360 = max(t360, 6000l);
   Serial.print("rotation time: ");
   Serial.println(t360);
   int maxAmp = 0;
   unsigned long minDistTime;
   SoundInfo sound;
   handleWifi();
   turnLeft();
   initialTime = millis();
   long distMin = distance();
   do { 
      //sound = getSoundInfo(); 
      //int amp = sound.amplitude;
      long crtDist = distance(); 
      if(distMin > crtDist) {
        distMin = crtDist;
        minDistTime = millis();
      }
     handleWifi();
     Serial.print("distMin: ");
     Serial.println(distMin);
   } while((millis() - initialTime) < t360);
   stopMotors();
   
   Serial.print("min dist time: ");
   Serial.println(minDistTime);
   Serial.print("min dist: ");
   Serial.println(distMin);
   handleWifi();

   unsigned long finalRotationTime = initialTime + t360 - minDistTime + 200;
   initialTime = millis();
   turnRight();
   do {
      handleWifi();
      delay(1);
   } while((distance() > distMin + 10) || ((millis() - initialTime) <= finalRotationTime));
   stopMotors();
   Serial.println(finalRotationTime); 
   handleWifi(); 
}

void searchNemo() {
  Serial.println("SEARCHING NEMO");
  moveForward(500);
  while(distance() > 15) {
    Serial.println(distance());
  }
  moveForward(300);
  while(distance() > 4) {
    Serial.println(distance());
  }
  stopMotors();
  delay(3000);
  stepBack(500);
  delay(2000);
  stopMotors();
}
void WIFI_SETUP() {
  Serial.begin(BAUD_SERIAL);
  Serial.setRxBufferSize(RXBUFFERSIZE);

  logger->begin(BAUD_LOGGER);

  USC0(0) |= (1 << UCLBE); // incomplete HardwareSerial API

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(1000);
//    ESP.restart();
  }
  
  MDNS.begin("TestHost", WiFi.localIP(), 0);

  //start server
  server.begin();
  server.setNoDelay(true);
}

void OTA_SETUP() {

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void handleWifi() {
     //check if there are any new clients
  if (server.hasClient()) {
    //find free/disconnected spot
    int i;
    for (i = 0; i < MAX_SRV_CLIENTS; i++)
      if (!serverClients[i]) { // equivalent to !serverClients[i].connected()
        serverClients[i] = server.available();
        logger->print("New client: index ");
        logger->print(i);
        break;
      }

    //no free/disconnected spot so reject
    if (i == MAX_SRV_CLIENTS) {
      server.available().println("busy");
      // hints: server.available() is a WiFiClient with short-term scope
      // when out of scope, a WiFiClient will
      // - flush() - all data will be sent
      // - stop() - automatically too
      logger->printf("server is busy with %d active connections\n", MAX_SRV_CLIENTS);
    }
  }

  //check TCP clients for data
  // Incredibly, this code is faster than the bufferred one below - #4620 is needed
  // loopback/3000000baud average 348KB/s
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    while (serverClients[i].available() && Serial.availableForWrite() > 0) {
      // working char by char is not very efficient
      Serial.write(serverClients[i].read());
    }

  // determine maximum output size "fair TCP use"
  // client.availableForWrite() returns 0 when !client.connected()
  size_t maxToTcp = 0;
  for (int i = 0; i < MAX_SRV_CLIENTS; i++)
    if (serverClients[i]) {
      size_t afw = serverClients[i].availableForWrite();
      if (afw) {
        if (!maxToTcp) {
          maxToTcp = afw;
        } else {
          maxToTcp = std::min(maxToTcp, afw);
        }
      } else {
        // warn but ignore congested clients
        logger->println("one client is congested");
      }
    }

  //check UART for data
  size_t len = std::min((size_t)Serial.available(), maxToTcp);
  len = std::min(len, (size_t)STACK_PROTECTOR);
  if (len) {
    uint8_t sbuf[len];
    size_t serial_got = Serial.readBytes(sbuf, len);
    // push UART data to all connected telnet clients
    for (int i = 0; i < MAX_SRV_CLIENTS; i++)
      // if client.availableForWrite() was 0 (congested)
      // and increased since then,
      // ensure write space is sufficient:
      if (serverClients[i].availableForWrite() >= serial_got) {
        size_t tcp_sent = serverClients[i].write(sbuf, serial_got);
        if (tcp_sent != len) {
          logger->printf("len mismatch: available:%zd serial-read:%zd tcp-write:%zd\n", len, serial_got, tcp_sent);
        }
      }
  }
  MDNS.update();
  ArduinoOTA.handle();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  Serial.begin(74880);
  OTA_SETUP();
  WIFI_SETUP();
  handleWifi();
  Serial.println("Waiting for signal");
  Serial.println(waitForSignal());
  findDirection();
  searchNemo();
}

void loop() {
  handleWifi();
}
