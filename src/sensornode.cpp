#include <Arduino.h>
#include <NewPing.h>

#define SONAR_NUM 1      // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.
#define trigPin 9
#define echoPin 10

NewPing sonar(trigPin, echoPin, MAX_DISTANCE);
int RED = 5;
int GREEN = 6; 
int YELLOW = 7; 

int ultrasonisk = 0;
int trykksensor = 0;
int photosensor = 0;

void setup() { 
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
}

\\Dette er en enkel if-setning som sjekker med photoresistor hvor lyst det. 
\\Kommer denne verdien under hundre skal lyktestolpen skrus på
void lyktestolper_pa(){
  if (photosensor <= 100){
    digitalWrite(YELLOW, HIGH); 
  }
  else {
    digitalWrite(YELLOW, LOW);
  }
}

void sensor_avlesning() 
{
photosensor = analogRead(A1); \\Henter sensor da fra photosensor
trykksensor = analogRead(A0); \\henter sensor data fra trykksensor
ultrasonisk = sonar.ping_cm(); \ henter sensor data fra ultasonisk sensor
}

void rodt_lys(){
  if (trykksensor >= 100 &&  ultrasonisk >= 100  ) \\Sjekker om trykksensor er trykket og om det kommer en bil
  {
    digitalWrite(GREEN,HIGH); \\lyset skrus på hvis det ikke kommer bil og touchpad er trykket
    digitalWrite(RED, LOW); \\det røde lyset skrus av
    delay (8000);
    }
else {
digitalWrite(GREEN,LOW);
digitalWrite(RED, HIGH);
}
}

void monitor(){ \\dette er komunikasjon med seriemonitor for å få innblikk i avlesningne til sensroene. 
  Serial.print("Ping: ");
  Serial.print(ultrasonisk); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  Serial.println (trykksensor);
  Serial.println (photosensor);
}

void loop() {
  sensor_avlesning(); \\tar for seg sensor avlesning
  monitor(); \\tar for seg monitor styring
  lyktestolper_pa(); \\tar for seg styring av lyktestoler
  rodt_lys(); \\ for seg styring av rødt og grønt lys. 
}
