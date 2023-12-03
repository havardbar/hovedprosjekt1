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
photosensor = analogRead(A1);
trykksensor = analogRead(A0);
ultrasonisk = sonar.ping_cm();
}

void rodt_lys(){
  if (trykksensor >= 100 &&  ultrasonisk >= 100  )
  {
    digitalWrite(GREEN,LOW);
    digitalWrite(RED, HIGH);
    delay (8000);
    }
else {
digitalWrite(GREEN,HIGH);
digitalWrite(RED, LOW);
}
}

void monitor(){
  Serial.print("Ping: ");
  Serial.print(ultrasonisk); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  Serial.println (trykksensor);
  Serial.println (photosensor);
}

void loop() {
  sensor_avlesning();
  monitor();
  lyktestolper_pa();
  rodt_lys();
}
