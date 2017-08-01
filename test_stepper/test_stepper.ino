/*
  Blink
 Turns on an LED on for one second, then off for one second, repeatedly.
 
 This example code is in the public domain.
 */

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(115200);
  pinMode(led, OUTPUT);     
  pinMode(8, OUTPUT);     
  pinMode(3, INPUT);     
}

// the loop routine runs over and over again forever:
float i = 0;
int num;
int freq;
int k = 1;
long s = 0;
long d;
void loop() {
  long m = micros(); 

  if (s<m)
  {
    freq = millis()%20000/10;
    if (freq<1000) {
      d = freq;
    }
    else {
      d = 2000-freq;
    }
    s += d;
    digitalWrite(8,HIGH);  
    delayMicroseconds(5);
    digitalWrite(8,LOW);
  }
  
  
  //  if (num%100==0) {
  //    Serial.println(t);
  //    num = 0;
  //  }
}






