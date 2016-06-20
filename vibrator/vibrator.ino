#include <SoftwareSerial.h>
SoftwareSerial bluetooth(19,-1);

#define VIBRATOR_PIN 5      // PWM pin on Arduino Nano

char character;         // variable to receive data from the serial port
int LED_PIN = 13;  // LED connected to pin 13 (on-board LED)
int value = 0;
String Data = "";
int MIN_VALUE = 200;
int MAX_VALUE = 50;
unsigned long previousMillis = 0;
unsigned long timer = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);  // pin 13 (on-board LED) as OUTPUT
  pinMode(VIBRATOR_PIN, OUTPUT);
  bluetooth.begin(9600);
  Serial.begin(9600);       // start serial communication at 115200bps
  Serial.println("Starting");
}
 
void loop() {
  if (value > 0) {
    vibrateOnInput(false);
  }

  //Once a bluetooth connection is establish, loop
  while (bluetooth.available())
  {
    character = bluetooth.read(); // Receive a single character from the software serial port
    Data.concat(character); // Add the received character to the receive buffer
    if (character == '\n')   //Delimiter at which the data needs to be parsed
    {
      value = Data.toInt();
   
      Serial.println(value); //Print out for debugging
      vibrateOnInput(true); //Call the vibrator method and tell there is a new input
      if (value < 50) {
        digitalWrite(LED_PIN, HIGH); //Debug LED
      }
      else {
        digitalWrite(LED_PIN, LOW);
      }
             
      // Clear receive buffer so we're ready to receive the next line
      Data = "";
    }
  }
}

void vibrateOnInput(boolean newValue ) 
{
  unsigned long currentMillis = millis();
  
  if (newValue){
    newValue = false;
    timer = currentMillis;
    previousMillis = currentMillis;
  }

  if (currentMillis - timer > 800) {
      analogWrite(VIBRATOR_PIN, 0);    // turn the motor off
  }
  else {
    //Below 50cm full vibration strength (analogWrite(150))
    Serial.println("Do you even work");
    if (value < MAX_VALUE) { 
      if(currentMillis - previousMillis < 50) {
        analogWrite(VIBRATOR_PIN, (MIN_VALUE - MAX_VALUE));
      }
      else if(currentMillis - previousMillis < 100) {
          analogWrite(VIBRATOR_PIN, 0);   
      }
      else {
          previousMillis = currentMillis;
      }
    }
  
    //Below 50cm and 200cm vibration strength and pulse length is depended on the distance
    else if (value < MIN_VALUE) {
      if(currentMillis - previousMillis < (50 + (value / 3))) {
        analogWrite(VIBRATOR_PIN, (MIN_VALUE - value)); 
      }
      else if (currentMillis - previousMillis > ((50 + (value / 3)) * 2)) {
        analogWrite(VIBRATOR_PIN, 0);
      }
      else {
        previousMillis = currentMillis;
      }
    }
    
    //If over 200cm don't vibrate
    else {
      analogWrite(VIBRATOR_PIN, 0);    // turn the motor off
      previousMillis = currentMillis;
    }
  }
}
