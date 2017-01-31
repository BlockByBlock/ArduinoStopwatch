/* 4-14-2010 SparkFun Electronics 2012 Nathan Seidle [2-11-2016 edited by BingCheng for WeightThresholdStudy] */

#define ledPin 13       //Status LED connected to digital pin 13
#define receiverPin 9    // button on pin 9 (DEFINE PWM READING PIN)
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>         //for arduino newer than 1.0 
SoftwareSerial mySerial(10, 11);    //mySerial(GPSTX, GPSRX)
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false    //true to listen to raw GPS data

byte signalState;                   // variable to store PWM
int lastsignalState;                // variable to store last PWM
long previousMillis = 0;            // variable to store last time LED was updated 
long startTime ;                    // START time for stop watch
long elapsedTime ;                  // ELAPSED time for stop watch
int fractional;                     // variable used to store FRACTIONAL part of TIME
int blinking = false;               // condition for blinking - timer is TIMING
long interval = 100;                // blink interval - change to suit
int value = LOW;                    // previous value of the LED

boolean usingInterrupt = false;  //off by default - keep track on interrupt 
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup() 
{ 
  Serial.begin(115200); //Set baud rate
  pinMode(ledPin, OUTPUT);  //LED     
  pinMode(receiverPin, INPUT);   //Read PWM
  
  GPS.begin(9600);      // 9600 NMEA is the default baud rate for MTK - some use 4800
     //RMC: Time, date, position, course and speed data (Up to 10Hz) 
     //GGA: Time, position and fix type data (RMGCCA - Up to 5Hz)
     //ALLDATA: Up to 1Hz ONLY
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    
     // Set the update rate, update and pos fix
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);   //Fix limit is at 5Hz

  useInterrupt(true); //interrupt go off every 1 m to read GPS data
  
  delay(1000); //Wait a second for OpenLog to init
  if (GPS.fix){
  Serial.println("Ready for logging"); 
  }
} 

// Interrupt every ms, store new GPS data
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  if (GPSECHO)
    if (c) UDR0 = c;  
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop() 
{ 
   if (timer > millis())  timer = millis(); //reset timer
  //check PWM signal
   signalState = pulseIn(receiverPin, HIGH);                   // read the PWM and store
   Serial.println(signalState);                //print the PWM value

   if (signalState > 100 &&  blinking == false){    
      // if true then trigger while clock is not running - START THE CLOCK
      startTime = millis();   // store the start time
      blinking = true;        // turn on blinking while timing
      delay(5);               // short delay to debounce switch
      lastsignalState = signalState;   // store signalState in lastsignalState, to compare next time 

      //every 0.5 second, print GPS
    if (millis() - timer > 500) { 
      timer = millis(); // reset the timer
    if (GPS.fix) {
          Serial.print("Speed (knots): "); Serial.println(GPS.speed);
          Serial.print("Altitude: "); Serial.println(GPS.altitude);
       }
     }    
   }

   else if (signalState < 100 && blinking == true){     
      // if true then trigger while clock is RUNNING - STOP THE CLOCK AND REPORT
        elapsedTime =   millis() - startTime;     // store elapsed time
        blinking = false;                         // turn off blinking, all done timing
        lastsignalState = signalState;            // store signalState in lastsignalState, to compare next time 

       // routine to report elapsed time 
        Serial.print( (int)(elapsedTime / 1000L));         // divide by 1000 to convert to seconds - then cast to an int to print
        Serial.print(".");                             // print decimal point
        Serial.print("Still Fix?: "); Serial.print((int)GPS.fix);

        // use modulo operator to get fractional part of time 
       fractional = (int)(elapsedTime % 1000L);

       if (fractional == 0)
          Serial.print("000");      // add three zero's
       else if (fractional < 10)    // if fractional < 10 the 0 is ignored giving a wrong time, so add the zeros
          Serial.print("00");       // add two zeros
       else if (fractional < 100)
          Serial.print("0");        // add one zero

       Serial.println(fractional);  // print fractional part of time 
   }
   else{
      lastsignalState = signalState;  // store signalState in lastsignalState, to compare next time
   }

   // blink routine - blink the LED while timing
   if ( (millis() - previousMillis > interval) ) {
      if (blinking == true){
         previousMillis = millis();    // remember the last time we blinked the LED
         // if the LED is off turn it on and vice-versa.
         if (value == LOW)
            value = HIGH;
         else
            value = LOW;
         digitalWrite(ledPin, value);
      }
      else{
         digitalWrite(ledPin, LOW); // turn off LED when not blinking
      }
   }
} 


