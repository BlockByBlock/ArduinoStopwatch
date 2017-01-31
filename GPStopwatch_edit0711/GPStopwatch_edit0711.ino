/* 4-14-2010 SparkFun Electronics 2012 Nathan Seidle [8-11-2016 edited by BingCheng for WeightThresholdStudy] */

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#define ledPin 13       //Status LED connected to digital pin 13

SoftwareSerial mySerial(10, 11);   //SoftwareSerial mySerial(GPStx, GPSrx) 
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false   //Echo GPS raw data
boolean usingInterrupt = false; //default as false
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

long previousMillis = 0;            // variable to store last time LED was updated 
long startTime ;                    // START time for stop watch
long elapsedTime ;                  // ELAPSED time for stop watch
int fractional;                     // variable used to store FRACTIONAL part of TIME
int blinking = false;               // condition for blinking - timer is TIMING
long interval = 100;                // blink interval - change to suit
int value = LOW;                    // previous value of the LED

volatile int pwm_value = 0;   
volatile int prev_time = 0;

void setup()  
{
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);  //LED
  attachInterrupt(0, rising, RISING);     
  
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  
     /*RMC: Time, date, position, course and speed data (Up to 10Hz) 
       GGA: Time, position and fix type data (RMGCCA - Up to 5Hz)
       ALLDATA: Up to 1Hz ONLY */
       //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
       //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
       //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
       GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
     /* Set the update rate, update and pos fix */
     //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
     GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
     //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
     //GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
     //GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);   //Fix limit is at 5Hz

    useInterrupt(true);
    delay(1000);
    Serial.println("Ready to CRASH and LOG!"); 
}

void rising() {
  attachInterrupt(0, falling, FALLING);
  prev_time = micros();
}
 
void falling() {
  attachInterrupt(0, rising, RISING);
  pwm_value = micros()-prev_time;
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // UDR0 is faster than Serial.print but only one char at a time
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop()                     // run over and over again
{
  //Serial.println(pwm_value);
  // Not using the interrupt above, 'hand query' the GPS
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // Received GPS, check checksum and parse
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (timer > millis())  timer = millis();  // if millis() or timer wraps around, we'll just reset it

  if (pwm_value < 1500 &&  blinking == false){
    // if true then trigger while clock is not running - START THE CLOCK at 5Hz (200)   
    if (millis() - timer > 200) {  
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {  
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
      startTime = millis();   // store the start time
      blinking = true;        // turn on blinking while timing
  }
     else if (pwm_value > 1500 && blinking == true){     
      // if true then trigger while clock is RUNNING - STOP THE CLOCK AND REPORT
        elapsedTime =   millis() - startTime;     // store elapsed time
        blinking = false;                         // turn off blinking, all done timing

       // routine to report elapsed time
        Serial.print("Time from CUT to IMPACT : "); 
        Serial.print( (int)(elapsedTime / 1000L));         // divide by 1000 to convert to seconds - then cast to an int to print
        Serial.print(".");                             // print decimal point
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


