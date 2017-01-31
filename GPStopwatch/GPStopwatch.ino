/* 4-14-2010 SparkFun Electronics 2012 Nathan Seidle [7-11-2016 edited by BingCheng for WeightThresholdStudy] */

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#define ledPin 13       //Status LED connected to digital pin 13
#define receiverPin 9    // button on pin 9 (DEFINE PWM READING PIN)

SoftwareSerial mySerial(0, 1);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

byte signalState;                   // variable to store PWM
int lastsignalState;                // variable to store last PWM
long previousMillis = 0;            // variable to store last time LED was updated 
long startTime ;                    // START time for stop watch
long elapsedTime ;                  // ELAPSED time for stop watch
int fractional;                     // variable used to store FRACTIONAL part of TIME
int blinking = false;               // condition for blinking - timer is TIMING
long interval = 100;                // blink interval - change to suit
int value = LOW;                    // previous value of the LED

void setup()  
{
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);  //LED     
  pinMode(receiverPin, INPUT);   //Read PWM
  
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  
     /*RMC: Time, date, position, course and speed data (Up to 10Hz) 
       GGA: Time, position and fix type data (RMGCCA - Up to 5Hz)
       ALLDATA: Up to 1Hz ONLY */
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
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
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
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
   //check PWM signal
   signalState = pulseIn(receiverPin, HIGH);                   // read the PWM and store
   Serial.println(signalState);                //print the PWM value

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 0.2 seconds or so, print out the current stats
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
}
