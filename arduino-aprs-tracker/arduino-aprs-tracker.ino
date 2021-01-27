/***
 * Arduino APRS Tracker (aat) with Arduino Pro Mini 3.3V/8 MHz
 * Install the following libraries through Arduino Library Manager
 * - TinyGPS by Mikal Hart
 * -
 * -
 ***/

#include <SoftwareSerial.h>
#include "src/SimpleTimer/SimpleTimer.h"
#include <TinyGPS.h>
#include <LibAPRS.h>

// Manual update button
#define BUTTON_PIN 10

// GPS SoftwareSerial
// Shares pins with (MISO 12/ MOSI 11) used for SPI
#define GPS_RX_PIN 12
#define GPS_TX_PIN 11
TinyGPS gps;
SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);

// LibAPRS
#define OPEN_SQUELCH false
#define ADC_REFERENCE REF_3V3


// PPT_PIN is defined on libAPRS/device.h
//#define PPT_PIN 3

// GPS_FIX_LED A3/D17
#define GPS_FIX_LED A3

// APRS settings
char APRS_CALLSIGN[] = "NOCALL";
const int APRS_SSID = 5;
char APRS_SYMBOL = '>';


// SmartBeaconing(tm) Setting  http://www.hamhud.net/hh2/smartbeacon.html implementation by LU5EFN

#define LOW_SPEED 5 // [km/h]
#define HIGH_SPEED 90

#define SLOW_RATE 1750 // [seg]
#define FAST_BEACON_RATE  175

#define TURN_MIN  30
#define TURN_SLOPE  240
#define MIN_TURN_TIME 20

//long instead of float for latitude and longitude
long lat = 0;
long lon = 0;

int speed_kt = 0;
int currentcourse=0;
int ialt=0;

// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

/*****************************************************************************************/
void setup()
{
  Serial.begin(115200);
  GPSSerial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GPS_FIX_LED,OUTPUT);

  Serial.println(F("Arduino APRS Tracker"));

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // 1 Hz update rate, not needed for an APRS Tracker
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // Request updates on antenna status, keep it disabled
  //GPS.sendCommand(PGCMD_ANTENNA);

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(APRS_CALLSIGN,APRS_SSID);
  APRS_setSymbol(APRS_SYMBOL);
  
}

/*****************************************************************************************/
void loop()
{
  bool newData = false;
  int year=0;
  byte month=0, day=0, hour=0, minute=0, second=0, hundredths=0;
  unsigned long age=0;

  float falt=0, fkmph=0;

  unsigned long lastTX =0, tx_interval= 0;

  int previouscourse = 0, turn_threshold = 0, courseDelta = 0;
  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPSSerial.available())
    {
      char c = GPSSerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
      {
      // FIXME potential wrong data coming from GPS
        gps.get_position(&lat, &lon, &age);
        if (lat != 0) newData = true;
      }
    }
  }

  if (newData)
  {
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &age);
    gps.get_position(&lat, &lon, &age);
    
    falt = gps.f_altitude(); // +/- altitude in meters
    ialt = int(falt*3.281);  // integer value of altitude in feet

    fkmph = gps.f_speed_kmph(); // speed in km/hs

    speed_kt = (int) gps.f_speed_knots();

    currentcourse = (int) gps.f_course();

    // Calculate difference of course to smartbeacon
    courseDelta = (int) ( previouscourse - currentcourse );
    courseDelta = abs (courseDelta);
    if (courseDelta > 180) {
      courseDelta = courseDelta - 360;
    }
    courseDelta = abs (courseDelta) ;

    if (age == TinyGPS::GPS_INVALID_AGE)
      Serial.println(F("No fix detected"));
    else if (age > 5000)
      Serial.println(F("Warning: possible stale data!"));
    else
      {
        //Serial.println(F("Data is current."));
        digitalWrite(GPS_FIX_LED, !digitalRead(GPS_FIX_LED)); // Toggles the GPS FIX LED on/off
      }


    Serial.print(static_cast<int>(day)); Serial.print(F("/")); Serial.print(static_cast<int>(month)); Serial.print(F("/")); Serial.print(year);
    Serial.print(F(" ")); Serial.print(static_cast<int>(hour)); Serial.print(F(":")); Serial.print(static_cast<int>(minute)); Serial.print(F(":")); Serial.print(static_cast<int>(second));
    Serial.print(F(" "));
    Serial.print(F("LAT="));Serial.print(lat);
    Serial.print(F(" LON="));Serial.print(lon);
    Serial.print(F(" "));
    Serial.print(deg_to_nmea(lat, true));
    Serial.print(F("/"));
    Serial.print(deg_to_nmea(lon, false));
    Serial.print(F(" Altitude m/ft: ")); Serial.print(falt);Serial.print(F("/"));Serial.println(ialt);


    if (digitalRead(BUTTON_PIN)==0)
    {
      while(digitalRead(BUTTON_PIN)==0) {}; //debounce
      Serial.println(F("MANUAL UPDATE"));
      locationUpdate();
    }
  
    // Based on HamHUB Smart Beaconing(tm) algorithm
    if ( fkmph < LOW_SPEED ) {
      tx_interval = SLOW_RATE *1000L;
    }
    else if ( fkmph > HIGH_SPEED) {
      tx_interval = FAST_BEACON_RATE  *1000L;
    }
    else {
    // Interval inbetween low and high speed
    tx_interval = ((FAST_BEACON_RATE * HIGH_SPEED) / fkmph ) *1000L ;
    }

    turn_threshold = TURN_MIN + TURN_SLOPE / fkmph;

    if (courseDelta > turn_threshold ){
      if ( millis() - lastTX > MIN_TURN_TIME *1000L){
        Serial.println(F("APRS UPDATE"));
        locationUpdate();
        lastTX = millis();
      }
    }

    previouscourse = currentcourse;

    if ( millis() - lastTX > tx_interval) {
      Serial.println(F("APRS UPDATE"));
      locationUpdate();
      lastTX = millis();
    }

  }

}

/*****************************************************************************************/
void aprs_msg_callback(struct AX25Msg *msg) {
}

/*****************************************************************************************/
void locationUpdate() {
//Altitude in Comment Text — The comment may contain an altitude value,
//in the form /A=aaaaaa, where aaaaaa is the altitude in feet. For example:
//A=001234. The altitude may appear anywhere in the comment.
//Source: APRS protcol

  char comment []= "Arduino APRS Tracker";
  char temp[8];
  char APRS_comment [36]="/A=";

  // Convert altitude in string and pad left
  sprintf(temp, "%06d", ialt);
  strcat(APRS_comment,temp);
  strcat(APRS_comment,comment);
  //Serial.println(APRS_comment);

  APRS_setLat((char*)deg_to_nmea(lat, true));
  APRS_setLon((char*)deg_to_nmea(lon, false));

  APRS_setSpeed(speed_kt);
  APRS_setCourse(currentcourse);

  // turn off SoftSerial to stop interrupting tx
  GPSSerial.end();

  // TX
  APRS_sendLoc(APRS_comment, strlen(APRS_comment));

  // read TX LED pin and wait till TX has finished. LibAPRS has TX_LED defined on (PB5), i use LED_BUILTIN on my version as TX_LED
  while(digitalRead(LED_BUILTIN));

  // start SoftSerial again
  GPSSerial.begin(9600);
}

/*****************************************************************************************/
/*
**  Convert degrees in long format to APRS string format
**  DDMM.hhN for latitude and DDDMM.hhW for longitude
**  D is degrees, M is minutes and h is hundredths of minutes.
**  http://www.aprs.net/vm/DOS/PROTOCOL.HTM
*/
char* deg_to_nmea(long deg, boolean is_lat) {
  bool is_negative=0;
  if (deg < 0) is_negative=1;

  // Use the absolute number for calculation and update the buffer at the end
  deg = labs(deg);

  unsigned long b = (deg % 1000000UL) * 60UL;
  unsigned long a = (deg / 1000000UL) * 100UL + b / 1000000UL;
  b = (b % 1000000UL) / 10000UL;

  conv_buf[0] = '0';
  // in case latitude is a 3 digit number (degrees in long format)
  if( a > 9999) {
    snprintf(conv_buf , 6, "%04lu", a);
  } else {
    snprintf(conv_buf + 1, 5, "%04lu", a);
  }

  conv_buf[5] = '.';
  snprintf(conv_buf + 6, 3, "%02lu", b);
  conv_buf[9] = '\0';
  if (is_lat) {
    if (is_negative) {conv_buf[8]='S';}
    else conv_buf[8]='N';
    return conv_buf+1;
    // conv_buf +1 because we want to omit the leading zero
    }
  else {
    if (is_negative) {conv_buf[8]='W';}
    else conv_buf[8]='E';
    return conv_buf;
    }
}
