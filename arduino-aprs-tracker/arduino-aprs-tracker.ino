// Arduino APRS Tracker (aat) with Arduino Pro Mini 3.3V/8 MHz
// Based on https://github.com/sh123/aprs_tracker
// 
#include <SoftwareSerial.h>
#include <SimpleTimer.h>
#include <TinyGPS.h>
#include <LibAPRS.h>

// Single shot button
#define BUTTON_PIN 10

// GPS SoftwareSerial
// Shares pins with (MISO 12/ MOSI 11) used for SPI
#define GPS_RX_PIN 12
#define GPS_TX_PIN 11

// LibAPRS
#define OPEN_SQUELCH false
#define ADC_REFERENCE REF_3V3

// APRS settings
char APRS_CALLSIGN[]="NOCALL";
const int APRS_SSID=5;
char APRS_SYMBOL='>';

// Timer
#define TIMER_DISABLED -1

TinyGPS gps;
SoftwareSerial GPSSerial(GPS_RX_PIN, GPS_TX_PIN);
SimpleTimer timer;

char aprs_update_timer_id = TIMER_DISABLED;
bool send_aprs_update = false;
//long instead of float for latitude and longitude
long lat = 0;
long lon = 0;

int year=0;
byte month=0, day=0, hour=0, minute=0, second=0, hundredths=0;
unsigned long age=0;

// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

void setup()  
{
  Serial.begin(115200);
  GPSSerial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  Serial.println(F("Arduino APRS Tracker"));

  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(APRS_CALLSIGN,APRS_SSID);
  APRS_setSymbol(APRS_SYMBOL);
  
  aprs_update_timer_id=timer.setInterval(2L*60L*1000L, setAprsUpdateFlag);
}

void loop()
{
  bool newData = false;

  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (GPSSerial.available())
    {
      char c = GPSSerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, NULL, &age);
    gps.get_position(&lat, &lon, &age);

    Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(year);
    Serial.print(" "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second));Serial.print(F(" "));

    Serial.print(F("LAT="));Serial.print(lat);
    Serial.print(F(" LON="));Serial.print(lon);

    Serial.print(F(" "));
    Serial.print(deg_to_nmea(lat, true));
    Serial.print(F("/"));

    Serial.println(deg_to_nmea(lon, false));

  if (digitalRead(BUTTON_PIN)==0)
  {
    while(digitalRead(BUTTON_PIN)==0) {}; //debounce
    Serial.println(F("MANUAL UPDATE"));
    locationUpdate();
  }
  
  if (send_aprs_update) {
    Serial.println(F("APRS UPDATE"));
    locationUpdate();
    send_aprs_update = false;
  }

  }
  timer.run();
}

void aprs_msg_callback(struct AX25Msg *msg) {
}

void locationUpdate() {
  char comment []= "Arduino APRS Tracker";

  APRS_setLat((char*)deg_to_nmea(lat, true));
  APRS_setLon((char*)deg_to_nmea(lon, false));
      
  // turn off SoftSerial to stop interrupting tx
  GPSSerial.end();
  
  // TX
  APRS_sendLoc(comment, strlen(comment));
 
  // read TX LED pin and wait till TX has finished (PB5) digital write 13 LED_BUILTIN
  while(bitRead(PORTB,5));

  // start SoftSerial again
  GPSSerial.begin(9600);
}

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
  if( a > 9999) { snprintf(conv_buf , 6, "%04u", a);} else snprintf(conv_buf + 1, 5, "%04u", a);

  conv_buf[5] = '.';
  snprintf(conv_buf + 6, 3, "%02u", b);
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

void setAprsUpdateFlag() {
  send_aprs_update = true;
}
