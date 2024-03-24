/***
 * Arduino APRS Tracker (aat) with Arduino Pro Mini 3.3V/8 MHz
 * Install the following libraries through Arduino Library Manager
 * - TinyGPS by Mikal Hart
 * -
 * -
 ***/

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <LibAPRS.h>

// Enable this define NEO-6M if you are constantly getting wrong data,
// the ADC ISR triggers during SoftwareSerial so we miss data coming from GPS
// (for some reason only on this type of GPS and only when running on 9600 bps)
//#define NEO-6M

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
int currentcourse = 0;
int ialtitude_feet = 0;

unsigned long lastTX = 0, tx_interval = 0;
int previouscourse = 0, turn_threshold = 0, courseDelta = 0;

bool newData = false;
// buffer for conversions
#define CONV_BUF_SIZE 16
static char conv_buf[CONV_BUF_SIZE];

/*****************************************************************************************/
void setup()
{
  Serial.begin(115200);
  GPSSerial.begin(9600);

#ifdef NEO-6M
  GPSSerial.print("$PUBX,41,1,0007,0003,4800,0*13\r\n");
  GPSSerial.begin(4800);
  GPSSerial.flush();
#endif

  // Reduce NMEA messages
  disableGPGLL();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GPS_FIX_LED,OUTPUT);

  Serial.println(F("Arduino APRS Tracker"));

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(APRS_CALLSIGN,APRS_SSID);
  APRS_setSymbol(APRS_SYMBOL);

  Serial.print(F("Callsign:     ")); Serial.print(APRS_CALLSIGN); Serial.print(F("-")); Serial.println(APRS_SSID);
  Serial.print(F("Free RAM:     ")); Serial.println(freeMemory());
  Serial.println(F("Date       Time     Latitude  Longitude  Course Speed APRS Lat/Lon       Altitude   Course current/Delta/turn_threshold"));
  Serial.println(F("                    (deg)     (deg)      (deg)  (Km/h)DDMM.mmN/DDDMM.mmW (m/ft)                "));
  Serial.println(F("-----------------------------------------------------------------------------------------------"));
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPSSerial.available())
      {
        if (gps.encode(GPSSerial.read())) newData = true;

      }
  } while (millis() - start < ms);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print(F("********** ******** "));
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  smartdelay(0);
}
/*****************************************************************************************/
void loop()
{
  float flat, flon;

  unsigned long age=0;

  float faltitude_meters=0, fkmph=0;

  // Process GPS data every second (or so)
  smartdelay(1000);
  gps.get_position(&lat, &lon, &age);

  while (age == TinyGPS::GPS_INVALID_AGE)
  {
    // It is not an ideal check but provides a good indication, could be no fix/GPS is not connected/GPS is sending garbage
    Serial.println(F("No fix detected"));
    smartdelay(1000);
    gps.get_position(&lat, &lon, &age);
  }

  if (newData && lat !=0)
  {
    // Parse twice the location with both float and long. Using float helps to paste it directly to google maps for troubleshooting
    gps.get_position(&lat, &lon, &age);
    gps.f_get_position(&flat, &flon, &age);

    // TinyGPS reports TinyGPS::GPS_INVALID_F_ALTITUDE = 1000000.0 for invalid altitude
    faltitude_meters = gps.f_altitude(); // +/- altitude in meters

    // Quick fix to ignore wrong data in calculations
    if (int (faltitude_meters) > 30000) faltitude_meters =0;
    ialtitude_feet = int(faltitude_meters*3.281);  // integer value of altitude in feet

    fkmph = gps.f_speed_kmph(); // speed in km/h

    // Quick fix to ignore wrong data in calculations
    if (int (fkmph) > 300) fkmph = 0;

    speed_kt = (int) gps.f_speed_knots(); // speed in knots for APRS

    // TinyGPS reports TinyGPS::GPS_INVALID_F_ANGLE = 1000.0 for invalid course
    currentcourse = (int) gps.f_course();

    // Quick fix to ignore wrong data in calculations
    if (currentcourse > 360 || currentcourse < 0) currentcourse=0;

    // Calculate difference of course to smartbeacon
    courseDelta = (int) ( previouscourse - currentcourse );
    courseDelta = abs (courseDelta);
    if (courseDelta > 180) {
      courseDelta = courseDelta - 360;
    }
    courseDelta = abs (courseDelta) ;

    digitalWrite(GPS_FIX_LED, !digitalRead(GPS_FIX_LED)); // Toggle the GPS_FIX_LED on/off

    print_date(gps);
    print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
    print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
    print_float(fkmph, TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
    Serial.print(deg_to_nmea(lat, true));Serial.print(F("/"));Serial.print(deg_to_nmea(lon, false));
    Serial.print(F(" ")); Serial.print(faltitude_meters);Serial.print(F("/"));Serial.print(ialtitude_feet);

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

    Serial.print(F(" "));Serial.print(currentcourse);Serial.print(F(" "));Serial.print(courseDelta);
    Serial.print(F(" "));Serial.println(turn_threshold);

    if (courseDelta > turn_threshold ){
      if ( millis() - lastTX > MIN_TURN_TIME *1000L){
        Serial.println(F("DEBUG: APRS UPDATE because of millis() - lastTX > MIN_TURN_TIME *1000L"));
        locationUpdate();
        lastTX = millis();
      }
    }

    previouscourse = currentcourse;
    // tx_internal with default values 1750000 milliseconds/1750 seconds/29.16 minutes
    if ( millis() - lastTX > tx_interval) {
      Serial.println(F("DEBUG: APRS UPDATE because of millis() - lastTX > tx_interval"));
      locationUpdate();
      lastTX = millis();
    }

    if (digitalRead(BUTTON_PIN)==0)
    {
      while(digitalRead(BUTTON_PIN)==0) {}; //debounce
      Serial.println(F("MANUAL UPDATE"));
      locationUpdate();
    }
  }
  newData = false;
}

/*****************************************************************************************/
void aprs_msg_callback(struct AX25Msg *msg) {
}

/*****************************************************************************************/
void locationUpdate() {
//Altitude in Comment Text — The comment may contain an altitude value,
//in the form /A=aaaaaa, where aaaaaa is the altitude in feet. For example:
//A=001234. The altitude may appear anywhere in the comment.
//Source: APRS protocol
//Which means /A=000XXXArduino APRS Tracker (29 characters)

  char comment []= "Arduino APRS Tracker";
  char temp[8];
  char APRS_comment [32]="/A=";

  // Convert altitude in string and pad left
  sprintf(temp, "%06d", ialtitude_feet);
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
#ifdef NEO-6M
   GPSSerial.begin(4800);
#else
   GPSSerial.begin(9600);
#endif
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

void disableGPGLL()
{
  const char *msg = "PUBX,40,GLL,0,0,0,0,0";

  // find checksum
  int checksum = 0;
  for (int i = 0; msg[i]; i++)
    checksum ^= (unsigned char)msg[i];

  // convert and create checksum HEX string
  char checkTmp[8];
  snprintf(checkTmp, sizeof(checkTmp)-1, "*%.2X", checksum);

  // send to module
  GPSSerial.print("$");
  GPSSerial.print(msg);
  GPSSerial.println(checkTmp);
  GPSSerial.flush();
}
