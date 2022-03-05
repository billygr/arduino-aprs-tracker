# arduino-aprs-tracker

Arduino APRS Tracker based on libAPRS with SmartBeaconing

## Hardware
* Arduino Pro Mini 3.3V 8 MHz https://www.adafruit.com/products/2377
* Adafruit Ultimate GPS https://www.adafruit.com/product/746
* or any GPS that can output NMEA (NEO-6M is tested with some minor issues)

## Libraries
* TinyGPS http://arduiniana.org/libraries/tinygps/
* libAPRS (modified) https://github.com/billygr/LibAPRS

## Notes
There are cases where the NEO-6M displays wrong data such as course/altitude. I have trace it to the interrupt of ADC (needed for APRS).
When the interrupt is fired, disrupts the SoftwareSerial so it looses GPS messages.
By default NEO-6M has these messages enabled GSV, RMC, GSA, GGA, GLL, VTG, TXT. 
The GLL is not needed so it is disabled on power on. If you still have issues enable the define, it will switch the NEO-6M on 4800 baud which looks like it fixes it
