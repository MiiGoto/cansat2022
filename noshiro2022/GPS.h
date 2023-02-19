#ifndef _GPS_h
#define _GPS_h

#include "Arduino.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

#define deg_to_rad(degr) (((degr) / 360) * 2 * M_PI)
#define rad_to_deg(rad) (((rad) / 2 / M_PI) * 360)

class GPS{
  public :
    GPS();
    void setup();
    void readSenser(double* lat, double* lng, double* meters);
    double get_gps_direction();
  private:
  float lat, lng, meters;
  unsigned long loop_count;
};
#endif
