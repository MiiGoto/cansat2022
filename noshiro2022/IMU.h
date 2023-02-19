#ifndef _IMU_h
#define _IMU_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define deg_to_rad(degr) (((degr) / 360) * 2 * M_PI) // 度をラジアンに変換する
#define rad_to_deg(rad) (((rad) / 2 / M_PI) * 360)  // ラジアンを度に変換する

class IMU{
  public:
    IMU();
    void setup();
    //void get_direction(double *dirction_now);
    double get_direction();
    bool check_stucked();
    bool landing_judgement();
    void print_acc_data(double acc[3]);

  private:
    const Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
    bool drop_flag = false;
    bool landing_flag = false;
    double o_xyz[3]; //orientation
    double a_xyz[3]; //acceleration
    double m_xyz[3]; //magnetic
    double angV_xyz[3]; //angular velocity
    double rpy[3]; //roll, pitch, yaw
    double horizontally_flag = false;
    
    double mx_max, mx_min, my_max, my_min, mz_max, mz_min;
    unsigned long m_init_time = 20*1000; //20 is defoult, 5 is for test
    //unsigned long m_init_time = 60*1000;
      //magnetic data max and min, for calibration
    double m_direction, m_gap = -7.3, dir_yaw_gap=180; //磁北の方位(真北から磁北のずれ), gap
    double direction; // dorection after calibration
    int drop_TH = 40; //drop threshold
    int landing_TH = 45; //landing threshold 

    void m_init();
    void get_data();
    
    //void get_rpy();


};
#endif
