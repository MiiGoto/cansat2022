#include "IMU.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>


// クラスを使うときに最初に呼び出されるやつ。一部のメンバ変数を初期化する。
IMU::IMU()
{
  mx_max = 0;
  mx_min = 0;
  my_max = 0;
  my_min = 0;
  mz_max = 0;
  mz_min = 0;
}

// .ino ファイルで使う用の関数, 初期化とキャリブレーションを行う, 引数 戻り値なし
void IMU::setup()
{
  // Serial.println("IMU setupping...");
  // // Serial.begin(115200);
  // Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
  // uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

  /* Initialise the sensor */

  if (!bno.begin())
  {
    /*There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  m_init();

  delay(1000);
  // Serial.println("IMU setting done.");
}

// 初期化とキャリブレーションを行う, 引数 戻り値なし
void IMU::m_init()
{
  unsigned long start_time = millis();
  // for (int i = 0; i < m_init_N; i++)
  while (millis() - start_time < m_init_time)
  {
    // Serial.print("Setting : ");
    // Serial.print((millis() - start_time) / 100);
    // Serial.print(" / ");
    // Serial.println(m_init_time / 100);
    int8_t boardTemp = bno.getTemp();
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if (mx_max < m_xyz[0])
    {
      mx_max = m_xyz[0];
    }
    else if (mx_min > m_xyz[0])
    {
      mx_min = m_xyz[0];
    }
    if (my_max < m_xyz[1])
    {
      my_max = m_xyz[1];
    }
    else if (my_min > m_xyz[1])
    {
      my_min = m_xyz[1];
    }
  }
}

// 進行方角の取得, 返り値:double 方角(deg)
double IMU::get_direction()
{
  // Serial.print("IMU data getting...  ");
  /* Get thw direction of travel*/
  double mx, my, mz;
  get_data();
  //中央値を0にする
  //ほんとは 中心のずれた楕円→(0, 0)を中心とする真円 って風に変換したい
  mx = m_xyz[0] - (mx_max + mx_min) / 2;
  my = m_xyz[1] - (my_max + my_min) / 2;
  mz = m_xyz[2] - (mz_max + mz_min) / 2;

  //傾きの補正
  //数式はあってるはずだけどうまくいかない。制御分かる人なんとかして
  // mx=mx*cos(pry[0])+my*sin(pry[1])*sin(pry[0])+mz*cos(pry[1])*sin(pry[0]);
  // my=my*cos(pry[1])-mz*sin(pry[1]);

  //方位の計算, データを方位角に変換(右回りが正)
  m_direction = -rad_to_deg(atan2(-mx, -my));
  if(m_direction < 0){
    m_direction += 360;
  }

  //手動で磁北と真北のずれ等々の修正
  m_direction += m_gap;
  if (m_direction < 0)
  {
    m_direction += 360;
  }
  else if (m_direction > 360)
  {
    m_direction -= 360;
  }

  // 傾いてたら地磁気は諦めて、yawを使う。傾きの補正がしっかりできればこれはいらない
  //補正のちゃんとした数式を使いたいね。制御分かる人なんとかして
  if (abs(rpy[0]) > 3 || abs(rpy[1]) > 3)
  {
    horizontally_flag = false;
    direction = rpy[2] - dir_yaw_gap;
    if (direction < 0)
    {
      direction += 360;
    }
    else if (direction > 360)
    {
      direction -= 360;
    }
    // Serial.print("USE [ YAW ] DATA NOW!!   ");
    // Serial.println(rpy[0]);
    // Serial.println(rpy[1]);
    // Serial.println(rpy[2]);
  }
  else
  {
    horizontally_flag = true;
    direction = m_direction;
    dir_yaw_gap = rpy[2] - m_direction;
    // // Serial.print("USE [ Magnetic ] DATA NOW!!   ");
    // // Serial.println(rpy[0]);
    // // Serial.println(rpy[1]);
    // // Serial.println(rpy[2]);
  }
  // Serial.println("IMU data-getting is DONE!!   \n");
  return direction;
  // return rpy[2];
}

// スタックしているかどうかの判定, 返り値:bool
// モーターが回転しているのに移動できていない→スタックしている
// 未実装
bool IMU::check_stucked();

// 落下後かどうかを判定する, 戻り値:bool
bool IMU::landing_judgement()
{
  if (landing_flag)
  {
    return true;
  }
  else
  {
    IMU::get_data();
    double acc_abs = sqrt(a_xyz[0] * a_xyz[0] + a_xyz[1] * a_xyz[1] + a_xyz[2] * a_xyz[2]);
    if (acc_abs >= landing_TH)
    {
      landing_flag = true;
      return true;
    }
    else
    {
      return false;
    }
  }
}

// code for test of IMU
void IMU::print_acc_data(double acc[3]){
  //IMU::get_data();
  acc[0] = a_xyz[0];
  acc[1] = a_xyz[1];
  acc[2] = a_xyz[2];

}

// データの取得, 引数 戻り値なし
void IMU::get_data()
{

  // could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  int8_t boardTemp = bno.getTemp();
  uint8_t system, gyro, accel, mag = 0;

  //一応ここでもキャリブレーションされてるはず。
  bno.getCalibration(&system, &gyro, &accel, &mag);

  o_xyz[0] = orientationData.orientation.x;
  o_xyz[1] = orientationData.orientation.y;
  o_xyz[2] = orientationData.orientation.z;
  a_xyz[0] = accelerometerData.acceleration.x;
  a_xyz[1] = accelerometerData.acceleration.y;
  a_xyz[2] = accelerometerData.acceleration.z;
  m_xyz[0] = magnetometerData.magnetic.x;
  m_xyz[1] = magnetometerData.magnetic.y;
  m_xyz[2] = magnetometerData.magnetic.z;
  angV_xyz[0] = angVelocityData.gyro.x;
  angV_xyz[1] = angVelocityData.gyro.y;
  angV_xyz[2] = angVelocityData.gyro.z;

  rpy[0] = 0.995 * (o_xyz[2] + angV_xyz[2] * BNO055_SAMPLERATE_DELAY_MS / 1000) + 0.005 * atan2(a_xyz[1], a_xyz[2]);
  rpy[1] = 0.995 * (o_xyz[1] + angV_xyz[1] * BNO055_SAMPLERATE_DELAY_MS / 1000) + 0.005 * atan2(-a_xyz[0], sqrt(a_xyz[1] * a_xyz[1] + a_xyz[2] * a_xyz[2]));
  rpy[2] = 0.995 * (o_xyz[0] + angV_xyz[0] * BNO055_SAMPLERATE_DELAY_MS / 1000) + 0.005 * atan2(-a_xyz[2], sqrt(a_xyz[0] * a_xyz[0] + a_xyz[1] * a_xyz[1]));
}
