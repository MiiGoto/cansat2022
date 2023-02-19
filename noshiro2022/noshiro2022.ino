/* 
  落下耐久試験(2mぐらいから自由落下のやつ)のあとにゴールに向かって走らせる
  GPS.cpp, IMU.cpp はSerial.print関係を省いたスタンドアローン仕様

  接続 (Aruduino nano)
    A4(SDA)   BNO055(SDA)
    A5(SCL)   BNO055(SCL)
    D2        GPS(RXT)
    D4        GPS(TXD)
    D7        LED(DIN)
    D10(SS)   SD(CS)
    D11(MOSI) SD(MOSI)
    D12(MISO) SD(MISO)
    D13(SCK)  SD(SCK)
    GND       All GND

  Author: Kawai
*/
// INCLUDE FILES ///////////////////////////////////////////////////////////
#include "IMU.h"
#include "GPS.h"
#include "LED.h"
#include <SPI.h>
#include <SD.h>
#include "CytronMotorDriver.h" //install from github
#include "math.h"

// omajinai
// name the instance,which is instance of GPS class, as "GPS".
IMU IMU;
GPS GPS;

// DIFINE VARIABLES ////////////////////////////////////////////////////////

// IMU
double direction;


// GPS 左から緯度、経度、高さ
// 常盤小 体育館 40.225924, 140.138352
double lat = 40.225939, lng = 140.138345, meters = 0; //初期値は投下地点にできるとよいね。もしくはGPSが０の時は動かずにデータ取得を待つようにするか
double previous_lat = 40.225939, previous_lng = 140.138345; // 前回のGPSデータ
double lat_estimated = 40.225939, lng_estimated = 140.138345;
// GOAL
// 寮 goal_lat = 35.10759; goal_lng = 136.98219;
// 中学 goal_lat = 35.10488; goal_lng = 136.98283;
// G中央 goal_lat = 35.10684; goal_lng = 136.98120;
// 落下試験の駐車場 goal_lat = 35.10576; goal_lng = 136.98361;
// 常盤小 体育館 40.226039, 140.138286
static double goal_lat = 40.226039;
static double goal_lng = 140.138286;

// LED
const int DIN_PIN = 7; // D7
const int LED_COUNT = 3; // LEDの数
LED LED(LED_COUNT, DIN_PIN, NEO_GRB + NEO_KHZ800);

// SDCARD
File myFile;
String csv_file = "gps.txt"; // ファイル名に文字数制限があるっぽい
String log_file = "log.txt";
String backupfile = "BCKUP.txt";
const int chipSelect = 10;    // ChipSelect番号設定

// MOTOR_DRIVER
CytronMD motor1(PWM_PWM, 3, 5);   // PWM 1A = Pin 3, PWM 1B = Pin 5.
CytronMD motor2(PWM_PWM, 6, 9); // PWM 2A = Pin 6, PWM 2B = Pin 9.
int speed_low = 128; //50% speed
int speed_high = 255; //100% speed
int speed_spin = 64; //25% speed
int speed_stop = 0; //stop
int delay_time = 5000ul;
int motor_gap = 15; //motor gap, 一秒間で (モーター１の動いた距離) = (モーター２の動いた距離) + (モーター２の動いた距離)/motor_gap
// 動き出すときに尾が地面と擦れて左に曲がる癖があるので、それを考慮して修正したほうがいいかも

// OHTER
double estimated_move_distance = 0.0000127; //缶サットの推定移動距離のGPSの値に換算したもの(5秒の場合は2mで、0.0000127)
int estimated_count =0;
int allowable_gap = 3; // ゴールまでの許容誤差 /m
int turn_count = 0; //方向転換の回数のカウンター、再起処理の回数計測用
int turn_count_max = 50; //方向転換の回数のカウンターの最大値

// FUNCTIONS ///////////////////////////////////////////////////////////////
void sdcard_setsup(){
  // Serial.print("sdcard setupping ...");
  while(!SD.begin(chipSelect)){
    // Serial.println("SD card initialization failed!");
    delay(150);
  }
  // Serial.println("DONE");
}

void sdcard_write(String filename, String data){
  myFile = SD.open(filename, FILE_WRITE); // creat new file
  while (!myFile){
    // Serial.println("SD card open failed!");
    delay(20); // これを入れないとうまく書き込めない場合がある。秒数がシビアかもしれない
    myFile = SD.open(filename, FILE_WRITE);
  }
  myFile.println(data); // write contents
  myFile.close();
  delay(20); 
  myFile = SD.open(backupfile, FILE_WRITE); // creat new file
  while (!myFile){
    // Serial.println("SD card open failed!");
    delay(20); // これを入れないとうまく書き込めない場合がある。秒数がシビアかもしれない
    myFile = SD.open(filename, FILE_WRITE);
  }
  myFile.println(data); // write contents
  myFile.close();
}

String sdcard_read(String filename){
  myFile = SD.open(filename);
  if (myFile){
    while (myFile.available()){
      Serial.write(myFile.read()); // read contents
    }
    myFile.close();
  }
  else{
    // Serial.println("file open failed!");
  }
}

void move_forward(int speed, int delay_time) {
  int start_speed = 70;
  int end_speed = 50;
  for (int i = 1; i <= 10; i++) {
  motor1.setSpeed(start_speed + i*(speed-start_speed)/10);
  motor2.setSpeed(start_speed + i*(speed - motor_gap -start_speed)/10);
  delay(delay_time/8/10); // 8分の1 (delay_time)秒間に10stepで加速したいので
  }
  motor1.setSpeed(speed);   // Motor 1 runs backward.
  motor2.setSpeed((speed+speed/motor_gap));   // Motor 2 runs backward.
  delay(delay_time/2);
  for (int i = 0; i <= 10; i++) {
    motor1.setSpeed(speed - i*(speed - end_speed)/10);   // Motor 1 runs backward.
    motor2.setSpeed(speed - motor_gap - i*(speed-motor_gap - end_speed)/10);   // Motor 2 runs backward.
    delay(delay_time/8/10); // 8分の1 (delay_time)秒間に10stepで加速したいので
  }
}

// spin_leftはしっぽ展開時にも使う
void spin_left(int speed, int delay_time) {
  motor1.setSpeed(speed/2);   // Motor 1 runs forward.
  motor2.setSpeed(-(speed/2+speed/motor_gap/2));   // Motor 2 runs forward.
  delay(delay_time/8);
}

void spin_right(int speed, int delay_time) {
  motor1.setSpeed(-speed/2);   // Motor 1 runs forward.
  motor2.setSpeed(speed/2+speed/motor_gap/2);   // Motor 2 runs forward.
  delay(delay_time/8);
}

void spin_to_given_direction(int speed, double d_goal) {
  int unit_time = 5; // 回転の単位時間 /ms 
  int allowable_gap = 2; // 許容誤差 /deg
  double d_now = IMU.get_direction();
  double gap = d_goal - d_now;
  if (abs(gap) > 180) {
    gap = -360*gap/abs(gap) + gap;
  }
  for(int i = 0; i < 5; i++) {
    d_now = IMU.get_direction();
    gap = d_goal - d_now;
    if (abs(gap) > 180) {
      gap = -360*gap/abs(gap) + gap;
    }
    sdcard_write(log_file, String(millis()) + ", d_now: " + String(d_now) + " d_goal: " + String(d_goal) + " gap: " + String(gap));
    // 終了条件
    if (abs(gap) < allowable_gap) {
      stop(100);
      return;
    }
    if(gap > 0) {
      spin_right(speed, abs(gap)*unit_time);
      
    sdcard_write(log_file, String(millis()) + ",turn right. speed: " + String(speed) + ", turning time: " + String(abs(gap)*unit_time));
    }
    else {
      spin_left(speed, abs(gap)*unit_time);
      sdcard_write(log_file, String(millis()) + ",turn left. speed: " + String(speed) + ", turning time: " + String(abs(gap)*unit_time));

    }
  }
  if (turn_count > turn_count_max) {
    stop(100);
    return;
  }
  turn_count++;
  if (abs(gap) > allowable_gap) {
    spin_to_given_direction(speed, d_goal);
    return;
  }
}

void stop(int delay_time) { // intで受け取るので32767以上(32秒以上)には使えない
  motor1.setSpeed(speed_stop);   // Motor 1 runs forward.
  motor2.setSpeed(speed_stop);   // Motor 2 runs forward.
  delay(delay_time);
}


// 真北を0とした時の機体から見たゴールの角度を求める関数
double calcurate_goal_direction(double lat, double lng, double goal_lat, double goal_lng){
  double direction_for_goal;
  direction_for_goal = rad_to_deg(atan2(goal_lng - lng, goal_lat - lat));
  if (direction_for_goal < 0) {
    direction_for_goal += 360;
  }
  return direction_for_goal;
}

// GPSの値の差をメートル単位に変換する関数
double calcurate_distance(double lat, double lng, double goal_lat, double goal_lng){
  double distance;
  double lat_gap = deg_to_rad(goal_lat - lat);
  double lng_gap = deg_to_rad(goal_lng - lng);
  double r = 6378.137; // 地球の半径 /km
  distance = sqrt((pow(lat_gap*r, 2)+pow(lng_gap*r, 2))*pow(10,6));
  return distance;
}

double sign(double A){
    return (A>0)-(A<0);
}

// ARDUINO SETUP ///////////////////////////////////////////////////////////
void setup(){
  // Serial.begin(115200);
  // while (!Serial){
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }
  // Serial.println("Setupping...");
  stop(100); // これないと暴れる。原因は不明。消さないで
  LED.setup();
  LED.light_led(0, 0, 9);
  sdcard_setsup();
  LED.light_led(0, 9, 0);
  sdcard_write(log_file, String(millis()) + ", SDcard Setup is done");
  GPS.setup();
  LED.light_led(0, 9, 9);
  sdcard_write(log_file, String(millis()) + ", GPS Setup is done");
  IMU.setup();
  LED.light_led(9, 0, 0);
  sdcard_write(log_file, String(millis()) + ", IMU Setup is done");
  sdcard_write(log_file, String(millis()) + ", Setup is Over");

  // Serial.println("Setup is Over");
}

// ARDUINO LOOP ////////////////////////////////////////////////////////////
void loop(){
  sdcard_write(log_file, String(millis()) + ", < Arduino-loop Start >");
  int count;
  LED.light_led(4, 4, 4); // 待機中は黄色
  stop(delay_time);
  // 落下検出まで待機
  while(true){
    if(IMU.landing_judgement()){
      sdcard_write(log_file, String(millis()) + ", landing_judged");
      break;
    }
    else{
      delay(10);
    }
  }
  LED.light_led(4, 2, 4); // 落下検出後は黄色, 赤, 黄
  delay(1000ul*60*5);
  //#################################################################なおす5分
  LED.light_led(2, 2, 2); // 動き始める5秒前は赤色
  stop(delay_time);
  LED.light_led(5, 5, 5); // 正常動作中は緑
  sdcard_write(log_file, String(millis()) + ", Motor operation start ");

  spin_left(speed_spin, delay_time/5); // 足展開
  move_forward(speed_low, delay_time/3); // パラシュート絡まり防止のため、いったん前進


  sdcard_write(log_file, String(millis()) + ", Serch_loop Start ");
  for (count=0; count<10; count++){

    // Get datas from IMU&GPS
    direction = IMU.get_direction();
    GPS.readSenser(&lat, &lng, &meters);
    
    // calculate relative direction to goal
    double direction_for_goal = calcurate_goal_direction(lat, lng, goal_lat, goal_lng);

    // Write datas to SD card
    sdcard_write(csv_file, String(millis()) + ", " +String(count) + "," + String(lat, 6) + "," + String(lng,6) + "," + String(meters) + "," + String(direction, 6) + "," + String(direction_for_goal, 6));
    sdcard_write(log_file, String(millis()) + ", " +String(count) + "-before_turned," + String(lat, 6) + "," + String(lng,6) + "," + String(meters) + "," + String(direction, 6) + "," + String(direction_for_goal, 6));

    // GPSが取れていなかった場合の移動シミュレーション。まっすぐゴールに進んでいるものとする
    if (lat==previous_lat && lng==previous_lng){
      estimated_count++;
      double ratio = abs(goal_lat-lat) / abs(goal_lng-lng);
      // double ratio = abs(goal_lng-lng) / abs(goal_lat-lat); こっちがあってる。lng+を使う
      lat_estimated = lat + sign(goal_lat-lat)*rad_to_deg(1/(500*sqrt((pow(ratio,2) + 1)*pow(6378.137,2))))*estimated_count; //緯度経度から距離を求める計算式を逆算したら出てくる。radとdegreeの変換に注意
      //lng_estimated = lng + sign(goal_lng-lng)*ratio*rad_to_deg(1/(500*sqrt((pow(ratio,2) + 1)*pow(6378.137,2))))*estimated_count;
      lng_estimated = lng - sign(goal_lng-lng)*ratio*rad_to_deg(1/(500*sqrt((pow(ratio,2) + 1)*pow(6378.137,2))))*estimated_count;
      // lat = previous_lat + sign(goal_lat-lat)*2*pow(estimated_move_distance, 2)*cos(arctan(goal_lat-lat/goal_lng-lng))*estimated_count;
      // lng = previous_lng + sign(goal_lng-lng)*2*pow(estimated_move_distance, 2)*sin(arctan(goal_lat-lat/goal_lng-lng))*estimated_count;

      // lat = previous_lat + sign(goal_lat-lat)*estimated_move_distance*estimated_count;
      // lng = previous_lng + sign(goal_lng-lng)*estimated_move_distance*estimated_count;
      
      sdcard_write(csv_file, String(millis()) + ", " +String(count) + "-estimated," + String(lat_estimated, 6) + "," + String(lng_estimated,6) + "," + String(meters) + "," + String(direction, 6) + "," + String(direction_for_goal, 6));
      sdcard_write(log_file, String(millis()) + ", " +String(count) + "-estimated," + String(lat_estimated, 6) + "," + String(lng_estimated,6) + "," + String(meters) + "," + String(direction, 6) + "," + String(direction_for_goal, 6));
    }
    else{
      estimated_count = 0;
    }
    previous_lat = lat;
    previous_lng = lng;



    // ゴールに十分近づいていたら停止する。 
    if (calcurate_distance(lat, lng, goal_lat, goal_lng) < allowable_gap) {
      sdcard_write(log_file, String(millis()) + ", " + "Goal_judge distance is "+String(calcurate_distance(lat, lng, goal_lat, goal_lng),6));
      break;
    } 
    if (calcurate_distance(lat_estimated, lng_estimated, goal_lat, goal_lng) < allowable_gap) {
      sdcard_write(log_file, String(millis()) + ", " + "Goal_judge with estimated. distance is "+String(calcurate_distance(lat_estimated, lng_estimated, goal_lat, goal_lng),6));
      break;
    }

    // turn toward the goal
    turn_count = 0;
    spin_to_given_direction(speed_spin, direction_for_goal);
    sdcard_write(log_file, String(millis()) + ", " + String(count) + "-after_turned," + String(lat, 6) + "," + String(lng,6) + "," + String(meters) + "," + String(direction, 6) + "," + String(direction_for_goal, 6));
    // if(!IMU.horizontally_flag){
    //   sdcard_write(log_file, "NOT horizontally");
    // }
    // double IUM_data[3] = IMU.print_acc_data();
    // sdcard_write(log_file, "IMU_data_xyz" + "," +String(IMU_data[0]) + "," + String(IMU_data[1]) + "," + String(IMU_data[2]));
    // Move forward
    move_forward(speed_low, delay_time);
    sdcard_write(log_file, String(millis()) + ", " + "move_forward for " +String(delay_time)+ "ms at speed_low");
    stop(1000);
  }
  
  sdcard_write(log_file, String(millis()) + ", " + "Arrival at goal with an allowable_gap of " + allowable_gap +" meters");
  while(1){
    stop(delay_time/40);
    LED.rainbow(); // rainbowは関数内にdelayがあるのでstopが先
  }
}
