#include "LED.h"


LED::LED(int led_count, int din_pin, int neo_pixel_type) : Adafruit_NeoPixel(led_count, din_pin, neo_pixel_type){}

void LED::setup(){
  LED::begin();
}

void LED::convert_num_to_color(int color_num, int color[3]){
  switch(color_num){
    case 0: // black
      color[0] = 0;
      color[1] = 0;
      color[2] = 0;
      break;
    case 1: // pink
      color[0] = 10;
      color[1] = 2;
      color[2] = 2;
      break;
    case 2: // red
      color[0] = 10;
      color[1] = 0;
      color[2] = 0;
      break;
    case 3: // orange
      color[0] = 10;
      color[1] = 2;
      color[2] = 0;
      break;
    case 4: // yellow
      color[0] = 10;
      color[1] = 10;
      color[2] = 0;
      break;
    case 5: // green
      color[0] = 0;
      color[1] = 10;
      color[2] = 5;
      break;
    case 6: // blue
      color[0] = 0;
      color[1] = 0;
      color[2] = 10;
      break;
    case 7: // purple
      color[0] = 10;
      color[1] = 0;
      color[2] = 10;
      break;
    case 8: // sky blue
      color[0] = 0;
      color[1] = 2;
      color[2] = 10;
      break;
    case 9: // white
      color[0] = 10;
      color[1] = 10;
      color[2] = 10;
      break;
  }
}

void LED::light_led(int color_num_0, int color_num_1, int color_num_2){
  int color_num[3] = {color_num_0, color_num_1, color_num_2};
  int color_rgb[3];
  
  LED::clear();
  for(int i = 0; i < 3; i++){
    convert_num_to_color(color_num[i], color_rgb);
    LED::setPixelColor(i, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
  }
  LED::show();
}

void LED::rainbow(){
  int rgb_sum = 255;
  int color_rgb[3];
  int delay_time = 3;
  for (int i = 0; i < rgb_sum; i++) {
    color_rgb[0] = rgb_sum - i;
    color_rgb[1] = i;
    color_rgb[2] = 0;
    LED::clear();
    LED::setPixelColor(0, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::setPixelColor(1, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::setPixelColor(2, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::show();
    delay(delay_time);
    }
  for (int i = 0; i < rgb_sum; i++) {
    color_rgb[0] = 0;
    color_rgb[1] = rgb_sum - i;
    color_rgb[2] = i;
    LED::clear();
    LED::setPixelColor(0, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::setPixelColor(1, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::setPixelColor(2, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::show();
    delay(delay_time);
    }
  for (int i = 0; i < rgb_sum; i++) {
    color_rgb[0] = i;
    color_rgb[1] = 0;
    color_rgb[2] = rgb_sum - i;
    LED::clear();
    LED::setPixelColor(0, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::setPixelColor(1, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::setPixelColor(2, LED::Color(color_rgb[0], color_rgb[1], color_rgb[2]));
    LED::show();
    delay(delay_time);
    }
}