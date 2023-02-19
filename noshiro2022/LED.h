#ifndef _LED_h
#define _LED_h

#include <Adafruit_NeoPixel.h>

class LED : public Adafruit_NeoPixel{
  // LED(int led_count, int din_pin, int neo_pixel_type);
  public :
    LED(int led_count, int din_pin, int neo_pixel_type);
    void setup();
    void light_led(int color_num_0, int color_num_1, int color_num_2);
    void rainbow();
  private :
    void convert_num_to_color(int color_num, int color[3]);
    
};
#endif
