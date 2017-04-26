
// Setup for all the neoPixels
const byte LEFT_STRIP  = 3;
const byte RIGHT_STRIP = 2;
const byte LEFT_RING   = 5;
const byte RIGHT_RING  = 4;

Adafruit_NeoPixel left_strip = Adafruit_NeoPixel(8, LEFT_STRIP,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel right_strip = Adafruit_NeoPixel(8, RIGHT_STRIP,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel left_ring = Adafruit_NeoPixel(12, LEFT_RING,NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel right_ring = Adafruit_NeoPixel(12, RIGHT_RING,NEO_GRBW + NEO_KHZ800);

uint32_t white = left_strip.Color(0,0,0,255);
uint32_t yellow= left_strip.Color(255,128,0);
uint32_t red   = left_strip.Color(255,0,0,0);
uint32_t off   = left_strip.Color(0,0,0,0);
uint32_t blue  = left_strip.Color(0,64,255,0);
uint32_t purple= left_strip.Color(255,0,255,0);

unsigned long blink_time;
byte blinked = 0;
int delay_period = 500; // Blinking speed for alive light


// Initialize lights
void setup_lights() {
  
  // Setup all the NeoPixels
  left_strip.begin();
  right_strip.begin();
  left_ring.begin();
  right_ring.begin();
  
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
  
  left_strip.setBrightness(16);
  right_strip.setBrightness(16);
  left_ring.setBrightness(16);
  right_ring.setBrightness(16);
}


// Blink all NeoPixels on and off
void blink_lights() {

  current_time = millis();
  
  if(current_time - blink_time > delay_period){
    blinked = !blinked;
    
    // Logic to do turning/stopped lights
    if(blinked){
      if (linear_vel == 0 && angular_vel == 0 && ir_estop == 1)
        change_all_colors(purple);
      else if (linear_vel == 0 && angular_vel == 0)
        change_all_colors(red);
      else if (ir_estop == 1)
        change_all_colors(blue);
      else if (angular_vel < -3){ // turning left
        if(linear_vel>=0)
          change_left_colors(white);
        else
          change_left_colors(yellow);
      }
      else if (angular_vel > 3){ // turning right
        if(linear_vel>=0)
          change_right_colors(white);
        else
          change_right_colors(yellow);
      }
      else if (linear_vel >= 0)
        change_all_colors(white);
      else
        change_all_colors(yellow);
    }
    else
      change_all_colors(off);
    
    blink_time = millis();
  }
}


// Helper function to make all NeoPixels a given color
void change_all_colors(uint32_t color) {
  
  for (int i = 0; i < 8; i++)
  {
    left_strip.setPixelColor(i,color);
    right_strip.setPixelColor(i,color);
  }
  for (int i = 0; i < 12; i++)
  {
    left_ring.setPixelColor(i,color);
    right_ring.setPixelColor(i,color);
  }
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
}


// Helper functions for right and left banks of lights
void change_left_colors(uint32_t color) {
  
  for (int i = 0; i < 8; i++)
  {
    left_strip.setPixelColor(i,color);
    right_strip.setPixelColor(i,off);
  }
  for (int i = 0; i < 12; i++)
  {
    left_ring.setPixelColor(i,color);
    right_ring.setPixelColor(i,off);
  }
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
}


void change_right_colors(uint32_t color) {
  
  for (int i = 0; i < 8; i++)
  {
    left_strip.setPixelColor(i,off);
    right_strip.setPixelColor(i,color);
  }
  for (int i = 0; i < 12; i++)
  {
    left_ring.setPixelColor(i,off);
    right_ring.setPixelColor(i,color);
  }
  left_strip.show();
  right_strip.show();
  left_ring.show();
  right_ring.show();
}


// Check to see if the physical E-stop is pressed and change light behavior based on result
void read_estop() {
  
  if (digitalRead(ESTOP_PIN)) {
    delay_period = 500;
  }
  else {
    delay_period = 150;
  }
}
