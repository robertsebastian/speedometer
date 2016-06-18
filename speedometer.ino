#include <avr/pgmspace.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <FastLED.h>

#define DISP_DATA_PIN 2
#define DISP_CLOCK_PIN 3
#define DISP_LATCH_PIN 4
#define SPEED_SENSOR_PIN 5
#define LED_ENABLE_PIN 8
#define LED_DISP_DATA_PIN 9
#define UC_LED_PIN 13

#define NUM_LEDS 51
#define NUM_SECTIONS 4
#define L_TOP_START 0
#define L_TOP_END 12
#define L_TOP_NUM 13
#define L_BACK_START 13
#define L_BACK_END 24
#define L_BACK_NUM 12
#define L_CHAIN_START 25
#define L_CHAIN_END 33
#define L_CHAIN_NUM 9
#define L_DOWN_START 34
#define L_DOWN_END 50
#define L_DOWN_NUM 17

const static float tire_radius_mm = 335.0;

static Bounce debouncer = Bounce();
static CRGB leds[NUM_LEDS];
static uint8_t displayed_number = 99; // Number displayed on nixie tubes

struct SavedConfigType {
  uint8_t check_byte;
  uint8_t led_brightness;
  uint8_t led_scale[NUM_SECTIONS];
  uint8_t led_pattern;
  uint8_t led_pattern_arg;
  int     led_animation_speed;
  uint8_t led_speed_multiplier_enable;
  uint8_t speed_enable;
};

static SavedConfigType saved = {
  .check_byte                  = 0xAD,
  .led_brightness              = 128,
  .led_scale                   = {255, 255, 255, 255},
  .led_pattern                 = 0,
  .led_pattern_arg             = 0,
  .led_animation_speed         = 20,
  .led_speed_multiplier_enable = 0,
  .speed_enable                = 1
};

// Mapping to the required bit pattern
#define LEFT_TUBE 0
#define RIGHT_TUBE 1
#define N_UNUSED_PINS 10
const static int pattern_width[2] = {10, 12};
const static int digit_pattern[][2] = {
  {0x200, 0x800}, // 0
  {0x001, 0x001}, // 1
  {0x020, 0x020}, // 2
  {0x002, 0x002}, // 3
  {0x040, 0x040}, // 4
  {0x004, 0x004}, // 5
  {0x080, 0x200}, // 6
  {0x008, 0x008}, // 7
  {0x100, 0x400}, // 8
  {0x010, 0x010}, // 9
  {0x000, 0x000} // Blank
};

void setup() {
  // Initialize speed display
  pinMode(DISP_DATA_PIN, OUTPUT);
  pinMode(DISP_CLOCK_PIN, OUTPUT);
  pinMode(DISP_LATCH_PIN, OUTPUT);
  digitalWrite(DISP_DATA_PIN, HIGH);
  digitalWrite(DISP_CLOCK_PIN, HIGH);
  digitalWrite(DISP_LATCH_PIN, HIGH);
  
  update_display(0); 

  // Initialize speed sensor
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  debouncer.attach(SPEED_SENSOR_PIN);
  debouncer.interval(5); // interval in ms

  // Determine if EEPROM is initialized and read it if so, otherwise initialize it for next time
  uint8_t check_byte;
  EEPROM.get(0, check_byte);
  if(check_byte == saved.check_byte) {
    EEPROM.get(0, saved);
  }
  
  // Initalize LEDs
  pinMode(UC_LED_PIN, OUTPUT);
  digitalWrite(UC_LED_PIN, LOW);
  
  FastLED.addLeds<WS2812, LED_DISP_DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(saved.led_brightness);  
  FastLED.show();

  // Initialize serial port
  Serial.begin(9600);

  // Indicate ready
  Serial.println(F("Ready!"));
  update_display(99);
}

void loop() {
  static unsigned long last_tick_time = millis();
  static unsigned int speed_disp = 0;
  static int last_speed_pin = 1;
  
  unsigned long curr_time = millis();
  unsigned long elapsed_time = curr_time - last_tick_time;  

  // Speedometer logic
  debouncer.update();
  int speed_pin = debouncer.read();

  if(speed_pin != last_speed_pin) {
    last_speed_pin = speed_pin;
    if(speed_pin == 0) {
      float speed_sample = 2.0 * M_PI * 335.0 / 1609344.0 / (elapsed_time / 3600000.0f);
      if(saved.speed_enable) update_display((int)speed_sample);

      last_tick_time = curr_time;
    }
  }

  if(elapsed_time >= 4000) {
    if(saved.speed_enable) update_display(0);
    //last_tick_time = curr_time;
  }

  digitalWrite(UC_LED_PIN, !speed_pin);

  // Accept serial commands over bluetooth
  if(Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print(cmd);
    
    if(cmd == 'd') {
      update_display(bt_serial_read_arg());
      
    } else if(cmd == 'e') {
      saved.speed_enable = bt_serial_read_arg();
      update_saved_config();
      
    } else if(cmd == 'b') {
      saved.led_brightness = bt_serial_read_arg();
      FastLED.setBrightness(saved.led_brightness);
      led_show_scaled();
      update_saved_config;
      
    } else if(cmd == 's') {
      for(uint8_t i = 0; i < NUM_SECTIONS; i++) {
        saved.led_scale[i] = bt_serial_read_arg();
      }      
      
      led_show_scaled();
      update_saved_config();
      
    } else if(cmd == 'p') {
      saved.led_pattern     = bt_serial_read_arg();
      saved.led_pattern_arg = bt_serial_read_arg();
      update_saved_config();
      
    } else if(cmd == 'a') {
      saved.led_animation_speed = bt_serial_read_arg();
      update_saved_config();
      
    } else if(cmd == 'm') {
      saved.led_speed_multiplier_enable = bt_serial_read_arg();
      update_saved_config();
      
    } else if(cmd == 'h') {
      Serial.print(F("\n"
        "e<1|0>: Enable speed display\n"
        "d<val>: Set speed display\n"
        "b<val>: Set LED overall brightness\n"
        "s<t>,<b>,<c>,<b>: Set LED scaling\n"
        "p<num>,<arg>: Set LED pattern\n"
        "a<val>: Set LED animation speed (0 == off)\n"
        "m<1|0>: Set LED speed multiplier enable\n"
        "\n"
        "Patterns:\n"
        "0: rainbow\n"
        "1: pulse\n"
        "2: chase\n"
        "3: chase2\n"
        "4: solid\n"
        "5: noise\n"
        "6: blinky\n"
        "7: pulse2\n"
        ));
    }
    
    // Indicate doneness
    Serial.println(".");
  }

  update_pattern();
}

// Shift a digit onto the nixie tube display
void shift_digit(int tube_num, int digit) {
  for(int i = 0; i < pattern_width[tube_num]; i++) {
    digitalWrite(DISP_CLOCK_PIN, LOW);
    digitalWrite(DISP_DATA_PIN, (digit_pattern[digit][tube_num] >> i) & 1);
    digitalWrite(DISP_CLOCK_PIN, HIGH);
  }
}

// Write number to nixie tube display
void update_display(unsigned int num)
{
  if(displayed_number == num) return;
  displayed_number = num;
  
  shift_digit(LEFT_TUBE, 10);
  shift_digit(LEFT_TUBE, (num / 10) % 10);
  shift_digit(RIGHT_TUBE, num % 10);

  digitalWrite(DISP_LATCH_PIN, HIGH);
  digitalWrite(DISP_LATCH_PIN, LOW);
  digitalWrite(DISP_LATCH_PIN, HIGH);
}

// Show LEDs, dimming each section according to a configurable pattern
void led_show_scaled()
{
  nscale8(leds + L_TOP_START,   L_TOP_NUM,   saved.led_scale[0]);
  nscale8(leds + L_BACK_START,  L_BACK_NUM,  saved.led_scale[1]);
  nscale8(leds + L_CHAIN_START, L_CHAIN_NUM, saved.led_scale[2]);
  nscale8(leds + L_DOWN_START,  L_DOWN_NUM,  saved.led_scale[3]);
  FastLED.show();
}

// Read and echo back integer arguments
int bt_serial_read_arg()
{
  int arg = Serial.parseInt();
  Serial.read(); // Consume separator
  Serial.print(arg);
  Serial.print(',');
  
  return arg;
}

// Save configuration to EEPROM after changes
void update_saved_config()
{
  EEPROM.put(0, saved);
}

// Write pattern to LED strip if required
void update_pattern()
{
  static uint8_t last_pattern = 99;
  static uint8_t last_pattern_arg = 99;
  static bool last_animated = true;
  static unsigned long last_update_time = millis();

  // Trying to make an option for animation speed to be proportional to riding speed -- not there yet
  int anim_speed = saved.led_animation_speed;
  if(saved.led_speed_multiplier_enable) {
    anim_speed -= max(anim_speed, anim_speed * (int)displayed_number / 30);
  }

  // Figure out if we need to update the animation frame
  bool animated = anim_speed != 0;
  bool stopping_animation = last_animated && !animated;
  bool new_pattern = saved.led_pattern != last_pattern || saved.led_pattern_arg != last_pattern_arg;
  bool frame_time = (millis() - last_update_time) > anim_speed;

  if(new_pattern || stopping_animation || (animated && frame_time)) {
    last_update_time = millis();

    // Update the pattern
    switch(saved.led_pattern) {
      case 0: pattern_rainbow(); break;
      case 1: pattern_pulse(); break;
      case 2: pattern_chase(); break;
      case 3: pattern_chase2(); break;
      case 4: pattern_solid(); break;
      case 5: pattern_noise(); break;
      case 6: pattern_blinky(); break;
      case 7: pattern_pulse2(); break;
      default:
        FastLED.clear();
        break;
    }
  
    led_show_scaled();
  }
  
  last_pattern = saved.led_pattern;
  last_pattern_arg = saved.led_pattern_arg;
  last_animated = animated;
}

// Blink the back section of the bike
void pattern_blinky()
{
  static bool on = true;

  FastLED.clear();
  if(on || saved.led_animation_speed == 0) {
    fill_solid(leds + L_BACK_START, L_BACK_NUM, CHSV(saved.led_pattern_arg, 255, 255));
  }
  
  on = !on;
}

// Random variations in value
void pattern_noise()
{
  for(uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(saved.led_pattern_arg, 255, random8());
  }
}

// Solid color
void pattern_solid()
{
  fill_solid(leds, NUM_LEDS, CHSV(saved.led_pattern_arg, 255, 255));
}

// Small group of LEDs circles the strip
void pattern_chase()
{
  static uint8_t i = 0;

  FastLED.clear();
  leds[(i + 0) % NUM_LEDS] = CHSV(saved.led_pattern_arg, 255, 50);
  leds[(i + 1) % NUM_LEDS] = CHSV(saved.led_pattern_arg, 255, 128);
  leds[(i + 2) % NUM_LEDS] = CHSV(saved.led_pattern_arg, 255, 255);
  leds[(i + 3) % NUM_LEDS] = CHSV(saved.led_pattern_arg, 255, 128);
  leds[(i + 3) % NUM_LEDS] = CHSV(saved.led_pattern_arg, 255, 50);
  i++;
}

// Color pulse moves to the back of the bike from the top and bottom simultaneously
void pattern_chase2()
{
  static uint8_t i = 0;
  uint8_t len = min(L_TOP_NUM + L_BACK_NUM, L_DOWN_NUM + L_CHAIN_NUM);

  FastLED.clear();
  leds[(i + 0) % len] = CHSV(saved.led_pattern_arg, 255, 128);
  leds[(i + 1) % len] = CHSV(saved.led_pattern_arg, 255, 255);
  leds[NUM_LEDS - (i + 1) % len - 1] = CHSV(saved.led_pattern_arg, 255, 255);
  leds[NUM_LEDS - (i + 0) % len - 1] = CHSV(saved.led_pattern_arg, 255, 128);
  i++;
}

// Pulse from max value to black
void pattern_pulse()
{
  static uint8_t angle;
  fill_solid(leds, NUM_LEDS, CHSV(saved.led_pattern_arg, 255, cubicwave8(angle++)));
}

// Pulse from max value to 1/4 value
void pattern_pulse2()
{
  static uint8_t angle;
  fill_solid(leds, NUM_LEDS, CHSV(saved.led_pattern_arg, 255, qadd8(cubicwave8(angle++) / 4 * 3, 64)));
}

// Show either all rainbow colors around the strip or cycle between all colors
void pattern_rainbow()
{
  static uint8_t offset = 0;
  
  if(saved.led_pattern_arg == 0) {
    fill_rainbow(leds, NUM_LEDS, offset++, 255 / NUM_LEDS);
  } else {
    fill_solid(leds, NUM_LEDS, CHSV(offset++, 255, 255));
  }
}

