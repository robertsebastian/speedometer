#include <avr/pgmspace.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <PinChangeInterrupt.h>

#define DISP_DATA_PIN 2
#define DISP_CLOCK_PIN 3
#define DISP_LATCH_PIN 4
#define SPEED_SENSOR_PIN 5
#define LED_ENABLE_PIN 8
#define LED_DISP_DATA_PIN 9

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

#define arr_len(arr) (sizeof(arr) / sizeof(arr[0]))

const static float tire_radius_mm = 335.0;

static Bounce debouncer = Bounce();
static CRGB leds[NUM_LEDS];
static uint8_t displayed_number = 99; // Number displayed on nixie tubes
static uint8_t speed_mph = 0;
static uint8_t speed_norm = 0; // Speed mapping 0-20 mph to 0-255
static volatile bool speed_sensor_triggered = false;
static bool layers_updated = false;

#define N_PATTERN_ARGS 2
#define N_PATTERN_LAYERS 3

struct PatternLayer {
  uint8_t num;
  uint8_t arg[N_PATTERN_ARGS];
};

struct SavedConfigType {
  uint8_t      check_byte;
  uint8_t      led_brightness;
  uint8_t      led_scale[NUM_SECTIONS];
  PatternLayer layers[N_PATTERN_LAYERS];
  int          led_animation_speed;
  uint8_t      led_animation_step;
  uint8_t      led_speed_multiplier_enable;
  uint8_t      speed_enable;

  SavedConfigType() :
    check_byte(0xA1),
    led_brightness(128),
    led_scale({255, 255, 255, 255}),
    led_animation_speed(20),
    led_animation_step(1),
    led_speed_multiplier_enable(0),
    speed_enable(1)
  {}

  void save() {
    EEPROM.put(0, *this);    
  }

  // Determine if EEPROM is initialized and read it if so, otherwise initialize it for next time
  void load() {
    uint8_t read_check_byte;
    EEPROM.get(0, read_check_byte);
    if(read_check_byte == check_byte) {
      EEPROM.get(0, *this);
    } else {
      save();
    }
  }
};
static SavedConfigType saved;

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


struct PatternInfo {
  typedef void (*PatternFunc)(uint8_t anim_idx, const PatternLayer & layer);
  
  PatternFunc func;
  const __FlashStringHelper *name;

  PatternInfo() : func(NULL), name(NULL) {}
  PatternInfo(PatternFunc func_, const __FlashStringHelper *name_) : func(func_), name(name_) {}
};

static PatternInfo patterns[12];

void setup() {
  patterns[0]  = PatternInfo(pattern_rainbow,  F("Rainbow"));
  patterns[1]  = PatternInfo(pattern_rainbow2, F("Rainbow 2"));
  patterns[2]  = PatternInfo(pattern_pulse,    F("Pulse"));
  patterns[3]  = PatternInfo(pattern_pulse2,   F("Pulse 2")); 
  patterns[4]  = PatternInfo(pattern_chase,    F("Chase"));
  patterns[5]  = PatternInfo(pattern_chase2,   F("Chase 2"));
  patterns[6]  = PatternInfo(pattern_solid,    F("Solid"));
  patterns[7]  = PatternInfo(pattern_noise,    F("Noise"));
  patterns[8]  = PatternInfo(pattern_blinky,   F("Blinky"));
  patterns[9]  = PatternInfo(overlay_sparkle,  F("Sparkle"));
  patterns[10] = PatternInfo(overlay_sparkle2, F("Sparkle 2"));
  patterns[11] = PatternInfo(overlay_speed_mult, F("Speed overlay"));
  
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
  attachPinChangeInterrupt(digitalPinToPCINT(SPEED_SENSOR_PIN), handle_speed_sensor_pin_int, CHANGE);
  //debouncer.attach(SPEED_SENSOR_PIN);
  //debouncer.interval(5); // interval in ms

  // Init configuration
  saved.load();
  
  // Initalize LEDs
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
  update_speed();

  if(saved.speed_enable) {
    update_display(speed_mph);
  }

  update_pattern();
  process_inputs();
}

void process_inputs()
{
  // Accept serial commands over bluetooth
  if(Serial.available() > 0) {
    char cmd = Serial.read();
    Serial.print(cmd);
    
    if(cmd == 'd') {
      //update_display(bt_serial_read_arg());
      
    } else if(cmd == 'e') {
      saved.speed_enable = bt_serial_read_arg();
      saved.save();
      
    } else if(cmd == 'b') {
      saved.led_brightness = bt_serial_read_arg();
      FastLED.setBrightness(saved.led_brightness);
      led_show_scaled();
      saved.save();
      
    } else if(cmd == 's') {
      for(uint8_t i = 0; i < NUM_SECTIONS; i++) {
        saved.led_scale[i] = bt_serial_read_arg();
      }      
      
      led_show_scaled();
      saved.save();
      
    } else if(cmd == 'p') {
      uint8_t layer  = bt_serial_read_arg() % N_PATTERN_LAYERS;
      saved.layers[layer].num = bt_serial_read_arg();
      for(uint8_t i = 0; i < N_PATTERN_ARGS; i++) {
        saved.layers[layer].arg[i] = bt_serial_read_arg();
      }
      saved.save();

      layers_updated = true;
    } else if(cmd == 'a') {
      saved.led_animation_speed = bt_serial_read_arg();
      saved.save();
      
    } else if(cmd == 'm') {
      saved.led_speed_multiplier_enable = bt_serial_read_arg();
      saved.save();

    } else if(cmd == 't') {
      saved.led_animation_step = bt_serial_read_arg();
      saved.save();

    } else if(cmd == 'l') {
      Serial.println(F("0:off"));
      for(uint8_t i = 0; i < arr_len(patterns); i++) {
        Serial.print(i + 1);
        Serial.print(':');
        Serial.println(patterns[i].name);
      }
      
    } else if(cmd == 'h') {
      Serial.print(F("\n"
        "e<1|0>: Enable speed display\n"
        "d<val>: Set speed display\n"
        "b<val>: Set LED overall brightness\n"
        "s<t>,<b>,<c>,<b>: Set LED scaling\n"
        "p<layer>,<num>,<arg1>,<arg2>: Set LED pattern\n"
        "a<val>: Set LED animation speed (0 == off)\n"
        "m<1|0>: Set LED speed multiplier enable\n"
        "l: Show patterns\n"
        ));
    }
    
    // Indicate doneness
    Serial.println(".");
  }
}

void update_speed()
{
  static unsigned long last_tick_time = millis();
  
  unsigned long curr_time = millis();
  unsigned long elapsed_time = curr_time - last_tick_time;  

  if(speed_sensor_triggered) {
    speed_sensor_triggered = false;
    
    float speed_calculated = 2.0 * M_PI * 335.0 / 1609344.0 / (elapsed_time / 3600000.0f);
    speed_mph = (uint8_t)speed_calculated;
    speed_norm = (uint8_t)min(speed_calculated / 20.0 * 255.0, 255.0);
    
    last_tick_time = curr_time;
  }

  if(elapsed_time >= 4000) {
    speed_mph = 0;
  }
}

void handle_speed_sensor_pin_int()
{
  static unsigned long last_trigger_time = 0;
  unsigned long elapsed_time = millis() - last_trigger_time;
  
  if(digitalRead(SPEED_SENSOR_PIN) == LOW && elapsed_time > 20) {
    speed_sensor_triggered = true;
  }
  if(digitalRead(SPEED_SENSOR_PIN) == LOW) {
    last_trigger_time = millis();
  }
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
void update_display(uint8_t num)
{
  if(displayed_number == num) return;
  displayed_number = num;
  
  shift_digit(LEFT_TUBE, 10);
  if(num == 255) {
    // If 255 show blank display
    shift_digit(LEFT_TUBE, 10);
    shift_digit(RIGHT_TUBE, 10);    
  } else {
    shift_digit(LEFT_TUBE, (num / 10) % 10);
    shift_digit(RIGHT_TUBE, num % 10);
  }

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

// Write pattern to LED strip if required
void update_pattern()
{
  static bool last_animated = true;
  static unsigned long last_update_time = millis();
  static uint8_t anim_idx;

  // Trying to make an option for animation speed to be proportional to riding speed -- not there yet
  int anim_speed = saved.led_animation_speed;
  if(saved.led_speed_multiplier_enable) {
    anim_speed -= max(anim_speed, anim_speed * (int)displayed_number / 30);
  }

  // Figure out if we need to update the animation frame
  bool animated = anim_speed != 0;
  bool stopping_animation = last_animated && !animated;
  bool frame_time = (millis() - last_update_time) > anim_speed;

  if(layers_updated || stopping_animation || (animated && frame_time)) {
    last_update_time = millis();
    anim_idx += saved.led_animation_step;

    // Update the pattern
    FastLED.clear();
    for(uint8_t i = 0; i < N_PATTERN_LAYERS; i++) {
      uint8_t layer = saved.layers[i].num;
      if(layer > 0 && layer <= arr_len(patterns)) {
        patterns[layer - 1].func(anim_idx, saved.layers[i]);
      }
    }
    
    led_show_scaled();
    
    layers_updated = false;    
  }
  last_animated = animated;
}

// Blink the back section of the bike
void pattern_blinky(uint8_t anim_idx, const PatternLayer & layer)
{
  static bool on = true;

  if(on || saved.led_animation_speed == 0) {
    fill_solid(leds + L_BACK_START, L_BACK_NUM, CHSV(layer.arg[0], 255, 255));
  }
  
  on = !on;
}

// Random variations in value
void pattern_noise(uint8_t anim_idx, const PatternLayer & layer)
{
  for(uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(layer.arg[0], 255, random8());
  }
}

// Solid color
void pattern_solid(uint8_t anim_idx, const PatternLayer & layer)
{
  fill_solid(leds, NUM_LEDS, CHSV(layer.arg[0], 255, 255));
}

// Small group of LEDs circles the strip
void pattern_chase(uint8_t anim_idx, const PatternLayer & layer)
{
  uint8_t hue = layer.arg[0];
  leds[(anim_idx + 0) % NUM_LEDS] = CHSV(hue, 255, 50);
  leds[(anim_idx + 1) % NUM_LEDS] = CHSV(hue, 255, 128);
  leds[(anim_idx + 2) % NUM_LEDS] = CHSV(hue, 255, 255);
  leds[(anim_idx + 3) % NUM_LEDS] = CHSV(hue, 255, 128);
  leds[(anim_idx + 3) % NUM_LEDS] = CHSV(hue, 255, 50);
}

// Color pulse moves to the back of the bike from the top and bottom simultaneously
void pattern_chase2(uint8_t anim_idx, const PatternLayer & layer)
{
  uint8_t len = min(L_TOP_NUM + L_BACK_NUM, L_DOWN_NUM + L_CHAIN_NUM);

  leds[(anim_idx + 0) % len] = CHSV(layer.arg[0], 255, 128);
  leds[(anim_idx + 1) % len] = CHSV(layer.arg[0], 255, 255);
  leds[NUM_LEDS - (anim_idx + 1) % len - 1] = CHSV(layer.arg[0], 255, 255);
  leds[NUM_LEDS - (anim_idx + 0) % len - 1] = CHSV(layer.arg[0], 255, 128);
}

// Pulse from max value to black
void pattern_pulse(uint8_t anim_idx, const PatternLayer & layer)
{
  fill_solid(leds, NUM_LEDS, CHSV(layer.arg[0], 255, cubicwave8(anim_idx++)));
}

// Pulse from max value to 1/4 value
void pattern_pulse2(uint8_t anim_idx, const PatternLayer & layer)
{
  fill_solid(leds, NUM_LEDS, CHSV(layer.arg[0], 255, qadd8(cubicwave8(anim_idx++) / 4 * 3, 64)));
}

// Show either all rainbow colors around the strip or cycle between all colors
void pattern_rainbow(uint8_t anim_idx, const PatternLayer & layer)
{
  fill_rainbow(leds, NUM_LEDS, anim_idx++, 255 / NUM_LEDS);
}

void pattern_rainbow2(uint8_t anim_idx, const PatternLayer & layer)
{
  fill_solid(leds, NUM_LEDS, CHSV(anim_idx++, 255, 255));
}

void overlay_speed_mult(uint8_t anim_idx, const PatternLayer & layer)
{
  for(uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8(dim8_raw(speed_norm));
  }
}

void overlay_sparkle(uint8_t anim_idx, const PatternLayer & layer)
{
  leds[random8(NUM_LEDS)] = CRGB::White;
}

void overlay_sparkle2(uint8_t anim_idx, const PatternLayer & layer)
{
  leds[random8(NUM_LEDS)] = CRGB::White;
  leds[random8(NUM_LEDS)] = CRGB::White;
}
