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

#define MAX_CURRENT_MA 400

#define arr_len(arr) (sizeof(arr) / sizeof(arr[0]))

const static float tire_radius_mm = 335.0;

static Bounce debouncer = Bounce();
static CRGB leds[NUM_LEDS];
static uint8_t displayed_number = 99; // Number displayed on nixie tubes
static uint8_t speed_mph = 0;
static uint8_t speed_norm = 0; // Speed mapping 0-20 mph to 0-255
static uint8_t layer_idx = 0;
static volatile bool speed_sensor_triggered = false;
static bool layers_updated = false;
static bool made_it_through_one_loop = false;

#define N_PATTERN_ARGS 3
#define N_PATTERN_LAYERS 4

struct PatternLayer {
  uint8_t num;
  uint8_t arg[N_PATTERN_ARGS];
  int     speed;
  uint8_t step;

  PatternLayer() : speed(20), step(1) {}
};

struct SavedConfigType {
  uint8_t      check_byte;
  uint8_t      led_brightness;
  uint8_t      led_scale[NUM_SECTIONS];
  PatternLayer layers[N_PATTERN_LAYERS];
  uint8_t      speed_enable;

  SavedConfigType() :
    check_byte(0xA8),
    led_brightness(128),
    led_scale({255, 255, 255, 255}),
    speed_enable(1)
  {}

  void save() {
    EEPROM.put(0, *this);    
  }

  // Determine if EEPROM is initialized and read it if so, otherwise initialize it for next time
  void load() {
    uint8_t read_check_byte, read_completed_loop_last_time;
    EEPROM.get(0, read_check_byte);
    if(read_check_byte == check_byte) {
      EEPROM.get(0, *this);
    } else {
      save();
    }
  }

  void print_state() {
    Serial.print('b');
    Serial.println(led_brightness);
    Serial.print('s');
    for(uint8_t i = 0; i < arr_len(led_scale); i++) {
      Serial.print(led_scale[i]);
      Serial.print(',');
    }
    Serial.print('\n');
    
    for(uint8_t layer_idx = 0; layer_idx < N_PATTERN_LAYERS; layer_idx++) {
      const PatternLayer & layer = layers[layer_idx];

      Serial.print('p');
      Serial.print(layer_idx);
      Serial.print(',');
      Serial.print(layer.num);
      for(uint8_t i = 0; i < N_PATTERN_ARGS; i++) {
        Serial.print(',');
        Serial.print(layer.arg[i]);
      }
      Serial.print('\n');

      Serial.print('a');
      Serial.print(layer_idx);
      Serial.print(',');
      Serial.println(layer.speed);
      
      Serial.print('t');
      Serial.print(layer_idx);
      Serial.print(',');
      Serial.println(layer.step);
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
  typedef void (*PatternFunc)(uint8_t & anim_idx, const PatternLayer & layer);
  
  PatternFunc func;
  const __FlashStringHelper *name;

  PatternInfo() : func(NULL), name(NULL) {}
  PatternInfo(PatternFunc func_, const __FlashStringHelper *name_) : func(func_), name(name_) {}
};

static PatternInfo patterns[17];

void setup() {
  patterns[0]  = PatternInfo(pattern_rainbow,    F("Rainbow"));
  patterns[1]  = PatternInfo(pattern_rainbow2,   F("Rainbow 2,saturation,0,255,value,0,255"));
  patterns[2]  = PatternInfo(pattern_pulse,      F("Pulse,range,0,255"));
  patterns[3]  = PatternInfo(pattern_chase,      F("Chase circle,hue,0,255,length,0,51"));
  patterns[4]  = PatternInfo(pattern_chase2,     F("Chase dual,hue,0,255,length,0,26"));
  patterns[5]  = PatternInfo(pattern_solid,      F("Solid HSV,hue,0,255,saturation,0,255,value,0,255"));
  patterns[6]  = PatternInfo(pattern_solid_rgb,  F("Solid RGB,red,0,255,green,0,255,blue,0,255"));
  patterns[7]  = PatternInfo(pattern_noise,      F("Noise,magnitude,0,255"));
  patterns[8]  = PatternInfo(pattern_blinky,     F("Blinky,hue,0,255,saturation,0,255,pulse width,0,255"));
  patterns[9]  = PatternInfo(overlay_sparkle,    F("Sparkle,number,1,5,hue,0,255,saturation,0,255"));
  patterns[10] = PatternInfo(overlay_speed_mult, F("Brightness speed overlay,min,0,255,max,0,255"));
  patterns[11] = PatternInfo(overlay_anim_speed, F("Animation speed overlay,layer,0,3,min,0,255,max,0,255"));
  patterns[12] = PatternInfo(pattern_murica,     F("Murica,length,1,17"));
  patterns[13] = PatternInfo(overlay_speed_fake, F("Fake Speed,normalized speed,0,255"));
  patterns[14] = PatternInfo(pattern_ants,       F("Marching ants,hue,0,255,length on,0,30,length off,0,30"));
  patterns[15] = PatternInfo(pattern_dual_ants,  F("Marching ants 2,length,1,17,hue 1,0,255,hue 2,0,255"));
  patterns[16] = PatternInfo(pattern_rainbow3,   F("Rainbow 3,length,1,7"));
  // !!! UPDATE LENGTH OF PATTERNS ARRAY IF YOU ADD HERE !!!
  
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

  //for(int i = 0; i < EEPROM.length(); i++) EEPROM.write(i, 0);

  // Init configuration
  saved.load();
  
  // Initalize LEDs
  FastLED.addLeds<WS2812, LED_DISP_DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MAX_CURRENT_MA);
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

  update_layers();
  process_inputs();
}

void process_inputs()
{
  static bool needs_save = false;
  static uint16_t last_save_time = 0;
  
  // Accept serial commands over bluetooth
  if(Serial.available() > 0) {
    char cmd = Serial.read();
    
    if(cmd == 'd') { // Set speed display
      update_display(bt_serial_read_arg());
      
    } else if(cmd == 'e') { // Enable speed display
      saved.speed_enable = bt_serial_read_arg();
      
    } else if(cmd == 'b') {
      saved.led_brightness = bt_serial_read_arg();
      FastLED.setBrightness(saved.led_brightness);
      led_show_scaled();
      
    } else if(cmd == 's') { // Set LED brightness scaling
      for(uint8_t i = 0; i < NUM_SECTIONS; i++) {
        saved.led_scale[i] = bt_serial_read_arg();
      }      
      
      led_show_scaled();
      
    } else if(cmd == 'p') { // Set LED pattern
      uint8_t layer = bt_serial_read_arg() % N_PATTERN_LAYERS;
      saved.layers[layer].num = bt_serial_read_arg();
      for(uint8_t i = 0; i < N_PATTERN_ARGS; i++) {
        saved.layers[layer].arg[i] = bt_serial_read_arg();
      }

      layers_updated = true;
    } else if(cmd == 'a') { // Set LED animation speed
      uint8_t layer = bt_serial_read_arg() % N_PATTERN_LAYERS;
      saved.layers[layer].speed = bt_serial_read_arg();

    } else if(cmd == 't') { // Set LED animation step
      uint8_t layer = bt_serial_read_arg() % N_PATTERN_LAYERS;
      saved.layers[layer].step = bt_serial_read_arg();

    } else if(cmd == 'l') { // List patterns
      Serial.println(F("l0,off"));
      for(uint8_t i = 0; i < arr_len(patterns); i++) {
        Serial.print('l');
        Serial.print(i + 1);
        Serial.print(',');
        Serial.println(patterns[i].name);
      }

    } else if(cmd == 'c') { // Show current configuration
      saved.print_state();
      
    } else if(cmd == 'h') { // Help
      Serial.print(F("\n"
        "e<1|0>: Enable speed display\n"
        "d<val>: Set speed display\n"
        "b<val>: Set LED overall brightness\n"
        "s<t>,<b>,<c>,<b>: Set LED scaling\n"
        "p<layer>,<num>,<arg1>,<arg2>,<arg3>: Set LED pattern\n"
        "a<layer>,<val>: Set LED animation speed (0 == off)\n"
        "t<layer>,<val>: Set LED animatino step\n"
        "c: Show current configuration\n"
        "l: Show patterns\n"
        ));
    }

    // Acknowledge message
    Serial.println('.');

    // Save after commands
    needs_save = true;
  }

  // To avoid burning out our EEPROM with rapid slider changes only commit changes every second
  if(needs_save && last_save_time != seconds16()) {
    saved.save();
    needs_save = false;
    last_save_time = seconds16();
  }
}

void update_speed()
{
  static unsigned long last_tick_time = millis();
  
  unsigned long curr_time = millis();
  unsigned long elapsed_time = curr_time - last_tick_time;  

  if(speed_sensor_triggered) {
    speed_sensor_triggered = false;

    // Based on 700C wheel and 25mm tire
    float speed_calculated = 2.0 * M_PI * 335.0 / 1609344.0 / (elapsed_time / 3600000.0f);
    speed_mph = (uint8_t)speed_calculated;
    speed_norm = (uint8_t)min(speed_calculated / 20.0 * 255.0, 255.0);
    
    last_tick_time = curr_time;
  }

  if(elapsed_time >= 4000) {
    speed_mph = 0;
    speed_norm = 0;
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
  static unsigned long last_update_time = 0;
  
  if(displayed_number == num) return;

  unsigned long curr_time = millis();  
  if(curr_time < last_update_time + 100) return;

  last_update_time = curr_time;
  
  if(displayed_number > num) {
    displayed_number--;
  } else {
    displayed_number++;
  }
  
  shift_digit(LEFT_TUBE, 10);
  if(displayed_number == 255) {
    // If 255 show blank display
    shift_digit(LEFT_TUBE, 10);
    shift_digit(RIGHT_TUBE, 10);    
  } else {
    shift_digit(LEFT_TUBE, (displayed_number / 10) % 10);
    shift_digit(RIGHT_TUBE, displayed_number % 10);
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
  
  return arg;
}

// Write pattern to LED strip if required
void update_layers()
{
  static unsigned long last_update_time[N_PATTERN_LAYERS];
  static uint8_t anim_idx[N_PATTERN_LAYERS];

  // Determine if an update is required for any layer
  bool update_required = layers_updated;
  for(uint8_t layer_idx = 0; layer_idx < N_PATTERN_LAYERS; layer_idx++) {
    const PatternLayer & layer = saved.layers[layer_idx];
    
    // Nothing to do if this is an empty layer
    if(layer.num < 1 || layer.num > arr_len(patterns)) continue;
    
    update_required = update_required || (millis() - last_update_time[layer_idx]) > layer.speed;
  }

  // If no layer requires an update, nothing to do
  if(!update_required) return;

  for(layer_idx = 0; layer_idx < N_PATTERN_LAYERS; layer_idx++) {
    const PatternLayer & layer = saved.layers[layer_idx];

    // Nothing to do if this is an empty layer
    if(layer.num < 1 || layer.num > arr_len(patterns)) continue;

    // Update animation step if requred
    if(millis() - last_update_time[layer_idx] > layer.speed) {
      last_update_time[layer_idx] = millis();
      anim_idx[layer_idx] += layer.step;
    }

    // Update the pattern
    patterns[layer.num - 1].func(anim_idx[layer_idx], layer);
  }

  led_show_scaled();
  layers_updated = false;  
}

// Blink the back section of the bike
void pattern_blinky(uint8_t & anim_idx, const PatternLayer & layer)
{
  if(squarewave8(anim_idx, layer.arg[2])) {
    fill_solid(leds + L_BACK_START, L_BACK_NUM, CHSV(layer.arg[0], layer.arg[1], 255));
  }
}

// Random variations in value
void pattern_noise(uint8_t & anim_idx, const PatternLayer & layer)
{
  static uint8_t last_idx = anim_idx;
  static uint8_t scale[NUM_LEDS];

  if(anim_idx != last_idx) {
    for(uint8_t i = 0; i < NUM_LEDS; i++) {
      scale[i] = random8(layer.arg[0]);
    }
    last_idx = anim_idx;
  }

  for(uint8_t i = 0; i < NUM_LEDS; i++) {
    //CHSV color = leds[i];
    leds[i].nscale8_video(255 - scale[i]);
  }
}

// Solid color
void pattern_solid(uint8_t & anim_idx, const PatternLayer & layer)
{
  fill_solid(leds, NUM_LEDS, CHSV(layer.arg[0], layer.arg[1], layer.arg[2]));
}

void pattern_solid_rgb(uint8_t & anim_idx, const PatternLayer & layer)
{
  fill_solid(leds, NUM_LEDS, CRGB(layer.arg[0], layer.arg[1], layer.arg[2]));
}

// Small group of LEDs circles the strip
void pattern_chase(uint8_t & anim_idx, const PatternLayer & layer)
{
  if(anim_idx >= NUM_LEDS) anim_idx = 0;
  
  uint8_t hue = layer.arg[0];
  uint8_t len = min(layer.arg[1], NUM_LEDS);

  for(uint8_t i = 0; i < len; i++) {
    leds[(anim_idx + i) % NUM_LEDS] = CHSV(hue, 255, 255);
  }
}

// Color pulse moves to the back of the bike from the top and bottom simultaneously
void pattern_chase2(uint8_t & anim_idx, const PatternLayer & layer)
{
  uint8_t max_top = L_TOP_NUM + L_BACK_NUM;
  uint8_t max_bottom = L_DOWN_NUM + L_CHAIN_NUM;
  uint8_t max_len = min(max_top, max_bottom);
  uint8_t hue = layer.arg[0];
  uint8_t len = min(layer.arg[1], max_len);

  if(anim_idx >= max_len) anim_idx = 0;

  for(uint8_t i = 0; i < len; i++) {
    leds[(anim_idx + i) % max_top] = CHSV(hue, 255, 255);
    leds[NUM_LEDS - (anim_idx + i) % max_bottom - 1] = CHSV(hue, 255, 255);
  }
}

// Pulse from max value to 1/4 value
void pattern_pulse(uint8_t & anim_idx, const PatternLayer & layer)
{
  nscale8(leds, NUM_LEDS, map8(cubicwave8(anim_idx), layer.arg[0], 255));
}

// Show either all rainbow colors around the strip or cycle between all colors
void pattern_rainbow(uint8_t & anim_idx, const PatternLayer & layer)
{
  fill_rainbow(leds, NUM_LEDS, anim_idx, 255 / NUM_LEDS);
}

void pattern_rainbow2(uint8_t & anim_idx, const PatternLayer & layer)
{
  fill_solid(leds, NUM_LEDS, CHSV(anim_idx, layer.arg[0], layer.arg[1]));
}

void pattern_multi_band(uint8_t anim_idx, const PatternLayer & layer, CRGB pattern[], uint8_t n_colors, uint8_t pat_len)
{
  pat_len = constrain(pat_len, 1, NUM_LEDS);
    
  for(uint8_t i = 0, pat_idx = 0; i < NUM_LEDS; i++) {
    if(i % pat_len == 0 && NUM_LEDS - i > pat_len / 2) {
      pat_idx = (pat_idx + 1) % n_colors;
    }

    CRGB color = pattern[pat_idx];
    if(color.r != 0 || color.g != 0 || color.b != 0) {
      leds[(i + anim_idx) % NUM_LEDS] = color;
    }
  }
}

void pattern_murica(uint8_t & anim_idx, const PatternLayer & layer) {
  CRGB pattern[] = {CRGB::Red, CRGB::White, CRGB::Blue};
  
  pattern_multi_band(anim_idx, layer, pattern, arr_len(pattern), layer.arg[0]);
}

void pattern_dual_ants(uint8_t & anim_idx, const PatternLayer & layer) {
  CRGB pattern[] = {CHSV(layer.arg[1], 255, 255), CHSV(layer.arg[2], 255, 255), CRGB::Black};
  
  pattern_multi_band(anim_idx, layer, pattern, arr_len(pattern), layer.arg[0]);
}

void pattern_rainbow3(uint8_t & anim_idx, const PatternLayer & layer) {
  CRGB pattern[] = {CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Indigo, CRGB::Violet};
  
  pattern_multi_band(anim_idx, layer, pattern, arr_len(pattern), layer.arg[0]);
}

void pattern_ants(uint8_t & anim_idx, const PatternLayer & layer) {
  uint8_t hue = layer.arg[0];
  uint8_t on_len = layer.arg[1];
  uint8_t off_len = layer.arg[2];

  if(anim_idx >= NUM_LEDS) anim_idx = 0;

  for(uint8_t i = 0; i < NUM_LEDS; i += on_len + off_len) {
    for(uint8_t j = 0; j < on_len; j++) {
      leds[(i + j + anim_idx) % NUM_LEDS] = CHSV(hue, 255, 255);
    }
  }
}

void overlay_speed_mult(uint8_t & anim_idx, const PatternLayer & layer)
{
  uint8_t scale = dim8_raw(scale8(speed_norm, layer.arg[1] - layer.arg[0]) + layer.arg[0]);
  for(uint8_t i = 0; i < NUM_LEDS; i++) {
    leds[i].nscale8(scale);
  }
}

void overlay_speed_fake(uint8_t & anim_idx, const PatternLayer & layer)
{
  speed_norm = layer.arg[0];
}

void overlay_sparkle(uint8_t & anim_idx, const PatternLayer & layer)
{
  static const uint8_t MAX_SPARKLES = 5;
  static uint8_t last_anim_idx[N_PATTERN_LAYERS];
  static uint8_t last_leds[N_PATTERN_LAYERS][MAX_SPARKLES];
  uint8_t n_sparkles = constrain(layer.arg[0], 1, MAX_SPARKLES);
  
  if(anim_idx != last_anim_idx[layer_idx]) {
    for(uint8_t i = 0; i < n_sparkles; i++) {
      last_leds[layer_idx][i] = random8(NUM_LEDS);
    }
    last_anim_idx[layer_idx] = anim_idx;
  }

  for(uint8_t i = 0; i < n_sparkles; i++) {
    leds[last_leds[layer_idx][i]] = CHSV(layer.arg[1], layer.arg[2], 255);
  }
}

void overlay_sparkle2(uint8_t & anim_idx, const PatternLayer & layer)
{
  static uint8_t last_anim_idx = 0;
  static uint8_t last_led = 0;
  if(anim_idx != last_anim_idx) {
    last_led = random8(NUM_LEDS);
    last_anim_idx = anim_idx;
  }

  leds[last_led] = CRGB::White;  
}

void overlay_anim_speed(uint8_t & anim_idx, const PatternLayer &layer) {
  static uint8_t last_anim_idx;

  if(anim_idx == last_anim_idx) return;
  last_anim_idx = anim_idx;
  
  uint8_t tgt_layer = layer.arg[0] % N_PATTERN_LAYERS;
  uint16_t range_low = layer.arg[1];
  uint16_t range_high = layer.arg[2];

  saved.layers[tgt_layer].speed = (range_high - range_low) * (255 - speed_norm) / 256 + range_low;
}

