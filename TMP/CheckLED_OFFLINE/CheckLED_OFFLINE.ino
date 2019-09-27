// A basic everyday NeoPixel strip test program.

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

ESP8266WiFiMulti WiFiMulti;


// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN D5
//pinMode(LED_PIN, OUTPUT);
//pixels.begin(); // This initializes the NeoPixel library.


// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 20

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

const uint16_t port = 5555;
const char * host = "192.168.43.71"; // ip or dns
// Use WiFiClient class to create TCP connections
WiFiClient client;
// setup() function -- runs once at startup --------------------------------

void setup()
{
  Serial.begin(115200);
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.
  pinMode(LED_PIN, OUTPUT);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)


}

uint32_t red_color = strip.Color(  255,   0, 0);
uint32_t green_color = strip.Color(  0,   255, 0);
uint32_t blue_color = strip.Color( 0,   0, 255);


// loop() function -- runs repeatedly as long as board is on ---------------





void turn_on_box(byte box_num, uint32_t color) //start in zero
{

  int start_index = (box_num) * 4;
  for (int i = start_index; i < start_index + 4; i++)
  {
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(10);                           //  Pause for a moment
  }

}

void turn_off_box(byte box_num) //start in zero
{

  int start_index = (box_num) * 4;
  for (int i = start_index; i < start_index + 4; i++)
  {
    strip.setPixelColor(i, 0);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(0);                           //  Pause for a moment
  }

}


void hit_box2(byte box_num, uint32_t color, int time_in_mili)
{
  int start_index = (box_num) * 4;
  for (int a = 0; a < time_in_mili / 10; a++) { // Repeat 10 times...
    for (int b = start_index; b < start_index + 4; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < start_index + 4; c += 1) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(10);  // Pause for a moment
    }
  }
}


void loop()
{

  for (int box=0; box<1; box++)
   {
  turn_on_box(box, red_color);
  delay(2000);
  hit_box2(box, green_color, 1000) ;
  turn_off_box(box);
  delay(1000);
  }
}
