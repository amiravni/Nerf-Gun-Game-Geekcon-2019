#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#define LED_PIN D6
#define LED_COUNT 240
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const char* ssid     = "BestProjectEver";
const char* password = "1223334444";

WiFiUDP UDPTestServer;
unsigned int UDPPort = 5555;

const int packetSize = 300;
byte packetBuffer[packetSize];


uint32_t red_color = strip.Color(  255,   0, 0);
uint32_t green_color = strip.Color(  0,   255, 0);
uint32_t blue_color = strip.Color( 0,   0, 255);



void turn_on_box(byte box_num, uint32_t color) //start in zero
{

  int start_index = (box_num) * 4;
  for (int i = start_index; i < start_index + 4; i++)
  {
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    //strip.show();                          //  Update strip to match
    //delay(10);                           //  Pause for a moment
  }

}

void turn_off_box(byte box_num) //start in zero
{

  int start_index = (box_num) * 4;
  for (int i = start_index; i < start_index + 4; i++)
  {
    strip.setPixelColor(i, 0);         //  Set pixel's color (in RAM)
    //strip.show();                          //  Update strip to match
    //delay(0);                           //  Pause for a moment
  }

}

void hit_box2(byte box_num, uint32_t color, int time_in_mili)
{
  int start_index = (box_num) * 4;
  for (int a = 0; a < time_in_mili / 10; a++) { // Repeat 10 times...
    for (int b = start_index; b < start_index + 4; b++) { //  'b' counts from 0 to 2...
      //strip.clear();         //   Set all pixels in RAM to 0 (off)
      turn_off_box(box_num);
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < start_index + 4; c += 1) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(10);  // Pause for a moment
    }
  }
  turn_off_box(box_num);
}



void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);
  //WiFi.config(IPAddress(192, 168, 1, 60), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected"); 
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  UDPTestServer.begin(UDPPort);

  pinMode(LED_PIN, OUTPUT);

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(255); // Set BRIGHTNESS to about 1/5 (max = 255)
 
}

int value = 0;

void loop() {
   handleUDPServer();
   delay(1);
}

void handleUDPServer() {
  int cb = UDPTestServer.parsePacket();
  if (cb) {
    Serial.println(cb);
    UDPTestServer.read(packetBuffer, packetSize);
    String myData = "";
    for(int i = 0; i < cb; i++) {
      myData += (char)packetBuffer[i];
      Serial.println((int)myData[i]);
      delay(0);
    }
    for(int i = 0; i < cb; i=i+5) {
      Serial.println(i);
      
      byte boxId = (byte)myData[i];
      int opCode = (int)myData[i + 1];
      int r = (int)myData[i + 2];
      int g = (int)myData[i + 3];
      int b = (int)myData[i + 4];
      uint32_t color = strip.Color(  r,   g, b);
      switch (opCode) {
        case 0:
          turn_on_box(boxId,color);
          Serial.println("ON!");
          break;
        case 1:
          turn_off_box(boxId);
          break;
        case 2:
          hit_box2(boxId, color, 100);
          break;
        default:
          break;
      }
      if (opCode > 10) {
         hit_box2(boxId, color, opCode*10);
      }
      delay(0);
      
    }
    Serial.println("Done!");
    //Serial.println(myData);
    strip.show(); // Update strip with new contents
    delay(10);  // Pause for a moment
    
  }
}
