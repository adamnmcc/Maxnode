/*
   Message format is "state,brightness,red,green,blue
*/

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <FastLED.h>
#include <DHT.h>

/* ***************************************************** */
/* WiFi settings */
const char* ssid     = "TeamAwesome";  // Put your WiFi SSID here
const char* password = "CaptainQueenPrincessPrince";  // Put your WiFi password here

//const char* ssid     = "RSA-Guest";  // Put your WiFi SSID here
//const char* password = "fire1794";  // Put your WiFi password here

const char* server = "m21.cloudmqtt.com";

/* FreePixel LED settings */
#define NUM_LEDS 16                // How many leds in your strip?
// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the WS2801 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 2
//#define CLOCK_PIN D1
uint32_t ledPosition = 0;
/* RGB values are set in the MQTT callback and then referenced when LED colour is set */
int red ;
int green;
int blue;
int brightness;
volatile bool updateLights = false;
CRGB leds[NUM_LEDS];    // Define the array of leds


/* Humidity sensor settings */
#define DHTPIN 5
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
unsigned long timeLater = 0;

/* input button settings currentl plugged into D2 on Elecrow NodeMCU Dev Board */
const int button = 4;


/* MQTT Settings */
String ledstatusTopic   = "sensor/maxnode/status";       // MQTT topic
String colourTopic   = "sensor/maxnode/leds";       // MQTT topic
String brightnessTopic   = "sensor/maxnode/brightness";       // MQTT topic
String humidTopic = "sensor/maxnode/humidity";    // MQTT topic
String tempTopic  = "sensor/maxnode/temperature"; // MQTT topic
String command;
String statestring;
String state = "off";


#define BUFFER_SIZE 100
#define CLIENT_ID "maxnode"

unsigned long previousMillis;     

// Initialize global variables for sequences
uint8_t thisdelay = 50;                                        // A delay value for the sequence(s)
uint8_t thishue = 0;                                          // Starting hue value.
uint8_t deltahue = 10;                                        // Hue change between pixels.
uint8_t  thisfade = 8;                                        // How quickly does it fade? Lower = slower fade rate.
uint8_t   thisinc = 1;                                        // Incremental value for rotating hues
uint8_t   thissat = 100;                                      // The saturation, where 255 = brilliant colours.
uint8_t   thisbri = 255;                                      // Brightness of a sequence. Remember, max_bright is the overall limiter.
int       huediff = 256;                                      // Range of random #'s to use for hue


WiFiClient wificlient;
PubSubClient client(wificlient, server);

/* ***************************************************** */
/**
   MQTT callback to process messages
*/

uint8_t gHue = 0; 

void callback(const MQTT::Publish& pub)
{
  if (pub.has_stream())
  {
    uint8_t buf[BUFFER_SIZE];
    int read;
    while (read = pub.payload_stream()->read(buf, BUFFER_SIZE))
    {
      Serial.write(buf, read);
    }
    pub.payload_stream()->stop();
    Serial.println("");
  } else
    

  if (pub.payload_string() .equals("Rainbow"))
    {command = pub.payload_string();
      
    }
  else if (pub.payload_string().equals("Confetti"))
    {command = pub.payload_string();
      
    }
  else {
  command = pub.payload_string();
  int i_state                 = pub.payload_string().indexOf(',');
  int i_brightness            = pub.payload_string().indexOf(',', i_state + 1);
  int i_red                   = pub.payload_string().indexOf(',',i_brightness + 1);                  
  int i_green                 = pub.payload_string().indexOf(',',i_red + 1); 
  int i_blue                  = pub.payload_string().indexOf(',',i_green + 1); 

  state = pub.payload_string().substring(0,i_state); 
  if (pub.payload_string().substring(i_state+1,i_brightness).toInt() == 0   && state == "on") 
  { brightness = brightness; } else 
  { brightness = pub.payload_string().substring(i_state+1,i_brightness).toInt(); }

  if (pub.payload_string().substring(i_brightness+1) == ",,")

  {
   command = pub.payload_string().substring(i_brightness+1);
   red = red;
   green = green;
   blue = blue;
   }
  else {
    command = pub.payload_string().substring(i_brightness+1);
  red = pub.payload_string().substring(i_brightness+1, i_red).toInt(); 
  green = pub.payload_string().substring(i_red+1,i_green).toInt();
  blue = pub.payload_string().substring(i_green+1,i_blue).toInt();
  }
  
     
//  if ( pub.payload_string().substring(i_brightness+1, i_red).toInt() == 0  && state.equals("on")) 
//  { red = red+1; } else
//  { red = pub.payload_string().substring(i_brightness+1, i_red).toInt(); }

//  if ( pub.payload_string().substring(i_red+1,i_green).toInt() == 0  && state.equals("on")) 
//  { green = green+1; } else
//  { green = pub.payload_string().substring(i_red+1,i_green).toInt(); }

//  if ( pub.payload_string().substring(i_green+1,i_blue).toInt() == 0  && state.equals("on")) 
//  { blue = blue+1; } else
//  {blue = pub.payload_string().substring(i_green+1,i_blue).toInt();}
  
  }

  Serial.print("Message: ");
  Serial.println(command);
  Serial.print("State: ");
  Serial.println(state);
  Serial.print("Brightness: ");
  Serial.println(brightness);
  Serial.print("Red: ");
  Serial.println(red);
  Serial.print("Green: ");
  Serial.println(green);
  Serial.print("Blue: ");
  Serial.println(blue);
  
  updateLights = true;
  
}

/**
   Relies on 4 global variables being set: "ledPosition", "red", "green", and "blue"
*/


void publishState() {
//  statestring = printf("%s,%d,%d,%d,%d", state, brightness, red, green, blue);
  statestring = state+','+brightness+','+red+','+green+','+blue;
  client.publish(ledstatusTopic , statestring);
}

void confetti() {                                             // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(leds, NUM_LEDS, thisfade);                    // Low values = slower fade.
  int pos = random16(NUM_LEDS);                               // Pick an LED at random.
  leds[pos] += CHSV((thishue + random16(huediff))/4 , thissat, thisbri);  // I use 12 bits for hue so that the hue increment isn't too quick.
  thishue = thishue + thisinc;                                // It increments here.
} // confetti()


void ChangeMe() {                                             // A time (rather than loop) based demo sequencer. This gives us full control over the length of each sequence.
  uint8_t secondHand = (millis() / 1000) % 50;                // IMPORTANT!!! Change '15' to a different value to change duration of the loop.
  static uint8_t lastSecond = 99;                             // Static variable, means it's only defined once. This is our 'debounce' variable.
  if (lastSecond != secondHand) {                             // Debounce to make sure we're not repeating an assignment.
    lastSecond = secondHand;
    switch(secondHand) {
      case  0: thisinc=1; thishue=192; thissat=255; thisfade=2; huediff=256; break;  // You can change values here, one at a time , or altogether.
      case  5: thisinc=2; thishue=128; thisfade=8; huediff=64; break;
      case 10: thisinc=1; thishue=random16(255); thisfade=1; huediff=16; break;      // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
      case 15: break;                                                                // Here's the matching 15 for the other one.
    }
  }
}

void rainbow() 
{

  thishue++;
  fill_rainbow(leds, NUM_LEDS, thishue, deltahue);

}



void setColor()
{
  if (updateLights)
  {
    updateLights = false;

   
    fill_solid( leds, NUM_LEDS, CRGB(green,red,blue));
    FastLED.setBrightness(brightness);    
    FastLED.show();
    publishState();
    

    
//    }
  }
}


/**
   Setup
*/
void setup() {
  pinMode(button,INPUT); 
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("WiFi begun");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Proceeding");

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if      (error == OTA_AUTH_ERROR   ) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR  ) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR    ) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
  //FastLED.setBrightness(128);
  //set_max_power_in_volts_and_milliamps(5, 500);       

  // Scan red, then green, then blue across the LEDs
//  for (int i = 0; i <= NUM_LEDS; i++)
//  {
//    leds[i] = CRGB::Red;
//    FastLED.show();
//    delay(200);
//    leds[i] = CRGB::Black;
//    FastLED.show();
//  }
//  for (int i = 0; i <= NUM_LEDS; i++)
//  {
//    leds[i] = CRGB::Green;
//    FastLED.show();
//    delay(200);
//    leds[i] = CRGB::Black;
//    FastLED.show();
//  }
//  for (int i = 0; i <= NUM_LEDS; i++)
//  {
//    leds[i] = CRGB::Blue;
//    FastLED.show();
//    delay(200);
//    leds[i] = CRGB::Black;
//    FastLED.show();
//  }
}

/**
   Main
*/
void loop() {
  ArduinoOTA.handle();
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, password);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      client.set_server(server, 14352);
      if (client.connect(MQTT::Connect("maxnode")
                       .set_auth("maxnode", "Anm4ck14781"))) {
        client.set_callback(callback);
        client.subscribe(MQTT::Subscribe()
                         .add_topic(colourTopic));
      }
    }
  }

  unsigned long timeNow = millis();
  if (timeNow >= timeLater) {
    timeLater = timeNow + 60000;

    float humidity    = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Below from stackxchange.com
    char tempC[10];
    dtostrf(temperature, 1, 2, tempC);
    char relH[10];
    dtostrf(humidity, 1, 2, relH);

    client.publish(MQTT::Publish(tempTopic, tempC).set_qos(1));
    //client.publish("test/humid", temperature);
    client.publish(MQTT::Publish(humidTopic, relH).set_qos(1));

    Serial.print("T: ");
    Serial.print(temperature, DEC);
    Serial.print("C H:");
    Serial.print(humidity, DEC);
    Serial.println("%");
    
  }

    if(digitalRead(button)){
  Serial.print("Button Pressed");
  }
 

  if (client.connected())
    client.loop();
    
   if (command.equals("Rainbow")) {
        EVERY_N_MILLISECONDS(thisdelay) {                           // FastLED based non-blocking routine to update/display the sequence.
          rainbow(); }
        show_at_max_brightness_for_power(); 
        }

    else if (command.equals("Confetti")) {
        EVERY_N_MILLISECONDS(thisdelay) { confetti();}
        show_at_max_brightness_for_power(); 
        }
    else
    {  setColor();    }

//rainbow();
      
//  }
}
