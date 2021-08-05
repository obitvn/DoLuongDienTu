/* This code works with MAX30102 + 128x32 OLED i2c + Buzzer and Arduino UNO
 * It's displays the Average BPM on the screen, with an animation and a buzzer sound
 * everytime a heart pulse is detected
 * It's a modified version of the HeartRate library example
 * Refer to www.surtrtech.com for more details or SurtrTech YouTube channel
 */

#include <Adafruit_GFX.h>        //OLED libraries
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "SPI.h"
#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include "pitches.h"
#include <WiFi.h>           // WiFi control for ESP32
#include <ThingsBoard.h>    // ThingsBoard SDK
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Tone32.h>

#define BUZZER_PIN 25
#define BUZZER_CHANNEL 0



MAX30105 particleSensor;

// WiFi access point
#define WIFI_AP_NAME        "Obit"
// WiFi password
#define WIFI_PASSWORD       "obitteam"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/ 
// to understand how to obtain an access token
#define TOKEN               "lyUM08VtVIJRxt5egoFa"
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "demo.thingsboard.io"
// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;
unsigned long check_wifi = 30000;
bool subscribed = false;
bool state = true;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,              //Logo2 and Logo3 are two bmp pictures that display on the OLED if called
0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,  };

static const unsigned char PROGMEM logo3_bmp[] =
{ 0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00  };


MPU6050 accelgyro;

void setup() {  
  pinMode(27,0);
  pinMode(14,0);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(300);
  // Initialize sensor
  Serial.begin(115200);
  particleSensor.begin(Wire, I2C_SPEED_FAST); //Use default I2C port, 400kHz speed
  //particleSensor.setup(0x1F, 4,3, 255, 200,4096); //Configure sensor with default settings
  particleSensor.setup(0x1F, 4,3, 255, 369,4096); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(8); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeIR(0x1f); //Turn Red LED to low to indicate sensor is running
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
    /* Default settings from datasheet. */
  accelgyro.initialize();
    if (!bmp.begin()) {
      delay(1);
  }
  Serial.println("Init wifi");
  tone(BUZZER_PIN, NOTE_B4, 500, BUZZER_CHANNEL);
  delay(2);
  noTone(BUZZER_PIN, BUZZER_CHANNEL);
  delay(2);
  tone(BUZZER_PIN, NOTE_B4, 500, BUZZER_CHANNEL);
  delay(2);
  noTone(BUZZER_PIN, BUZZER_CHANNEL);
  delay(2);
}

uint8_t scan=0;
int16_t ax, ay, az;
int16_t gx, last_gx, last_gy,  gy, last_gz, gz;
uint32_t count_g;
void loop() {
  if((digitalRead(27)==0)||(digitalRead(14)==0))
  {
    
    while((digitalRead(27)==0)||(digitalRead(14)==0))
    {
    display.clearDisplay(); 
    Serial.println("CALL DOCTOR");
    display.setTextSize(1.5);                                   //Near it display the average BPM you can display the BPM if you want
    display.setTextColor(WHITE); 
    display.setCursor(00,35);                
    display.println("CALL DOCTOR"); 
    display.display();
    tone(BUZZER_PIN, NOTE_B4, 500, BUZZER_CHANNEL);
    if(digitalRead(27)==0)
    {
    tb.sendTelemetryInt("call_doctor",12345);
    }
    if(digitalRead(14)==0)
    {
    tb.sendTelemetryInt("call_doctor",56734);
    }
    }
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    
  }
  count_g++;
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  if(abs(last_gx - gx)>12000||abs(last_gy - gy)>12000||abs(last_gz - gz)>12000)
  {
    count_g=0;
    Serial.println("clear wake up check");
  }
  else count_g++;

  if(count_g>1000)
  {
    for(int i=0; i<15; i++)
    {
    tone(BUZZER_PIN, NOTE_B4, 500, BUZZER_CHANNEL);
    display.clearDisplay(); 
    Serial.println("wake up check");
    display.setTextSize(2);                                   //Near it display the average BPM you can display the BPM if you want
    display.setTextColor(WHITE); 
    display.setCursor(00,35);                
    display.println("  wake up"); 
    display.display();
    delay(500);
    display.clearDisplay(); 
    Serial.println("clear screen");
    display.setTextSize(2);                                   //Near it display the average BPM you can display the BPM if you want
    display.setTextColor(WHITE); 
    display.setCursor(00,35);                
    display.println("       "); 
    display.display();
    delay(200);
    }
    noTone(BUZZER_PIN, BUZZER_CHANNEL);
    count_g=0;
  }
   delay(50);
  if(beatAvg>5)
  {
    if ((WiFi.status() != WL_CONNECTED) && (millis() > check_wifi))  {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    for(int i=0; i<5; i++)
    {
    digitalWrite(25,!digitalRead(25));
    delay(100);
    }
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    check_wifi = millis() + 30000;
    digitalWrite(25,0);
  }
  if (!tb.connected()) {
    subscribed = false;
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
  } 
  tb.loop();
  }

long irValue = particleSensor.getRed();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
//Serial.println(irValue);                                          //Also detecting a heartbeat
last_gx = gx;
last_gy = gy;
last_gz = gz;

if(irValue > 7000){                                           //If a finger is detected
    display.clearDisplay();                                   //Clear the display
    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);       //Draw the first bmp picture (little heart)
    if(scan)
    {
      display.setTextSize(1);                                   //Near it display the average BPM you can display the BPM if you want
      display.setTextColor(WHITE); 
      display.setCursor(45,3);                
      display.println("reading..."); 
      delay(500);
    }
    if((beatAvg>5)||(scan=0))
    {   
    scan=0;
    display.setTextSize(1);                                   //Near it display the average BPM you can display the BPM if you want
    display.setTextColor(WHITE); 
    display.setCursor(40,0);                
    display.println("BPM");             
    display.setCursor(90,0);  
    display.println(beatAvg); 
    
    display.setCursor(40,15);                
    display.println("X");             
    display.setCursor(90,15);  
    display.println(gx); 
    
    display.setCursor(40,30);                
    display.println("Y");             
    display.setCursor(90,30);  
    display.println(gy); 

    display.setCursor(40,45);                
    display.println("Z");             
    display.setCursor(90,45);  
    display.println(gz); 
    }
    
    display.display();
    
  if (checkForBeat(irValue) == true)                        //If a heart beat is detected
  {
    scan=0;
    /*
    display.clearDisplay();                                //Clear the display
    display.drawBitmap(0, 0, logo3_bmp, 32, 32, WHITE);    //Draw the second picture (bigger heart)
    display.setTextSize(1);                                //And still displays the average BPM
    display.setTextColor(WHITE);             
    display.setCursor(40,0);                
    display.println("BPM");             
    display.setCursor(90,0);  
    display.println(beatAvg+50); 
    
    display.setCursor(40,30);                
    display.println("TEMP");             
    display.setCursor(50,30);  
    display.println(temp_event.temperature); 
    
    display.setCursor(40,60);                
    display.println("PRES");             
    display.setCursor(90,60);  
    display.println(pressure_event.pressure); 

    
    tb.sendTelemetryInt("beat",beatAvg+50);
    display.display();                                         //Deactivate the buzzer to have the effect of a "bip"
    */
    //We sensed a beat!
    long delta = millis() - lastBeat;                   //Measure duration between two beats
    lastBeat = millis();
    beatsPerMinute = 58 / (delta / 1000.0);           //Calculating the BPM
    if (beatsPerMinute < 255 && beatsPerMinute > 2)               //To calculate the average we strore some values (4) then do some math to calculate the average
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
    tb.sendTelemetryFloat("temp",bmp.readTemperature());
    tb.sendTelemetryFloat("pressure",bmp.readPressure());
    tb.sendTelemetryInt("beat",beatAvg);
  }

}
  if (irValue < 7000){       //If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
     beatAvg=0;
     display.clearDisplay();
     display.setTextSize(1);                    
     display.setTextColor(WHITE);             
     display.setCursor(30,5);                
     display.println("Please wear it"); 
     display.setCursor(30,15);
     display.println("on your wrist");  
     display.display();
     scan=1;
     //noTone(3);
     }
     
  
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  
}

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      //WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}
