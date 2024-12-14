#include <Arduino.h>
// Fix Parameters
// Possible Values for Serial_Print_Mode  ! DONT TOUCH !
//
// Skatch to measure the outdoor termperature with a DS1820 onewire temp sensor
// send the temperature via TTN and forward it to my home server
//

#define   Serial_None            0  // No Serial Printout
#define   Serial_Info            1
#define   Serial_Error           2  // Only debug and error output will be printed via RS232(USB)
#define   Serial_Debug           3  // Log measurement as table via RS232(USB)

// Usable CPU-Types
// WIFI -> Heltev Wifi Kit 32
#define WIFI 0
// LORA  ->  Heltec Wifi Lora 32 (V2)
#define LORA 1
// STICK ->  Heltec Wireless Stick  (has LoRa on board)

//
// Includes
//====================================================================================================================================
#include "userdefines.h"
//==================================================================================================================================
//========oled display===========
#include <U8x8lib.h>
//=================================================================================================
#include "main.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>


//====================================================================================================================================

//====================================================================================================================================
// define gyros sensor data

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;


static BLEUUID BLESERVICE_UUID("230d70f2-278b-45c2-b3f3-377d44643933");
#define SERVICE_UUID          "230d70f2-278b-45c2-b3f3-377d44643933"
#define CHARACTERISTIC_UUID_1 "beb5483e-36e1-4688-b7f5-ea07361b26a8" // temperature
#define CHARACTERISTIC_UUID_2 "00af5a43-9709-4ba5-8d74-5abe36bfce11" // humidity
#define CHARACTERISTIC_UUID_3 "bb15e05d-209b-4ae4-bba5-7dc7475b560c" // pressure
#define CHARACTERISTIC_UUID_4 "b106da7e-901c-4d94-b10e-7fed5cf2d2ff" // gps longtitude
#define CHARACTERISTIC_UUID_5 "f20f4f86-5e9e-45ae-b47a-5c1f5f33bac2" // gps latitude
#define CHARACTERISTIC_UUID_6 "c1b64825-8f9d-43cb-bb1d-df8cfd2ca71d" // gps altitude
#define CHARACTERISTIC_UUID_7 "dc293512-3587-43fd-9c4d-c9fc45ab2c02" // gps sat
#define CHARACTERISTIC_UUID_8 "3d6f94b2-9c32-4d62-8f5d-c9620d1a4874" // gps speed


BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic_1 = NULL;
BLECharacteristic* pCharacteristic_2 = NULL;
BLECharacteristic* pCharacteristic_3 = NULL;
BLECharacteristic* pCharacteristic_4 = NULL;
BLECharacteristic* pCharacteristic_5 = NULL;
BLECharacteristic* pCharacteristic_6 = NULL;
BLECharacteristic* pCharacteristic_7 = NULL;
BLECharacteristic* pCharacteristic_8 = NULL;
BLEDescriptor *pDescr_1;
BLEDescriptor *pDescr_2;
BLEDescriptor *pDescr_3;
BLEDescriptor *pDescr_4;
BLEDescriptor *pDescr_5;
BLEDescriptor *pDescr_6;
BLEDescriptor *pDescr_7;
BLEDescriptor *pDescr_8;

BLE2902 *pBLE2902_1;
BLE2902 *pBLE2902_2;
BLE2902 *pBLE2902_3;
BLE2902 *pBLE2902_4;
BLE2902 *pBLE2902_5;
BLE2902 *pBLE2902_6;
BLE2902 *pBLE2902_7;
BLE2902 *pBLE2902_8;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
// bmp280 sensor
#include "thp_sensor.h"
//  wifi / html server

#include <WiFi.h>
#include <WiFiServer.h> 
//#include <ESPAsynchWebServer.h> //hs002
#include <HTTPClient.h> //001hs
WiFiServer server(80);
const char* serverName = "http://192.168.178.121:3000/"; //001hs

//hs001
HTTPClient http; //001hs
WiFiClient client1; //001hs
String my_Api_Key = "dHRudHJhY2tlcjpZODlBdzRtY3NH";




//==================================================================================================================================
// inclue Lora support, oled display, real time clock and GPS
//==================================================================================================================================
#include "loraWan.h"
// oled display
#include "display.h"
// realtime clock
#include "RTClib.h"
// gps stuff
#include <TinyGPS++.h> 


char          ss_id[30];
float currentVoltage=0;
uint8_t tx_payload[18];

//==========================================================================================
// BLE Callback function
//==========================================================================================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// TTN Send message interval for BPM280 sensor 60 sec - for gps data 20 sec(default 5min = 300 sec)) [sec]

#define TTN_MESSAGING_INTERVAL_BPM 60
#define TTN_MESSAGING_INTERVAL_GPS 20

static unsigned long transmission_timestamp_bpm = millis(); 
static unsigned long transmission_timestamp_gps = millis();

int ttn_messaging_interval_factor = 6;
int ttn_loop_cntr=0;


char rec_buffer[25]; //copy the uplink message into this buffer
int rec_buffer_len;
int lora_txt_completion=0;

//==================================================================================================================================
// rtc support 
//==================================================================================================================================
DateTime now;
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//==================================================================================================================================
// read the values from BME280/BME680 module 
//==================================================================================================================================
void read_THP(unsigned long current_ms, bool *have_thp, float *temperature, float *humidity, float *pressure) {
  *have_thp = read_thp_sensor(temperature, humidity, pressure);
}
//==================================================================================================================================
// Define the GPS Interface
//==================================================================================================================================
#define RXD2 23
#define TXD2 17
HardwareSerial neogps(1);  
TinyGPSPlus gps;
double lat=12.3456, lng=6.5432; //alt, speed, course;
double previous_lat = 0;
double previous_lng = 0;
// for string comparisor with fix 3 decimal ppints
char lat_str[8], lng_str[8], previous_lng_str[8], previous_lat_str[8];
static float  temperature = 12.34, humidity = 75.1, pressure =250.12, course=11.22, speed=22.33, alt=444.55;

float bpm_buffer[10][5];
float gps_buffer[10][5];

int bpm_array_cntr=0;
int gps_array_cntr=0;
uint32_t sat;

//==================================================================================================================================
// get the chip id 
//==================================================================================================================================
unsigned long getESPchipID() {
  uint64_t espid = ESP.getEfuseMac();
  uint8_t *pespid = (uint8_t*)&espid;
  uint32_t id = 0;
  uint8_t *pid = (uint8_t *)&id;
  pid[0] = (uint8_t)pespid[5];
  pid[1] = (uint8_t)pespid[4];
  pid[2] = (uint8_t)pespid[3];
  Serial.printf("ID: %08X\n", id);
  Serial.printf("MAC: %04X%08X\n",(uint16_t)(espid>>32),(uint32_t)espid);
  return id;
}

//==================================================================================================================================
// send BPM280 data to TTN
//==================================================================================================================================
void sendBMEData2TTN(int event, int tbd, uint32_t unixtime, float temperature, float humidity, float pressure, float currentVoltage) {
 
  unsigned char ttnData[30];
  for (int i=0; i < 30; i++) {
     ttnData[i] = 0x00;
  };
  
  ttnData[0] = event;
  ttnData[1] = tbd;

  // get rid of the comma
  uint32_t temp_Binary = (temperature+100) * 100;  // to avoid comma and negativ values
  uint32_t humidityBinary = humidity * 100;
  uint32_t pressureBinary = pressure * 100;
  
  uint32_t currentVoltage_Binary = ((currentVoltage) * 100);
  
  ttnData[2] = ( temp_Binary >> 8 ) & 0xFF;
  ttnData[3] = temp_Binary & 0xFF;

  ttnData[4] = ( humidityBinary >> 8 ) & 0xFF;
  ttnData[5] = humidityBinary & 0xFF;

  ttnData[6] = ( pressureBinary >> 16 ) & 0xFF;
  ttnData[7] = ( pressureBinary >> 8 ) & 0xFF;
  ttnData[8] = pressureBinary & 0xFF;

  ttnData[9] = (currentVoltage_Binary >> 8) & 0xFF;
  ttnData[10] = currentVoltage_Binary  & 0xFF;

  ttnData[11] = (unixtime >> 32) & 0xFF;
  ttnData[12] = (unixtime >> 24) & 0xFF;
  ttnData[13] = (unixtime >> 16) & 0xFF;
  ttnData[14] = (unixtime >> 8) & 0xFF;
  ttnData[15] = unixtime  & 0xFF;

  int cnt = 16;
  lorawan_send(1,ttnData,cnt,false,NULL,NULL,NULL);
}
//==================================================================================================================================
// send the GPS data to TTN
//==================================================================================================================================
void sendGPSData2TTN(int event, int tbd, uint32_t unixtime, double lat, double lng, float alt, uint32_t sat, float speed) {
 
  unsigned char ttnData[30];
  for (int i=0; i < 30; i++) {
     ttnData[i] = 0x00;
  };
  
  ttnData[0] = event;
  ttnData[1] = tbd;

  // get rid of the comma;

  uint32_t lat_Binary = (lat+200) * 10000;  //to avoid comma and negativ values
  uint32_t lng_Binary = (lng+200) * 10000;

  uint32_t alt_Binary = alt *100;
  uint32_t sat_Binary = sat;
  uint32_t speed_Binary = speed*100;


  ttnData[2] = ( lat_Binary >> 16 ) & 0xFF;
  ttnData[3] = ( lat_Binary >> 8 ) & 0xFF;
  ttnData[4] = lat_Binary & 0xFF;
  
  ttnData[5] = ( lng_Binary >> 16 ) & 0xFF;
  ttnData[6] = ( lng_Binary >> 8 ) & 0xFF;
  ttnData[7] = lng_Binary & 0xFF;

  ttnData[8] = (alt_Binary >> 8) & 0xFF;
  ttnData[9] = alt_Binary  & 0xFF;

  ttnData[10] = (sat_Binary >> 8) & 0xFF;
  ttnData[11] = sat_Binary  & 0xFF;
  
  ttnData[12] = (speed_Binary >> 8) & 0xFF;
  ttnData[13] = speed_Binary  & 0xFF;


  ttnData[14] = (unixtime >> 32) & 0xFF;
  ttnData[15] = (unixtime >> 24) & 0xFF;
  ttnData[16] = (unixtime >> 16) & 0xFF;
  ttnData[17] = (unixtime >> 8) & 0xFF;
  ttnData[18] = unixtime  & 0xFF;

  int cnt = 19;
  lorawan_send(1,ttnData,cnt,false,NULL,NULL,NULL);
}
//==================================================================================================================================
// mpu6050  routine - the gyros sensor
//==================================================================================================================================
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}



void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}

//****************************************************************************************************
//*** setup routine
//****************************************************************************************************
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println( "Let's go!");
  uint32_t xx = getESPchipID();

  // build SSID
  sprintf(ss_id,"ESP32-%d",xx);
  Serial.println(ss_id);
  //==================================================================================================================================
  // start GPS modul
  //==================================================================================================================================
  Serial.println("start gps modul");
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
 
  //==================================================================================================================================
  // start oled display
  //==================================================================================================================================
  Serial.println("calling setup_display");
  setup_display();
  delay(1000);
  //==================================================================================================================================
  // Initialize real time clock and ads1115
  //==================================================================================================================================
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    delay(1000);
  }
  //rtc.adjust(DateTime(2021, 1, 11, 18, 06, 0));
  now = rtc.now();
  Serial.print("now time = ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date &amp; time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date &amp; time, for example to set
    // January 21, 2014 at 3am you would call:
   rtc.adjust(DateTime(2021, 1, 12, 11, 19, 0));
  }
  //==================================================================================================================================
  // init LoRa
  //==================================================================================================================================
  Serial.println("calling lorawan setup");
  lorawan_setup();
  //==================================================================================================================================
  // init bme280 sensor
  //==================================================================================================================================
  Serial.println("calling setup BME sensor");
  setup_thp_sensor();
  //==================================================================================================================================
  // battery power
  //==================================================================================================================================
  adcAttachPin(37);
  //==================================================================================================================================
  // internal LED
  //==================================================================================================================================
  pinMode(25, OUTPUT);
  //==================================================================================================================================
  // setup wifi html server
  //  in this code I need a wifi connection to a router (wlan) or to the smartphone
  //  the access point will be defined in this network 
  //  there is another posibility where the esp32 itself opens an access point - this is not implemented here
  //==================================================================================================================================
  Serial.println("calling wifi setup");
  // hs002 comment it out
    
  const char* ssid = "...."; //ssid of your wifi
  const char* password = "...."; //password of your wifi

  WiFi.begin(ssid, password); //connecting to wifi
  int i=0;
  do {
    delay(1000);
    i++;
    Serial.print(".");
  } while ((i<10) & (WiFi.status() != WL_CONNECTED));
    
  if (WiFi.status() != WL_CONNECTED)// while wifi not connected
  {
    Serial.println("WiFi not connected!!"); 
  } else {
    Serial.println("");
    Serial.println("WiFi connected");
    server.begin();
    Serial.println("Server started");
    Serial.println(WiFi.localIP());  // Print the IP address
    delay(1000);
  

    //001hs  
    http.begin(client1, serverName);
  }
  //==================================================================================================================================
  // Initialize gyro sespr mpu6050
  //==================================================================================================================================
  // Try to initialize!
  // if (Wire.begin(4,15))
  //  Serial.println("mpu6050 successful init on i2c scl=4, sda=15");
  //else
  //  Serial.println("mpu6050 not successful");
  //setupMPU();
  //================================================================================================
  // BLE setup
  //================================================================================================
  // Create the BLE Device
  BLEDevice::init("ESP32-quectelgps");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  //BLEService *pService = pServer->createService(SERVICE_UUID);  // creates only 4 characteristics
  BLEService *pService = pServer->createService(BLESERVICE_UUID,30,0); //Create up to 10 charcteristics

  // Create a BLE Characteristic
  pCharacteristic_1 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_1,
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_INDICATE
                    );                   

  pCharacteristic_2 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_2,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE 
                    );  
  pCharacteristic_3 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_3,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE );                    
  pCharacteristic_4 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_4,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE );                    
  pCharacteristic_5 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_5,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE );                    
  pCharacteristic_6 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_6,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE );   
  pCharacteristic_7 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_7,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE );   
  pCharacteristic_8 = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_8,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE );                    
  
  
  // Create a BLE Descriptor
  
  pDescr_1 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_1->setValue("Temperature");
  pCharacteristic_1->addDescriptor(pDescr_1);
  
  pDescr_2 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_2->setValue("Humitity");
  pCharacteristic_2->addDescriptor(pDescr_2);
  
  pDescr_3 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_3->setValue("Pressure");
  pCharacteristic_3->addDescriptor(pDescr_3);

  pDescr_4 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_4->setValue("GPS Longtitude");
  pCharacteristic_4->addDescriptor(pDescr_4);
  
  pDescr_5 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_5->setValue("GPS Latitude");
  pCharacteristic_5->addDescriptor(pDescr_5);

  pDescr_6 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_6->setValue("altitude");
  pCharacteristic_6->addDescriptor(pDescr_6);

  pDescr_7 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_7->setValue("altitude");
  pCharacteristic_7->addDescriptor(pDescr_7);

  pDescr_8 = new BLEDescriptor((uint16_t)0x2901); //xxhs  
  //pDescr_1 = new BLEDescriptor((uint16_t)0X2B18); //xxhs
  pDescr_8->setValue("speed");
  pCharacteristic_7->addDescriptor(pDescr_8);
  
  
  pBLE2902_1 = new BLE2902();
  pBLE2902_1->setNotifications(true);
  pCharacteristic_1->addDescriptor(pBLE2902_1);
  
  pBLE2902_2 = new BLE2902();
  pBLE2902_2->setNotifications(true);
  pCharacteristic_2->addDescriptor(pBLE2902_2);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("BLE - Waiting a client connection to notify...");

} // end of setup

//****************************************************************************************************
// main loop
//****************************************************************************************************
void loop()
{
  
  //===============================================================================================
  // read the Temperature value
  // read battery value
  //===============================================================================================

  unsigned long current_ms = millis();  // to save multiple calls to millis()
  static bool have_thp = true;
  
  read_THP(current_ms, &have_thp, &temperature, &humidity, &pressure);
  float vBat;
  float val = analogRead(37);
  delay(100);
  vBat = val * (3.3/4095)  ;
  //************************************************************************************************
  //   loop on GPS serial communication - check if data is available every 1 sec new reading form GPS should occur
  //************************************************************************************************
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  { 
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  if(newData == true)
  {
    newData = false;

   if ((gps.location.isValid() == 1) & (gps.satellites.value()>=3)) {

      Serial.println("gps data valid");
      
      // toggle internal LED to indicate valid gps data
      if (digitalRead(25)==LOW) {
        digitalWrite(25, HIGH);
      } else
        digitalWrite(25, LOW);

      lat=(gps.location.lat());
      lng=(gps.location.lng());
      alt=gps.altitude.meters();  
      sat=gps.satellites.value();
      course= gps.course.deg();
      //distbetween = gps.distanceBetween(gps.location.lat(), gps.location.lng(), landingLat, landingLng));
      speed=gps.speed.kmph();
      Serial.print("latitude = ");
      Serial.println(lat,4);
      Serial.print("longtitude = ");
      Serial.println(lng,4);
      Serial.print("altitude = ");
      Serial.println(alt);
      Serial.print("sat = ");
      Serial.println(sat);
    } else {
      Serial.print("no of Sat = ");
      Serial.print(gps.satellites.value());
      Serial.println("  -- gps data invalid");
      lng=0;
      lat=0;
      //display_gps(0, 0, 0, 0, 0 , 0);
    }
  }
  //==================================================================================================================================
  // Show the values on the oled screen
  //==================================================================================================================================
   display_thp(temperature, humidity, pressure, lat, lng); 
  //================================================================================================================================== 
  // BLE Stuff
  // check if connected and notify the others with the new values
  //======================================================================
  if (deviceConnected) {
    // string sample
    //std::string message = "hello"; // up to 20 bytes
    //pCharacteristic_1->setValue(message);
    uint8_t temp[4];
    *((float*)temp) = temperature;
 
    pCharacteristic_1->setValue(temp,4); // set the value up to 20 bytes
    pCharacteristic_1->notify(); // actually sending the value
    
    pCharacteristic_2->setValue(humidity); // set the value up to 20 bytes
    pCharacteristic_2->notify(); // actually sending the value
    
    pCharacteristic_3->setValue(pressure); // set the value up to 20 bytes
    pCharacteristic_3->notify(); // actually sending the value
    
    pCharacteristic_4->setValue(lng); // set the value up to 20 bytes
    pCharacteristic_4->notify(); // actually sending the value
    
    pCharacteristic_5->setValue(lat); // set the value up to 20 bytes
    pCharacteristic_5->notify(); // actually sending the value
      
    pCharacteristic_6->setValue(alt); // set the value up to 20 bytes
    pCharacteristic_6->notify(); // actually sending the value

    pCharacteristic_7->setValue(sat);// set the value up to 20 bytes
    pCharacteristic_7->notify(); // actually sending the value

    pCharacteristic_8->setValue(speed);// set the value up to 20 bytes
    pCharacteristic_8->notify(); // actually sending the value
    
    Serial.println("BLE Connected - Data sent to BLE ");
    delay(2000);
  } 

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");

    oldDeviceConnected = deviceConnected;
    Serial.println("BLE disconnecting");
  } 

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE connecting");
  }
  //************************************************************************************
  /* Get new mpu6050 sensor events with the readings */
  //************************************************************************************
  recordAccelRegisters();
  recordGyroRegisters();
  //printData();

  //************************************************************************************
  /* Check if a wifi client - browser is connected to this server   */
  //************************************************************************************
  WiFiClient client = server.available(); // Check if a client has connected
  if (client) {

    //http_response1(temperature, humidity, pressure, client);
    // Prepare the response
    String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n <!DOCTYPE html> <html> <head> <title>Neo GPS Modul - Umgebunds- und GPS DATA</title> <style>";
    s += "a:link {background-color: RED;text-decoration: none;}";
    s += "table, th, td </style> </head> <body style=background-color:WHITE>";
    s += "<h1  style=color:RED;font-size:400%";
    s += " ALIGN=CENTER>Tony smart IoT - Quectel GPS und Umweltdaten</h1>" ;
    s += "<p ALIGN=CENTER style=color:BLUE;font-size:250% >";
    s += "<b>Details</b></p> <table ALIGN=CENTER style=";
    s += "font-size:250%;";
    
    s += "> <tr> <th>Temperature = </th>";
    s += "<td ALIGN=LEFT >";
    s += String(temperature);
    s += (" Grad Celsius");

    s += "</td> </tr> <tr> <th>Humidity =</th> <td ALIGN=LEFT >";
    s += String(humidity);
    s += (" %");
  
    s += "</td> </tr> <tr>  <th>Pressure =</th> <td ALIGN=LEFT >";
    s += String(pressure);
    
    s += (" hPa");
    s += "</td> </tr> <tr> <th>Latitude =</th> <td ALIGN=LEFT >";
    s += (gps.location.lat());
    s += "</td> </tr> <tr> <th>Longitude =</th> <td ALIGN=LEFT >";
    s += (gps.location.lng());
    s += "</td> </tr> <tr>  <th>Speed =</th> <td ALIGN=LEFT >";
    s += (gps.speed.kmph());
    s += (" kmph");
    s += "</td> </tr> <tr>  <th>No of Satellites =</th> <td ALIGN=LEFT >";
    s += (gps.satellites.value());
    s += "</td></tr> <tr> <th>Altitude =</th> <td ALIGN=LEFT >";
    s += (gps.altitude.meters());
    s += (" m");
    
  
    s += "</td>  </tr> </table> ";
   
    s += "</body> </html> ";
   
    client.print(s); // all the values are send to the webpage
    delay(100);

  }

  // *************************************************************************************************
  // check if have to send bpm280 data (temperatur, humitity, pressure)
  //   also save the values in an array
  // *************************************************************************************************
  if ((millis() - transmission_timestamp_bpm) >= (TTN_MESSAGING_INTERVAL_BPM * 1000)) {
    
    //DisplayGMC(100,200,300,true,false);
    transmission_timestamp_bpm=millis();
  
    Serial.print("Temperatur = ");
    Serial.println(temperature);
    Serial.print("humidity   = ");
    Serial.println(humidity);
    Serial.print("pressure   = ");
    Serial.println(pressure);
    Serial.println("-----");
    Serial.print("Satellites = ");
    Serial.println(sat);

    //************************************************************************************
    /* Wirte the last measurement into a ring buffer  */
    //************************************************************************************
    bpm_buffer[bpm_array_cntr][0]=rtc.now().unixtime();
    bpm_buffer[bpm_array_cntr][1]=temperature;
    bpm_buffer[bpm_array_cntr][2]=humidity;
    bpm_buffer[bpm_array_cntr][3]=pressure;
    bpm_buffer[bpm_array_cntr][4]=vBat;
    bpm_array_cntr++;
    if (bpm_array_cntr > 9) {
      bpm_array_cntr=0;
    }

    Serial.println("Sending BPM280 data to TTN ...");
    // (int event, int tbd, uint32_t unixtime, float temperature, float humidity, float pressure, float currentVoltage)
    // event is 1 inidicating bpm data is sent - required for TTN payload decoder to get correct values from receive butter 
    uint32_t sendBPMData2TTNStartTime= rtc.now().unixtime();
    
    sendBMEData2TTN(1,0,rtc.now().unixtime(), (temperature), humidity, pressure, vBat);
    
    uint32_t sendBPMData2TTNEndTime = rtc.now().unixtime();
    Serial.print("TTN transmisstion time = ");
    Serial.println(sendBPMData2TTNEndTime-sendBPMData2TTNStartTime);
    Serial.print("TTN txt completion code = ");
    Serial.println(lora_txt_completion); // this is set in lorawan.coo when ttn event is received

    
    // implement here wtdn when lora_txt_completion was not successful
    lora_txt_completion = 0; // delete error code for next attempt

    
    //001hs
    // Data to send with HTTP POST
    
    http.addHeader("Content-Type", "application/json");
     String httpRequestData = "{\"device_id\":\"eui-70b3d57ed005af1a\",\"app_id\":\"hs-heltec-womo-bme280\",\"BatV\":\"" + String(vBat) + "\",\"event\":\"1\",\"humidity\":\""+ String(humidity)+ "\",\"pressure\":\""+ String(pressure)+ "\",\"temperature\":\""+ String(temperature)+ "\",\"unixtime\":\"" + String(rtc.now().unixtime())+"\"}";
    
     Serial.println(httpRequestData);
        int httpResponseCode = http.POST(httpRequestData);
        //int httpResponseCode = http.GET();
        Serial.print("httpResponseCode = ");
        Serial.println(httpResponseCode);
        
    
  }
  // *************************************************************************************************
  // check if have to send gps data - if intervaltime is reached and we have valid gps data and gps data changed since last transmission
  //
  //  in parallel send the gps data also via wifi - http post request to server for gps data
  // *************************************************************************************************
 
  if ((millis() - transmission_timestamp_gps) >= (TTN_MESSAGING_INTERVAL_GPS * 1000)) {
    transmission_timestamp_gps=millis();

    if (gps.location.isValid() == 1) {
      Serial.println("have valid GPS Data ");
      
      // attention the sprintf does not cut the values after 3 digits - it will be rounded
      //  8.1234 - will be 8.123 and
      //  8.1235 - will be 8.124
      sprintf(lat_str,"%.3f",lat);
      sprintf(lng_str,"%.3f",lng);
      sprintf(previous_lat_str,"%.3f",previous_lat);
      sprintf(previous_lng_str,"%.3f",previous_lng);

       if ((strcmp(previous_lat_str,lat_str) != 0) | (strcmp(previous_lng_str,lng_str) != 0)) {
        Serial.println("----------------------------");
        Serial.println("GPS Data changed since last transmission ");
        Serial.println("----------------------------");
      
        previous_lat = lat;
        previous_lng = lng;
        
        gps_buffer[bpm_array_cntr][0]=rtc.now().unixtime();
        gps_buffer[bpm_array_cntr][1]=lng;
        gps_buffer[bpm_array_cntr][2]=lat;
        gps_buffer[bpm_array_cntr][3]=alt;
        gps_buffer[bpm_array_cntr][4]=sat;
        gps_array_cntr++;
        if (gps_array_cntr > 9) {
          gps_array_cntr=0;
        }
        Serial.println("Sending GPS data to TTN ...");
        // (int event, int tbd, uint32_t unixtime, double lat, double lng, float alt, uint32_t sat, float speed) 
        // event is 2 inidicating gps data is sent - required for TTN payload decoder to get correct values from receive butter 
        uint32_t sendData2TTNStartTime= rtc.now().unixtime();
        
        sendGPSData2TTN(2,0,rtc.now().unixtime(), lat, lng, alt, sat, speed);
        
        uint32_t sendData2TTNEndTime = rtc.now().unixtime();
        Serial.print("TTN transmisstion time = ");
        Serial.println(sendData2TTNEndTime-sendData2TTNStartTime);
        Serial.print("TTN txt completion code = ");
        Serial.println(lora_txt_completion); // this is set in lorawan.coo when ttn event is received
        // implement here wtdn when lora_txt_completion was not successful
        lora_txt_completion = 0; // delete error code for next attempt

        // send data via wifi
         //event: event,
         //latitude: lat,
         //longtitude: lng,
         //alt: alt,
         //sat: sat,
         //speed: speed,
         //unixtime: unixtime
        //  "unixtime": 171009224     
        http.addHeader("Content-Type", "application/json");
         
        String httpRequestData = "{\"device_id\":\"eui-70b3d57ed005af1a\",\"app_id\":\"hs-heltec-womo-bme280\",\"BatV\":\"" + String(vBat) + "\",\"event\":\"2\",\"latitude\":\""+ String(lat)+ "\",\"longtitude\":\""+ String(lng)+ "\",\"alt\":\""+ String(alt)+ "\",\"speed\":\""+ String(speed)+ "\",\"unixtime\":\"" + String(rtc.now().unixtime())+"\"}";           
        Serial.println(httpRequestData);
        int httpResponseCode = http.POST(httpRequestData);
        //int httpResponseCode = http.GET();
        Serial.print("httpResponseCode = ");
        Serial.println(httpResponseCode);
      }
    }
  }
  //******************************************************************************************
  // check if downlink data is available
  //******************************************************************************************
  if (rec_buffer_len > 0) {
  
    for (int i = 0; i < rec_buffer_len; i++) {
      if (rec_buffer[i] < 0x10) {
        Serial.print(F("0"));
      }
      Serial.print(rec_buffer[i], HEX);
    }
    Serial.print("\n");
    rec_buffer_len=0;
    //*****************************************************************************************
    // if we receive a set rtc command - byte 0 = 1
    //*****************************************************************************************
    if (rec_buffer[0] == 1) {
       uint32_t unixtime =  (rec_buffer[2] << 32) | (rec_buffer[3] << 24) | (rec_buffer[4] << 16) | (rec_buffer[5] << 8) | rec_buffer[6];
       
      Serial.println(String(unixtime));
      rtc.adjust(DateTime(unixtime));
    }    
    //*****************************************************************************************
    // if we receive a 'reboot command - byte 0 = 2
    //*****************************************************************************************
    if (rec_buffer[0] == 2) {
       ESP.restart();
    }
  } 
}
