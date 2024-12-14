// OLED display related code

#include <Arduino.h>
#include <U8x8lib.h>

#include "display.h"

#define PIN_DISPLAY_ON 25

#define PIN_OLED_RST 16
#define PIN_OLED_SCL 15
#define PIN_OLED_SDA 4

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);
U8X8_SSD1306_64X32_NONAME_HW_I2C u8x8_lora(PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);
U8X8 *pu8x8;

bool displayIsClear;

void setup_display() {
  pu8x8 = &u8x8;
  pu8x8->begin();

}

void clear_displayline(int line) {
  const char *blanks;
  blanks = "                ";  
  pu8x8->drawString(0, line, blanks);
}

void display_thp(float temperature, float humidity, float pressure, double lat, double lng) {
  char dsp_msg[8]; 
  const char *blanks;
  blanks = "        ";
  int i;

  pu8x8->setFont(u8x8_font_victoriamedium8_r);
  
  dtostrf(temperature,5,2,dsp_msg);
  pu8x8->drawString(0, 0, "Temp: ");
  pu8x8->drawString(10, 0, dsp_msg);
  //Serial.println(dsp_msg);
  strcpy(dsp_msg, "");
  //Serial.println(dsp_msg);
  dtostrf(pressure,5,2,dsp_msg);
  pu8x8->drawString(0, 2, "pressure: ");
  pu8x8->drawString(10, 2, dsp_msg);
  strcpy(dsp_msg, "");
  dtostrf(humidity,5,2,dsp_msg);
  pu8x8->drawString(0, 4, "Humidity: ");
  pu8x8->drawString(10, 4, dsp_msg);
  strcpy(dsp_msg, "");
  dtostrf(lat,5,2,dsp_msg);
  pu8x8->drawString(0, 6, "Lat: ");
  pu8x8->drawString(10, 6, dsp_msg);
  strcpy(dsp_msg, "");
  dtostrf(lng,5,2,dsp_msg);
  pu8x8->drawString(0, 7, "Lon: ");
  pu8x8->drawString(10, 7, dsp_msg);
  strcpy(dsp_msg, "");
}

//**********************************************************************************
// Display GPS Data
//**********************************************************************************
void display_gps(double lat, double lng, float alt, int sat) {
  char dsp_msg[8]; 
  const char *blanks;
  blanks = "        ";
  int i;
  pu8x8->setFont(u8x8_font_victoriamedium8_r);
  strcpy(dsp_msg, "");

  strcpy(dsp_msg, "");
  dtostrf(lat,5,2,dsp_msg);
  pu8x8->drawString(0, 0, "Lat: ");
  pu8x8->drawString(10, 1, dsp_msg);
  
  strcpy(dsp_msg, "");
  dtostrf(lng,5,2,dsp_msg);
  pu8x8->drawString(0, 2, "Lon: ");
  pu8x8->drawString(10, 3, dsp_msg);
  
  strcpy(dsp_msg, "");
  dtostrf(alt,5,2,dsp_msg);
  pu8x8->drawString(0, 5, "Alt: ");
  pu8x8->drawString(10, 5, dsp_msg);
  
  strcpy(dsp_msg, "");
  dtostrf(sat,5,2,dsp_msg);
  pu8x8->drawString(0, 7, "Sat: ");
  pu8x8->drawString(10, 7, dsp_msg);
  
}
//**********************************************************************************
void display_statusline(String txt) {
  if (txt.length() == 0)
    return;
  int line = 7;
  pu8x8->setFont(u8x8_font_victoriamedium8_r);
  clear_displayline(line);
  pu8x8->drawString(0, line, txt.c_str());
}

static int status[STATUS_MAX] = {ST_NODISPLAY, ST_NODISPLAY, ST_NODISPLAY, ST_NODISPLAY,
                                 ST_NODISPLAY, ST_NODISPLAY, ST_NODISPLAY, ST_NODISPLAY
                                };  // current status of misc. subsystems

static const char *status_chars[STATUS_MAX] = {
  // group WiFi and transmission to internet servers
  ".W0wA",  // ST_WIFI_OFF, ST_WIFI_CONNECTED, ST_WIFI_ERROR, ST_WIFI_CONNECTING, ST_WIFI_AP
  ".s1S?",  // ST_SCOMM_OFF, ST_SCOMM_IDLE, ST_SCOMM_ERROR, ST_SCOMM_SENDING, ST_SCOMM_INIT
  ".m2M?",  // ST_MADAVI_OFF, ST_MADAVI_IDLE, ST_MADAVI_ERROR, ST_MADAVI_SENDING, ST_MADAVI_INIT
  // group TTN (LoRa WAN)
  ".t3T?",  // ST_TTN_OFF, ST_TTN_IDLE, ST_TTN_ERROR, ST_TTN_SENDING, ST_TTN_INIT
  // group BlueTooth
  ".B4b?",  // ST_BLE_OFF, ST_BLE_CONNECTED, ST_BLE_ERROR, ST_BLE_CONNECTABLE, ST_BLE_INIT
  // group other
  ".",      // ST_NODISPLAY
  ".",      // ST_NODISPLAY
  ".H7",    // ST_NODISPLAY, ST_HV_OK, ST_HV_ERROR
};

void set_status(int index, int value) {
  if ((index >= 0) && (index < STATUS_MAX)) {
    if (status[index] != value) {
      status[index] = value;
      display_status();
    }
  } else
    //log(ERROR, "invalid parameters: set_status(%d, %d)", index, value);
    delay(100);
}

int get_status(int index) {
  return status[index];
}

char get_status_char(int index) {
  if ((index >= 0) && (index < STATUS_MAX)) {
    int idx = status[index];
    if (idx < strlen(status_chars[index]))
      return status_chars[index][idx];
    else
      //log(ERROR, "string status_chars[%d] is too short, no char at index %d", index, idx);
      delay(100);
  } else
    delay(100);
    //log(ERROR, "invalid parameters: get_status_char(%d)", index);
  return '?';  // some error happened
}

void display_status(void) {
  char output[17];  // max. 16 chars wide display + \0 terminator
  const char *format = "%c %c %c %c %c %c %c %c";  // 8 or 16 chars wide
  snprintf(output, 17, format,
           get_status_char(0), get_status_char(1), get_status_char(2), get_status_char(3),
           get_status_char(4), get_status_char(5), get_status_char(6), get_status_char(7)
          );
  display_statusline(output);
}

char *format_time(unsigned int secs) {
  static char result[4];
  unsigned int mins = secs / 60;
  unsigned int hours = secs / (60 * 60);
  unsigned int days = secs / (24 * 60 * 60);
  if (secs < 60) {
    snprintf(result, 4, "%2ds", secs);
  } else if (mins < 60) {
    snprintf(result, 4, "%2dm", mins);
  } else if (hours < 24) {
    snprintf(result, 4, "%2dh", hours);
  } else {
    days = days % 100;  // roll over after 100d
    snprintf(result, 4, "%2dd", days);
  }
  return result;
}
