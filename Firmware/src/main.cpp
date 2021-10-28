/** 
 * @file main.cpp
 * @author Tobias Poppe
 * @date 28.10.2021
 * @brief Main-File for the CAMO.NRW Firmware
 *
 * This file is the main.cpp file of the CAMO.NRWs Mainboard Firmware
 * All files and the documentation can be found on https://github.com/topoppe/camo.nrw.dachbox
 * @see https://www.camo.nrw
 * @see https://github.com/topoppe/camo.nrw.dachbox
 */

#include <Arduino.h>
#include "PCA9685.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ros.h>
#include <std_msgs/Float32.h>

/**
 * Define Debug-State, Pins and bat-capacity
 *
 */

#define DEBUG false //true: send plain debug-messages on UART, false: send ROS-Data on UART

#define DHTTYPE DHT22
#define DHTPIN 19
#define DS18B20_PIN 26
#define LTC2943_ALCC_PIN 36
#define taster_1_PIN 34
#define taster_2_PIN 35
#define taster_3_PIN 32
#define taster_4_PIN 33
#define taster_5_PIN 25
#define pc_remote_led_PIN 27
#define on_board_led_PIN 2

#define pwm_port_rgb_led_blau 0
#define pwm_port_rgb_led_rot 1
#define pwm_port_rgb_led_gruen 2
#define pwm_port_relais_fan 3
#define pwm_port_relais_r3 4
#define pwm_port_relais_r1_r2 5
#define pwm_port_fan_pwm 6
#define pwm_port_mosfet_pc_remote 7

#define battery_capacity 27 // Define Battery-Capacity to 27Ah

struct messwerte /*! struct for all measurment-data */
{
  float dht_temperature, dht_humidity;
  float ds18b20_temperature;
  float ltc2943_spannung, ltc2943_strom, ltc2943_temperature, battery_percent;
  uint16_t ltc2943_coulomb;
  float adc_extern_1, adc_charger, adc_externe_stromversorgung, adc_extern_2;
};

// --------------- Deep-Sleep-Settings  /*! Conversion factor for micro seconds to seconds */
#define uS_TO_S_FACTOR 1000000 /*! Conversion factor for micro seconds to seconds */
#define TIME_TO_DEEPSLEEP 60   /*! Time ESP32 will go to sleep (in seconds)  */

RTC_DATA_ATTR bool system_an;          /*! bool if the system is switched on or off, stored in deepsleep-save memory */
RTC_DATA_ATTR bool first_start = true; /*! bool for init the mcp3428 only at coldstart, stored in deepsleep-save memory */
RTC_DATA_ATTR uint8_t relais_mode = 0; /*! relaismode, stored in deepsleep-save memory */
long switch_sys_off_at_time = 0;       /*! switch of timer */

/**
 * @brief Routine for flushing the i2c-interface
 *
 */
void i2c_flush()
{
  while (Wire.available())
  {
    Wire.read();
  }
}

/**
 * @brief Routines for accesing the PCA9685 pwm-controller
 *
 */
RTC_DATA_ATTR PCA9685 pwmController; /*! Library using default B000000 (A5-A0) i2c address, and default Wire @400kHz */

/**
 * @brief Initialize the pwm-controller
 *
 */
void pwm_setup()
{
  pwmController.resetDevices();        /*! Resets all PCA9685 devices on i2c line */
  pwmController.init();                /*! Initializes module using default totem-pole driver mode, and default disabled phase balancer */
  pwmController.setPWMFrequency(1000); /*! Set PWM freq to 1000Hz (default is 200Hz, supports 24Hz to 1526Hz) */
}

/**
 * @brief Switch the relais 1 & 2 between using battery for the system or charging the battery
 *
 * @param on bool input for setting the relais to system-battery-powered (true) or charge the battery (false)
 */
void relais_switch_to_charge(bool on)
{
  if (on)
    pwmController.setChannelPWM(pwm_port_relais_r1_r2, 4096);
  else
    pwmController.setChannelPWM(pwm_port_relais_r1_r2, 0);
}

/**
 * @brief Switch the relais 3 between using battery or external power-suplly for the system
 *
 * @param on bool input for setting the relais to system-battery-powered (true) or external power-supply (false)
 */
void relais_switch_system_on_bat(bool on)
{
  if (on)
    pwmController.setChannelPWM(pwm_port_relais_r3, 4096);
  else
    pwmController.setChannelPWM(pwm_port_relais_r3, 0);
}

/**
 * @brief Set the fan-speed according to the temperature
 * temperature < 40 degree Celsius: Fans off
 * temperature > 70 degree Celsius: Fans at highets speed
 * temperture between: Fan-Speed linear to temperature
 * 
 * Max Fan-Speed is limited to about 80%, due to higher system-voltage on loaeding than fans are rated!
 * @param temperatur temperature as setup for the fans
 */
void luefter_steuerung(float temperatur)
{
  if (temperatur < 40)
    pwmController.setChannelPWM(pwm_port_fan_pwm, 0);
  else if (temperatur > 70)
    pwmController.setChannelPWM(pwm_port_fan_pwm, 3275);
  else
    pwmController.setChannelPWM(pwm_port_fan_pwm, map(int(temperatur), 40, 70, 1000, 3275)); //map temperaturen between 40° and 70° to 0% and 80% fanspeed
}

/**
 * @brief Switch fan power-supply to external-connector "Load battery"
 */
void fan_relais_extern()
{
  pwmController.setChannelPWM(pwm_port_relais_fan, 4096);
}

/**
 * @brief Switch fan power-supply to internal battery
 */
void fan_relais_batterie()
{
  pwmController.setChannelPWM(pwm_port_relais_fan, 0);
}

/**
 * @brief Switch the mosfet to controll the pc´s powerswitch on or off
 *
 * @param on bool input for switch the mosfet on (true, switch pressed) or off (false, switch not pressed)
 */
void pc_remote_mosfet(bool on)
{
  if (on)
    pwmController.setChannelPWM(pwm_port_mosfet_pc_remote, 4096);
  else
    pwmController.setChannelPWM(pwm_port_mosfet_pc_remote, 0);
}

/**
 * @brief Set the color of the build-in-rgb-led
 *
 * @param rot value of the red-channel 0-255 
 * @param gruen value of the green-channel 0-255
 * @param blau value of the blue-channel 0-255
 */
void set_rgb_led(int rot, int gruen, int blau)
{
  pwmController.setChannelPWM(pwm_port_rgb_led_rot, rot << 4);     //value shifted to 12-bit value
  pwmController.setChannelPWM(pwm_port_rgb_led_gruen, gruen << 4); //value shifted to 12-bit value
  pwmController.setChannelPWM(pwm_port_rgb_led_blau, blau << 4);   //value shifted to 12-bit value
}

// ------- DHT22-Sensor Routinen -----
DHT_Unified dht(DHTPIN, DHTTYPE);

/**
 * @brief Init the DHT-Sensor
 *
 */

void dht_setup()
{
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
}

/**
 * @brief Get values from the DHT-Sensor
 * 
 * Returns the measurment value into memory. Returns NAN if sensor can´t be read
 *
 * @param dht_temperature pointer to a float variable to store the temperature value into
 * @param dht_humidity pointer to a float variable to store the humidity value into
 */
void get_dht_values(float *dht_temperature, float *dht_humidity)
{
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature))
  {
    *dht_temperature = event.temperature;
  }
  else
  {
    *dht_temperature = NAN;
  }
  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity))
  {
    *dht_humidity = event.relative_humidity;
  }
  else
  {
    *dht_humidity = NAN;
  }
}

// ------- DS18B20-Sensor Routinen -----
OneWire ds18b20_oneWire(DS18B20_PIN);
DallasTemperature ds18b20_sensor(&ds18b20_oneWire);
DeviceAddress sensor_address;

/**
 * @brief Init the DS18B20-Sensor
 *
 */
void ds18b20_setup()
{
  ds18b20_sensor.begin();
  ds18b20_sensor.getAddress(sensor_address, 0);
  ds18b20_sensor.setResolution(sensor_address, 9);
}

/**
 * @brief Tell the DS18B20-Sensor to start a measurment. The measurments takes about 500 ms, so comeback later to read out.
 */
void ds18b20_start_meassurment()
{
  ds18b20_sensor.requestTemperatures(); // Send the command to get temperatures
}

/**
 * @brief Get values from the external DS18B20-Sensor
 * 
 * Returns the measurment value into memory. Returns NAN if sensor can´t be read
 *
 * @param dht_temperature pointer to a float variable to store the temperature value into
 */
void get_ds18b20_value(float *temperature)
{
  float tempC = ds18b20_sensor.getTempC(sensor_address);
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    *temperature = tempC;
  }
  else
  {
    *temperature = NAN;
  }
}

// ------- MCP3428-ADC Routinen -----
/**
 * @brief Init the MCP3428 ADC-IC
 * 
 * In fact just a general-call reset. Resets also the PWM-IC!
 *
 */
void mcp3428_setup()
{
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  Wire.endTransmission();
  delay(1); // MC3428 benötigt 0,5ms zum reset
}

/**
 * @brief Get values from ADC-IC
 * 
 * Calculates values in Volt from the raw-adc-values and stores them into memory
 *
 * @param adc_extern_1 pointer to a float variable to store the value from adc_extern_1 in Volt into
 * @param adc_charger pointer to a float variable to store the value from the external charger-connector in Volt into
 * @param adc_externe_stromversorgung pointer to a float variable to store the value from the external power-supply-connector in Volt into
 * @param adc_extern_2 pointer to a float variable to store the value from adc_extern_2 in Volt into
 */
void get_adc_values(float *adc_extern_1, float *adc_charger, float *adc_externe_stromversorgung, float *adc_extern_2)
{
  uint8_t buffer[3] = {};
  uint8_t config = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    config = (i << 5) | 0x80;
    i2c_flush();
    Wire.beginTransmission(0x68);
    Wire.write(config);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 4);
    delay(5); //wait for adc
    if (Wire.available() != 4)
      if (DEBUG)
        if (DEBUG)
          Serial.println("Read error!");

    for (uint8_t i = 0; i < 3; ++i)
      buffer[i] = Wire.read();

    long raw_result = (buffer[0] << 8) | buffer[1]; //combine 2 byte to a long value, adc returns 12-Bit-Meassurment
    if (raw_result == 0xFFFF)
      raw_result = 0;                  //Check for overflow if channel not connected
    float result = raw_result * 0.001; //Calculate Volt from Raw-Value
    result = (result * 5380) / 680;    //Calculate the Volt-Value in front of the voltagedivider
    if (i == 0)
      *adc_extern_1 = result;
    else if (i == 1)
      *adc_charger = result;
    else if (i == 2)
      *adc_externe_stromversorgung = result;
    else if (i == 3)
      *adc_extern_2 = result;
  }
}

// -------- LTC 2943 Routinen
uint8_t ltc2943_config = 0b11111000;
uint8_t ltc2943_config_standby = 0b11111001;
uint8_t ltc2943_adr = 0x64;

/**
 * @brief Init the LTC2943 battery gauge IC
 *
 */
void ltc2943_setup()
{
  Wire.beginTransmission(ltc2943_adr);
  Wire.write(0x01);
  Wire.write(ltc2943_config);
  Wire.endTransmission();
}

/**
 * @brief Switch the analog front-end of the LTC2943 into standby or vice versa. 
 * 
 * To avoid write processes while readout the LTC2943 needs to be shutdown before readout and switch again afterwards
 *
 * @param state true for switch to standby, false to switch on
 */
void ltc_2943_standby(bool state)
{
  if (!state)
  {
    Wire.beginTransmission(ltc2943_adr);
    Wire.write(0x01);
    Wire.write(ltc2943_config);
    Wire.endTransmission();
  }
  else
  {
    Wire.beginTransmission(ltc2943_adr);
    Wire.write(0x01);
    Wire.write(ltc2943_config_standby);
    Wire.endTransmission();
  }
}

/**
 * @brief tell the LTC-2943 the battery is fully charged
 */
void set_ltc_2943_coulomb_full()
{
  ltc_2943_standby(true);

  Wire.beginTransmission(ltc2943_adr);
  Wire.write(0x02);
  Wire.write(0xFF);
  Wire.write(0xFF);
  Wire.endTransmission();

  ltc_2943_standby(false);
}

/**
 * @brief Get values from LTC2943 battery gauge IC
 * 
 * Calculates values from the raw-values and stores them into memory
 *
 * @param spannung pointer to a float variable to store the voltage value at shunt into
 * @param strom pointer to a float variable to store the current value over shunt into
 * @param coulomb pointer to a float variable to store the integrated coulomb into
 * @param temperatur pointer to a float variable to store the ic´s temperature into
 * @param battery_percent pointer to a float variable to store the percentage SOC of the battery into
 */
void get_ltc2943_values(float *spannung, float *strom, uint16_t *coulomb, float *temperatur, float *battery_percent)
{
  uint8_t buffer[24] = {};
  int repeat = 0;

  ltc_2943_standby(true);
  while (repeat < 3)
  {
    i2c_flush();
    Wire.beginTransmission(ltc2943_adr);
    Wire.write(0x00);
    Wire.requestFrom(0x64, 24);
    delay(50); //wait for answer
    if (Wire.available() == 24)
    {
      for (uint8_t i = 0; i < 24; ++i)
        buffer[i] = Wire.read();
      Wire.endTransmission();
      ltc_2943_standby(false);
      break;
    }
    else
      repeat++;
  }
  if (repeat == 3)
  {
    // Read error!
    *spannung = NAN;
    *strom = NAN;
    *coulomb = NAN;
    *temperatur = NAN;
    set_rgb_led(255, 255, 255);
    Wire.endTransmission();
    ltc_2943_standby(false);
    return;
  }

  if (buffer[1] != ltc2943_config_standby)
  {
    ltc2943_setup();
  }
  else
  {
    // ----- readout voltage
    uint16_t spannung_raw = (buffer[0x08] << 8) | buffer[0x09]; //combine two int to long
    *spannung = 23.6 * (float(spannung_raw) / 65535);           //calculate voltage from raw value, see datasheet
    // ----- readout current
    uint16_t strom_raw = (buffer[0x0e] << 8) | buffer[0x0f];           //combine two int to long
    if (strom_raw < 38000)                                             //if raw value is to high, its a wrong Value, Charger is limited to 5A max;
      *strom = (0.06 / 0.0015) * ((float(strom_raw) - 32767) / 32767); //calculate current from raw value, see datasheet
    if (*strom > 0.05 && *strom <= 0.15)
      set_ltc_2943_coulomb_full(); //tell the IC battery is fully charged
    // ----- readout Coulomb-Counter
    uint16_t coulomb_counter_raw = (buffer[0x02] << 8) | buffer[0x03]; //combine two int to long
    if (coulomb_counter_raw <= 10)                                     //there is a minimal current needed by the charge-circuit. To avoid overflow of the Coulomb-Counter reset to max if overflow occured
    {
      set_ltc_2943_coulomb_full();
      coulomb_counter_raw = 0xFFFF;
    }
    *coulomb = coulomb_counter_raw;
    // ----- Temperatur auslesen
    uint16_t temperatur_raw = (buffer[0x14] << 8) | buffer[0x15];   //combine two int to long
    *temperatur = (510 * (float(temperatur_raw) / 65535)) - 273.15; //calculate temperature from raw value, see datasheet
    // --- Batterie in Prozent berechnen
    float coulomb_bat_empty = 65535 - (battery_capacity / 0.011333);              //calculate min. coulomb of the battery based on capacity
    float bat_percent_raw = map(*coulomb, 65535, int(coulomb_bat_empty), 100, 0); //map the coulomb-value to percentage
    if ((bat_percent_raw > 100) || (bat_percent_raw < 0))                         //catch if theres a failure and set percentage to zero
      bat_percent_raw = 0;
    *battery_percent = bat_percent_raw;
  }
}

// ------- E-Ink-Display Routinen -----
// Waveshare Pico-ePaper-2.9, 3-Color B/W/R, 296x128 Pixels
TaskHandle_t Task1;

#define ENABLE_GxEPD2_GFX 1

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <BitmapDisplay.h>
#include <TextDisplay.h>
#include <GxEPD2_display_selection_new_style.h>

BitmapDisplay bitmaps(display);

struct display_data /*! struct for all display-data */
{
  int akku_ladung;
  int ladestatus;
  int system_status;
  int versorgungs_art;
  int temperatur;
};
RTC_DATA_ATTR struct display_data data_on_display; //init a instance of display_data in secured memory to avoid display refresh at each wake-up

RTC_DATA_ATTR bool refresh_display_busy_flag = false; //flag for display-refresh is in progress

/**
 * @brief init the display
 */
void display_setup()
{
  display.init(115200);
  display.setRotation(3);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
}

/**
 * @brief background helper task for display-refreshing. 
 * 
 * Runs on second CPU-Core to avoid blocking while display refreshes
 */
void display_refresh_background_task(void *pvParameters)
{
  while (display.nextPage())
    ;
  refresh_display_busy_flag = false;
  vTaskDelete(NULL);
}

/**
 * @brief Refresh the display if there are newer values than already displayed
 * 
 * @param data_to_show struct with all data to show on display
 */
void show_data_on_display(struct display_data data_to_show)
{
  if ((data_to_show.akku_ladung != data_on_display.akku_ladung ||
       data_to_show.ladestatus != data_on_display.ladestatus ||
       data_to_show.system_status != data_on_display.system_status ||
       data_to_show.temperatur != data_on_display.temperatur ||
       data_to_show.versorgungs_art != data_on_display.versorgungs_art) &&
      !refresh_display_busy_flag)
  {
    refresh_display_busy_flag = true; //set display busy flag
    int16_t temp_x1, temp_y1;
    uint16_t temp_w, temp_h;
    int16_t foreground, background;

    if (DEBUG)
      Serial.print("akku_ladung: ");
    if (DEBUG)
      Serial.println(data_to_show.akku_ladung);

    if (DEBUG)
      Serial.print("Ladestatus: ");
    if (DEBUG)
      Serial.println(data_to_show.ladestatus);

    if (DEBUG)
      Serial.print("System Status: ");
    if (DEBUG)
      Serial.println(data_to_show.system_status);

    if (DEBUG)
      Serial.print("Temperatur: ");
    if (DEBUG)
      Serial.println(data_to_show.temperatur);

    if (DEBUG)
      Serial.print("Verorgungsart: ");
    if (DEBUG)
      Serial.println(data_to_show.versorgungs_art);

    display.firstPage();
    display.setFullWindow();
    display.fillRect(0, 0, 296, 128, GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    if (data_to_show.akku_ladung >= 10)
    {
      foreground = GxEPD_BLACK;
      background = GxEPD_WHITE;
    }
    else
    {
      foreground = GxEPD_WHITE;
      background = GxEPD_RED;
      display.fillRect(0, 0, 148, 64, background);
    }
    display.setCursor(25, 25);
    display.setTextColor(foreground);
    display.print("Akku: ");
    display.print(data_to_show.akku_ladung);
    display.print("% ");
    if (data_to_show.ladestatus == 1)
    {
      display.setCursor(10, 45);
      display.print("Wird geladen");
    }

    display.setCursor(15, 100);
    display.setTextColor(GxEPD_BLACK);
    if (data_to_show.versorgungs_art == 0)
    {
      display.print("Netzbetrieb");
    }
    else
    {
      display.print("Akkubetrieb");
    }

    if (data_to_show.temperatur <= 23)
    {
      foreground = GxEPD_BLACK;
      background = GxEPD_WHITE;
    }
    else
    {
      foreground = GxEPD_WHITE;
      background = GxEPD_RED;
      display.fillRect(148, 0, 296, 64, background);
    }
    display.setCursor(165, 25);
    display.setTextColor(foreground);
    display.print("Temperatur:");
    display.setCursor(195, 45);
    String temperatur_string = "";
    if (data_to_show.temperatur >= 0)
      temperatur_string = temperatur_string + "+" + data_to_show.temperatur; //Add a + before temperature if temperature is positive
    else
      temperatur_string = temperatur_string + "-" + data_to_show.temperatur; //Add a - before temperature if temperature is negative
    display.print(temperatur_string);
    // add a degree-sign after value
    display.getTextBounds(temperatur_string, 165, 45, &temp_x1, &temp_y1, &temp_w, &temp_h);
    display.fillCircle(195 + temp_w + 8, temp_y1 + 3, 3, foreground); //Xpos,Ypos,r,Farbe
    display.fillCircle(195 + temp_w + 8, temp_y1 + 3, 1, background); //Xpos,Ypos,r,Farbe
    display.setCursor(195 + temp_w + 12, 45);
    display.print("C");

    display.setCursor(170, 100);
    if (data_to_show.system_status == 0)
    {
      foreground = GxEPD_BLACK;
      background = GxEPD_WHITE;
      display.fillRect(148, 64, 296, 128, GxEPD_WHITE);
      display.setTextColor(GxEPD_BLACK);
      display.print("System aus");
    }
    else
    {
      display.fillRect(148, 64, 296, 128, GxEPD_RED);
      display.setTextColor(GxEPD_WHITE);
      display.print("System an");
    }

    display.drawFastHLine(0, 64, 296, GxEPD_BLACK);
    display.drawFastVLine(148, 0, 128, GxEPD_BLACK);

    data_on_display.akku_ladung = data_to_show.akku_ladung;
    data_on_display.ladestatus = data_to_show.ladestatus;
    data_on_display.system_status = data_to_show.system_status;
    data_on_display.temperatur = data_to_show.temperatur;
    data_on_display.versorgungs_art = data_to_show.versorgungs_art;

    //Create display refresh background task on second CPU-Core
    xTaskCreatePinnedToCore(
        display_refresh_background_task, /* Task function. */
        "Task1",                         /* name of task. */
        10000,                           /* Stack size of task */
        NULL,                            /* parameter of the task */
        1,                               /* priority of the task */
        &Task1,                          /* Task handle to keep track of created task */
        0);                              /* pin task to core 0 */
  }
}

// GPI und GPIO Routinen
/**
 * @brief init all GPIO ports that are directly connected to the ESP32
 * 
 * This is NOT for the GPIOs on the PWM-IC!
 */
void gpio_setup()
{
  pinMode(LTC2943_ALCC_PIN, INPUT);
  pinMode(taster_1_PIN, INPUT);
  pinMode(taster_2_PIN, INPUT);
  pinMode(taster_3_PIN, INPUT);
  pinMode(taster_4_PIN, INPUT);
  pinMode(taster_5_PIN, INPUT);
  pinMode(pc_remote_led_PIN, INPUT);
  pinMode(on_board_led_PIN, OUTPUT);
}

/**
 * @brief set the onboard LED on or off. 
 * @param on true = on, false = off
 */
void set_onboard_led(bool on)
{
  if (on)
    digitalWrite(on_board_led_PIN, HIGH);
  else
    digitalWrite(on_board_led_PIN, LOW);
}

/**
 * @brief Read all sensors and return their current values
 * @param messwerte pointer to a struct to save values into
 */

void read_all_sensors(struct messwerte *messwerte)
{

  ds18b20_start_meassurment();                                                                                                                                            //start the meassurment of the DS18B20. Takes sometime, so start early
  get_dht_values(&messwerte->dht_temperature, &messwerte->dht_humidity);                                                                                                  //Get the DHT-Values
  get_ds18b20_value(&messwerte->ds18b20_temperature);                                                                                                                     //Now get the DS18B20 Values
  get_ltc2943_values(&messwerte->ltc2943_spannung, &messwerte->ltc2943_strom, &messwerte->ltc2943_coulomb, &messwerte->ltc2943_temperature, &messwerte->battery_percent); //Get the battery-gauge values
  get_adc_values(&messwerte->adc_extern_1, &messwerte->adc_charger, &messwerte->adc_externe_stromversorgung, &messwerte->adc_extern_2);                                   //get the ADC-Values
}

// ------- ROS-Node Routinen -------------

// init all publishers
std_msgs::Float32 ros_msg_dht_temperature;
ros::Publisher pub_dht_temperature("dht_temperature", &ros_msg_dht_temperature);

std_msgs::Float32 ros_msg_dht_humidity;
ros::Publisher pub_dht_humidity("dht_humidity", &ros_msg_dht_humidity);

std_msgs::Float32 ros_msg_ds18b20_temperature;
ros::Publisher pub_ds18b20_temperature("ds18b20_temperature", &ros_msg_ds18b20_temperature);

std_msgs::Float32 ros_msg_ltc2943_spannung;
ros::Publisher pub_ltc2943_spannung("ltc2943_spannung", &ros_msg_ltc2943_spannung);

std_msgs::Float32 ros_msg_ltc2943_strom;
ros::Publisher pub_ltc2943_strom("ltc2943_strom", &ros_msg_ltc2943_strom);

std_msgs::Float32 ros_msg_ltc2943_temperature;
ros::Publisher pub_ltc2943_temperature("ltc2943_temperature", &ros_msg_ltc2943_temperature);

std_msgs::Float32 ros_msg_ltc2943_coulomb;
ros::Publisher pub_ltc2943_coulomb("ltc2943_coulomb", &ros_msg_ltc2943_coulomb);

std_msgs::Float32 ros_msg_ltc2943_battery_percentage;
ros::Publisher pub_ltc2943_battery_percentage("ltc2943_battery_percentage", &ros_msg_ltc2943_battery_percentage);

std_msgs::Float32 ros_msg_adc_extern_1;
ros::Publisher pub_adc_extern_1("adc_extern_1", &ros_msg_adc_extern_1);

std_msgs::Float32 ros_msg_adc_charger;
ros::Publisher pub_adc_charger("adc_charger", &ros_msg_adc_charger);

std_msgs::Float32 ros_msg_adc_externe_stromversorgung;
ros::Publisher pub_adc_externe_stromversorgung("adc_externe_stromversorgung", &ros_msg_adc_externe_stromversorgung);

std_msgs::Float32 ros_msg_adc_extern_2;
ros::Publisher pub_adc_extern_2("adc_extern_2", &ros_msg_adc_extern_2);

ros::NodeHandle nh;

/**
 * @brief init a ROS-Serial-Node and advertise all publishers to it
 */
void ros_serial_init()
{
  nh.initNode();
  nh.advertise(pub_dht_temperature);
  nh.advertise(pub_dht_humidity);
  nh.advertise(pub_ds18b20_temperature);
  nh.advertise(pub_ltc2943_spannung);
  nh.advertise(pub_ltc2943_strom);
  nh.advertise(pub_ltc2943_temperature);
  nh.advertise(pub_ltc2943_coulomb);
  nh.advertise(pub_adc_extern_1);
  nh.advertise(pub_adc_charger);
  nh.advertise(pub_adc_externe_stromversorgung);
  nh.advertise(pub_adc_extern_2);
  nh.advertise(pub_ltc2943_battery_percentage);
}

/** 
 * @brief write the actual values to all publishers and send it out by uart
 * @param messwerte data to send
 */
void ros_serial_send_msg(struct messwerte messwerte)
{
  digitalWrite(on_board_led_PIN, !digitalRead(on_board_led_PIN)); //toggle on-board-led for debuging. UART-Debug is not possible due to the block of ROS-Serial

  ros_msg_dht_temperature.data = messwerte.dht_temperature;
  ros_msg_dht_humidity.data = messwerte.dht_humidity;
  ros_msg_ds18b20_temperature.data = messwerte.ds18b20_temperature;
  ros_msg_ltc2943_spannung.data = messwerte.ltc2943_spannung;
  ros_msg_ltc2943_strom.data = messwerte.ltc2943_strom;
  ros_msg_ltc2943_temperature.data = messwerte.ltc2943_temperature;
  ros_msg_ltc2943_coulomb.data = messwerte.ltc2943_coulomb;
  ros_msg_adc_extern_1.data = messwerte.adc_extern_1;
  ros_msg_adc_charger.data = messwerte.adc_charger;
  ros_msg_adc_externe_stromversorgung.data = messwerte.adc_externe_stromversorgung;
  ros_msg_adc_extern_2.data = messwerte.adc_extern_2;
  ros_msg_ltc2943_battery_percentage.data = messwerte.battery_percent;

  pub_dht_temperature.publish(&ros_msg_dht_temperature);
  pub_dht_humidity.publish(&ros_msg_dht_humidity);
  pub_ds18b20_temperature.publish(&ros_msg_ds18b20_temperature);
  pub_ltc2943_spannung.publish(&ros_msg_ltc2943_spannung);
  pub_ltc2943_strom.publish(&ros_msg_ltc2943_strom);
  pub_ltc2943_temperature.publish(&ros_msg_ltc2943_temperature);
  pub_ltc2943_coulomb.publish(&ros_msg_ltc2943_coulomb);
  pub_adc_extern_1.publish(&ros_msg_adc_extern_1);
  pub_adc_charger.publish(&ros_msg_adc_charger);
  pub_adc_externe_stromversorgung.publish(&ros_msg_adc_externe_stromversorgung);
  pub_adc_extern_2.publish(&ros_msg_adc_extern_2);
  pub_ltc2943_battery_percentage.publish(&ros_msg_ltc2943_battery_percentage);
  nh.spinOnce(); //let the ROS-Serial communicate with the PC
}

// ------- Main Routinen -----

RTC_DATA_ATTR bool esp_was_in_deepsleep = 0;
int ros_publish_time;

/**
 * @brief setup routine, started after each coldstart or deepsleep-wakeup
 */
void setup()
{
  if (DEBUG)
    Serial.begin(115200); // Begin Serial and Wire interfaces
  Wire.begin();
  if (first_start)
  {
    mcp3428_setup();
    first_start = false;
  }
  dht_setup();
  display_setup();
  ltc2943_setup();
  ds18b20_setup();
  gpio_setup();
  if (!DEBUG)
    ros_serial_init();
}

/**
 * @brief Main Loop
 */
void loop()
{
  struct messwerte messwerte;
  struct display_data data_to_show;

  if (!system_an)
  {
    data_to_show.system_status = 0;
    if (esp_was_in_deepsleep && (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0))
    {
      pwm_setup();
      system_an = true;
      switch_sys_off_at_time = millis() + 30000;
      if (messwerte.adc_externe_stromversorgung <= 10)
      {
        relais_switch_system_on_bat(true);
        delay(500);
      }
      pc_remote_mosfet(true);
      delay(500);
      pc_remote_mosfet(false);
    }
    else
    {
      read_all_sensors(&messwerte);
      if (DEBUG)
        Serial.print("adc extern: ");
      Serial.print(messwerte.adc_externe_stromversorgung);
      Serial.print("    adc charge: ");
      Serial.println(messwerte.adc_charger);
      if (DEBUG)
        Serial.print("Akku Coulomb: ");
      Serial.print(messwerte.ltc2943_coulomb);
      Serial.print(" = ");
      Serial.print(messwerte.battery_percent);
      Serial.println("%");
      if (DEBUG)
        Serial.print("LTC Strom: ");
      Serial.println(messwerte.ltc2943_strom);
      if (DEBUG)
        Serial.print("Relais mode: ");
      Serial.println(relais_mode);

      if ((messwerte.adc_externe_stromversorgung < 12) && (messwerte.adc_charger < 10) && (relais_mode != 1))
      { //nichts angeschlossen
        pwm_setup();
        fan_relais_batterie();
        relais_switch_to_charge(false);
        relais_switch_system_on_bat(false);
        relais_mode = 1;
      }
      if ((messwerte.adc_externe_stromversorgung >= 12) && (messwerte.adc_charger < 10) && (relais_mode != 2))
      { //Extern angeschlossen
        pwm_setup();
        fan_relais_batterie();
        relais_switch_to_charge(false);
        relais_switch_system_on_bat(false);
        relais_mode = 2;
      }
      if ((messwerte.adc_externe_stromversorgung < 12) && (messwerte.adc_charger >= 10) && (relais_mode != 3))
      { //Laden angeschlossen
        pwm_setup();
        fan_relais_extern();
        relais_switch_to_charge(true);
        relais_switch_system_on_bat(false);
        relais_mode = 3;
      }
      if ((messwerte.adc_externe_stromversorgung >= 12) && (messwerte.adc_charger >= 10) && (relais_mode != 4))
      { //Extern angeschlossen, Laden angeschlossen
        pwm_setup();
        fan_relais_extern();
        relais_switch_to_charge(true);
        relais_switch_system_on_bat(false);
        relais_mode = 4;
      }

      if (messwerte.ltc2943_strom >= 0.2)
      {
        data_to_show.ladestatus = 1;
        if (messwerte.dht_temperature > 40)
        {
          pwm_setup();
          luefter_steuerung(messwerte.dht_temperature);
        }
      }
      else
        data_to_show.ladestatus = 0;

      if (messwerte.adc_externe_stromversorgung >= 10)
      {
        data_to_show.versorgungs_art = 0;
      }
      else
      {
        data_to_show.versorgungs_art = 1;
      }

      data_to_show.akku_ladung = round(messwerte.battery_percent);
      data_to_show.temperatur = round(messwerte.dht_temperature);
      show_data_on_display(data_to_show);

      while (refresh_display_busy_flag == true)
      {
        if (!digitalRead(taster_1_PIN))
          pc_remote_mosfet(true);
        else
          pc_remote_mosfet(false);
      } //wait if display is refreshing

      ros_publish_time = 0; //Reset the last send timestamp

      esp_was_in_deepsleep = true;
      set_rgb_led(0, 0, 0);
      esp_sleep_enable_timer_wakeup(TIME_TO_DEEPSLEEP * uS_TO_S_FACTOR);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, LOW);
      if (DEBUG)
        Serial.println("Going to Deepsleep");
      esp_deep_sleep_start();
    }
  }

  //System an
  if (system_an)
  {
    esp_was_in_deepsleep = false;
    relais_mode = 0;
    data_to_show.system_status = 1;
    if (!digitalRead(taster_1_PIN))
      pc_remote_mosfet(true);
    else
      pc_remote_mosfet(false);

    if (digitalRead(pc_remote_led_PIN))
      set_rgb_led(255, 0, 0);
    else
      set_rgb_led(0, 0, 0);

    if (digitalRead(pc_remote_led_PIN))
      switch_sys_off_at_time = millis() + 30000;

    if (millis() > switch_sys_off_at_time)
    {
      system_an = false;
      relais_switch_system_on_bat(true);
      while (refresh_display_busy_flag == true)
      {
        if (!digitalRead(taster_1_PIN))
          pc_remote_mosfet(true);
        else
          pc_remote_mosfet(false);
      } //wait if display is refreshing
      relais_switch_to_charge(true);
      relais_switch_system_on_bat(false);
      esp_sleep_enable_timer_wakeup(TIME_TO_DEEPSLEEP * uS_TO_S_FACTOR);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, LOW);
      if (DEBUG)
        Serial.println("Going to Deepsleep");
      esp_deep_sleep_start();
    }

    read_all_sensors(&messwerte);
    luefter_steuerung(messwerte.dht_temperature);

    if ((messwerte.adc_externe_stromversorgung < 12) && (messwerte.adc_charger < 10))
    { //nichts angeschlossen
      fan_relais_batterie();
      relais_switch_to_charge(false);
      relais_switch_system_on_bat(true);
    }
    if ((messwerte.adc_externe_stromversorgung >= 12) && (messwerte.adc_charger < 10))
    { //Extern angeschlossen
      fan_relais_batterie();
      relais_switch_to_charge(false);
      relais_switch_system_on_bat(false);
    }
    if ((messwerte.adc_externe_stromversorgung < 12) && (messwerte.adc_charger >= 10))
    { //Laden angeschlossen
      fan_relais_extern();
      relais_switch_to_charge(false);
      relais_switch_system_on_bat(true);
    }
    if ((messwerte.adc_externe_stromversorgung >= 12) && (messwerte.adc_charger >= 10))
    { //Extern angeschlossen, Laden angeschlossen
      fan_relais_extern();
      relais_switch_to_charge(true);
      relais_switch_system_on_bat(false);
    }

    if (millis() >= ros_publish_time)
    {
      if (!DEBUG)
        ros_serial_send_msg(messwerte);
      if (DEBUG)
        Serial.print(millis());
      if (DEBUG)
        Serial.println(": Sending ROS!");
      ros_publish_time = millis() + 5000;
    }
    else if (!DEBUG)
      nh.spinOnce();

    if (messwerte.adc_externe_stromversorgung >= 10)
    {
      data_to_show.versorgungs_art = 0;
    }
    else
    {
      data_to_show.versorgungs_art = 1;
    }

    if (messwerte.ltc2943_strom >= 0.5)
    {
      data_to_show.ladestatus = 1;
    }
    else
      data_to_show.ladestatus = 0;

    data_to_show.akku_ladung = round(messwerte.battery_percent);
    data_to_show.temperatur = round(messwerte.dht_temperature);
    show_data_on_display(data_to_show);

    if (DEBUG)
    {
      Serial.println("System ist an! ");
      Serial.print("ADC Charger: ");
      Serial.println(messwerte.adc_charger);
      Serial.print("ADC Externe versorgung: ");
      Serial.println(messwerte.adc_externe_stromversorgung);
      Serial.print("LTC Spannung: ");
      Serial.println(messwerte.ltc2943_spannung);
      Serial.print("LTC Strom: ");
      Serial.println(messwerte.ltc2943_strom);
      Serial.print("LTC Coulomb: ");
      Serial.println(messwerte.ltc2943_coulomb);
      Serial.print("LTC temperatur: ");
      Serial.println(messwerte.ltc2943_temperature);
      Serial.print("LTC bat prozent: ");
      Serial.println(messwerte.battery_percent);
      delay(1000);
    }
  }
}