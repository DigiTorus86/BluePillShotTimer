/* 2Bit Shot Timer
 *  
 * Configurable shot timer for detecting and recording shot times. 
 *  
 * Requires:
 * - STM32 BluePill (or similar microcontroller)
 * - 16x2 LCD character display
 * - ADMP401 Analog Microphone (or similar)
 * - Active Buzzer 3-24V
 * - 5 buttons
 * 
 * See BOM doc for full component list.
 * 
 * Uses the LiquidCrystal library for the HD44780-compatible LCD control:
 * https://github.com/arduino-libraries/LiquidCrystal
 * 
 * Uses the FlashStorage library by Khoi Hoang for storing user config settings in flash:
 * https://github.com/khoih-prog/FlashStorage_STM32F1
 *  
 * Copyright (c) 2022 Paul Pagel
 * This is free software; see the license.txt file for more information.
 * There is no warranty; not even for merchantability or fitness for a particular purpose.
 *  
 * - 16x2 LCD Character Display Pins -
 * LCD 01 VSS pin to GND
 * LCD 02 VDD pin to 5V
 * LCD 03 VO pin to POT
 * LCD 04 RS pin to digital pin PB11
 * LCD 05 RW pin to GND
 * LCD 06 EN pin to digital pin PB10
 * LCD 07-10 D0 to D3 pins not used
 * LCD 11 D4 pin to digital pin PB0
 * LCD 12 D5 pin to digital pin PA7
 * LCD 13 D6 pin to digital pin PA6
 * LCD 14 D7 pin to digital pin PA5
 * LCD 15 BL+/BLA pin to 5V
 * LCD 16 BL-/BLK pin to GND
 */

#include <LiquidCrystal.h>
#include <FlashStorage_STM32F1.h>


#define FLASH_DEBUG                 2
#define USING_FLASH_SECTOR_NUMBER   (REGISTERED_NUMBER_FLASH_SECTORS - 2)

#define LCD_RS_PIN  PA5  //PB11
#define LCD_EN_PIN  PA6  //PB10
#define LCD_D4_PIN  PA7  //PB0
#define LCD_D5_PIN  PB0  //PA7
#define LCD_D6_PIN  PB10 //PA6
#define LCD_D7_PIN  PB11 //PA5
#define LCD_BK_PIN  PB9  // Backlight control connected to 2N3904 NPN base, connect BLK/- to emitter
#define MIC_PIN     PA1  // Analog in
#define BUZZER_PIN  PB8  // Buzzer control connected to 2N3904 NPN base, connect buzzer GND to emitter
#define MENU_UP_PIN PB12 // All button pins should be pulled up
#define MENU_DN_PIN PB13
#define SET_UP_PIN  PB14
#define SET_DN_PIN  PB15
#define SET_OK_PIN  PA8
#define START_PIN   PA15
#define LED1_PIN    PC13 // Built-in LED, in open-drain mode (LOW to turn on)
#define LED2_PIN    PA2  // Additional indicator LED

#define MAX_SHOTS   256
#define MAX_PARS    5
#define DC_OFFSET   2048   // DC offset in mic signal
#define NOISE       10     // Noise/hum/interference in mic signal

#define MIN_DETECT_THRESH 3000  // based on 12-bit ADC
#define MAX_DETECT_THRESH 4000

#define MIN_DEADTIME_SAMPLES 10
#define MAX_DEADTIME_SAMPLES 200


#define MIN_BUZZER_VOL 0        // PWM setting
#define MAX_BUZZER_VOL 250

#define MIN_BUZZER_MS 0
#define MAX_BUZZER_MS 1000

#define MIN_BKLIGHT_MS 0
#define MAX_BKLIGHT_MS 20000

#define EEPROM_VERSION 1
#define EEPROM_START   0xFEEDFACE
#define EEPROM_END     0xDEADBEEF

enum app_mode_type 
{
  MODE_TITLE,     // Display the app title
  MODE_MENU1,     // Start mode, delay times
  MODE_MENU2,     // Par times
  MODE_MENU3,     // Shot sensitivity
  MODE_MENU4,     // Buzzer volume and duration
  MODE_MENU5,     // Backlight duration + TBD
  MODE_STARTING,  // Countdown to start
  MODE_RECORDING, // Timer started, detecting and recording shots
  MODE_RESULTS,   // Display the recorded stats, split times
  MODE_REFRESH    // Set previous mode to refresh in order to trigger a redraw of the current screen
};

enum start_mode_type
{
  START_MODE_RANDOM,  // Use the a random delay between min and max value for the start countdown
  START_MODE_FIXED    // Use a specific countdown delay value (min_start_ms)
};

// ** Persisted configuration settings **

struct    app_config_type
{
  enum      start_mode_type start_mode; // Random or Fixed
  uint32_t  min_start_ms;
  uint32_t  max_start_ms; 
  uint16_t  par_count;
  long      par_ms[MAX_PARS];
  uint16_t  buzzer_vol;           // PWM setting for buzzer (0 = silent)
  uint32_t  buzzer_ms;            // Duration of buzzer sound
  uint32_t  bklight_ms;           // Backlight duration from when button last pushed to turnoff time
  int       detect_thresh;        // Shot detection threshold (using 12 bit ADC)
  long      min_interval_samples; // Minimum # of samples below detection threshold before next shot
};
app_config_type app_config;

// ** Operational values **
enum      app_mode_type app_mode, prev_app_mode;
long      start_time;             // All "time" values in reference to system clock millis()
long      end_time;
long      delay_end_time;
long      buzzer_off_time;
long      bklight_off_time;
long      par_time;
uint8_t   shot_count;
long      shot_time[MAX_SHOTS];
bool      buzzer_start_on;
bool      buzzer_par_on;

int       mic_lvl     = 10;       // Current "dampened" audio level
int       mic_min_avg = 0;        // For dynamic adjustment of graph low & high
int       mic_max_avg = 2048;

bool      btn_menu_up_pressed, btn_menu_up_released;
bool      btn_menu_dn_pressed, btn_menu_dn_released;
bool      btn_set_up_pressed, btn_set_up_released;
bool      btn_set_dn_pressed, btn_set_dn_released;
bool      btn_set_ok_pressed, btn_set_ok_released;
bool      btn_start_pressed, btn_start_released;


// The 16x2 LCD character display
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

/*
 * Debug output for hex values
 */
void displayHex(uint16_t value)
{
  if (value <= 0xF)
    Serial.print("000");
  else if (value <= 0xFF)
    Serial.print("00");
  else if (value <= 0xFFF)
    Serial.print("0");
  Serial.print(value, HEX);
}

/*
 * Debug output for flash simulated EEPROM settings
 */
void displayEEPROM()
{
  uint8_t data;

  Serial.println("----Simulated EEPROM Dump----");
  lcd.clear();
  lcd.print("Dump:");
  
  for (int i = 0; i < 128; i++)
  {
    data = EEPROM.read(i);
    displayHex(data);
    Serial.print(" ");
    lcd.print(" ");
    lcd.print(data, HEX);

    if (i % 16 == 0)
    {
      delay(1000);
    }
    
  }
  Serial.println("");
  Serial.println("--------");
}

/*
 * Reads the app config settings from flash memory/simulated EEPROM if they are available.
 * Returns true if able to read settings.  False if no settings were read or failed validation.
 */
bool readConfig()
{
  int address = 0x0;
  int data;
  int data_version;
  
  Serial.print(F("\nStart EmulatedEEPROM on ")); Serial.println(BOARD_NAME);
  Serial.println(FLASH_STORAGE_STM32F1_VERSION);
  
  EEPROM.init();
     
  EEPROM.get(address, data);
  if (data != EEPROM_START)
  {
    Serial.print("Initial read value: ");
    displayHex(data);
    Serial.println();
    Serial.println(F("Unable to load settings from flash."));
    
    return false;
  }

  Serial.println(F("Getting settings from Simulated EEPROM"));

  // Future-proofing in case multiple firmware eeprom versions 
  address += sizeof(EEPROM_START);
  EEPROM.get(address, data_version);
  Serial.print(F("Data version: "));
  Serial.println(data_version);

  // -- read the actual config settings -- //

  address += sizeof(data_version);
  EEPROM.get(address, app_config);
  
  Serial.print(F("Start Mode: "));
  Serial.println(app_config.start_mode);
  Serial.print(F("Min Delay: "));
  Serial.println(app_config.min_start_ms);
  Serial.print(F("Max Delay: "));
  Serial.println(app_config.max_start_ms);

  Serial.print(F("Par Times: "));
  for (int i = 0; i < MAX_PARS; i++)
  {
    Serial.print(app_config.par_ms[i]);
  }

  Serial.println("");
  Serial.print(F("Buzzer ms: "));
  Serial.println(app_config.buzzer_ms);
  Serial.print(F("Backlight ms: "));
  Serial.println(app_config.bklight_ms);

  address += sizeof(app_config);
  EEPROM.get(address, data);
  if (data != EEPROM_END)
  {
    Serial.println(F("EEPROM write validation failed!"));
    Serial.print(F("Validation read value: "));
    displayHex(data);
    Serial.println();
    
    return false;
  }

  return true;
}

/*
 * Writes the app config settings to flash memory/simulated EEPROM
 * Returns true on success, false on failure.
 */
bool writeConfig()
{
  int address = 0x0;
  int data;

  data = EEPROM_START;
  EEPROM.put(address, data);

  data = EEPROM_VERSION;
  address += sizeof(EEPROM_START);
  EEPROM.put(address, data);

  address += sizeof(data);
  EEPROM.put(address, app_config);

  data = EEPROM_END;
  address += sizeof(app_config);
  EEPROM.put(address, data);

  EEPROM.commit();

  data = 0;
  EEPROM.get(address, data);
  
  return (data == EEPROM_END);
}

/*
 * Setup and Initialization 
 */
void setup() 
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("2Bit Shot Timer");
  
  // LEDs and display
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  ledsOn();
  
  pinMode(LCD_BK_PIN, OUTPUT);
  digitalWrite(LCD_BK_PIN, HIGH); // turn on backlight
  
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("2Bit Shot Timer");

  // Buttons 
  pinMode(MENU_UP_PIN, INPUT_PULLUP);
  pinMode(MENU_DN_PIN, INPUT_PULLUP);
  pinMode(SET_UP_PIN, INPUT_PULLUP);
  pinMode(SET_DN_PIN, INPUT_PULLUP);
  pinMode(SET_OK_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);

  // Sound
  pinMode(MIC_PIN, INPUT_ANALOG);
  pinMode(BUZZER_PIN, OUTPUT);

  randomSeed(analogRead(MIC_PIN)); // for random start times

  if (readConfig())
  {
    lcd.setCursor(0, 1);
    lcd.print("Settings loaded");
  }
  else
  {
    // Unable to read from the simulated EEPROM or not previously set up
    // Use default configuration values instead

    lcd.setCursor(0, 1);
    lcd.print("Factory settings");
    
    app_config.start_mode = START_MODE_RANDOM;
    app_config.min_start_ms = 2000;
    app_config.max_start_ms = 5000;
    app_config.par_count = 1;

    for (int i = 0; i < MAX_PARS; i++)
    {
      app_config.par_ms[i] = 5000L;  
    }

    app_config.buzzer_vol = 75;
    app_config.buzzer_ms = 100;
    app_config.bklight_ms = 8000;
    app_config.detect_thresh = 3650;
    app_config.min_interval_samples = 50;

    if (!writeConfig())
    {
      lcd.setCursor(0, 1);
      lcd.print("EEPROM write err");
    }
  }
  
  bklight_off_time = millis() + app_config.bklight_ms;
  app_mode = MODE_TITLE;
  
  handleTitle();
  ledsOff();
}


void checkButtons()
{
  if (digitalRead(MENU_UP_PIN) == LOW)
  {
    btn_menu_up_pressed = true;
    bklight_off_time = millis() + app_config.bklight_ms;
  }
  else if(btn_menu_up_pressed)
  {
    btn_menu_up_pressed = false;
    btn_menu_up_released = true;
  }
  else
  {
    btn_menu_up_pressed = false;
    btn_menu_up_released = false;
  }

  if (digitalRead(MENU_DN_PIN) == LOW)
  {
    btn_menu_dn_pressed = true;
    bklight_off_time = millis() + app_config.bklight_ms;
  }
  else if(btn_menu_dn_pressed)
  {
    btn_menu_dn_pressed = false;
    btn_menu_dn_released = true;
  }
  else
  {
    btn_menu_dn_pressed = false;
    btn_menu_dn_released = false;
  }

  if (digitalRead(SET_UP_PIN) == LOW)
  {
    btn_set_up_pressed = true;
    bklight_off_time = millis() + app_config.bklight_ms;
  }
  else if(btn_set_up_pressed)
  {
    btn_set_up_pressed = false;
    btn_set_up_released = true;
  }
  else
  {
    btn_set_up_pressed = false;
    btn_set_up_released = false;
  }

  if (digitalRead(SET_DN_PIN) == LOW)
  {
    btn_set_dn_pressed = true;
    bklight_off_time = millis() + app_config.bklight_ms;
  }
  else if(btn_set_dn_pressed)
  {
    btn_set_dn_pressed = false;
    btn_set_dn_released = true;
  }
  else
  {
    btn_set_dn_pressed = false;
    btn_set_dn_released = false;
  }

  if (digitalRead(SET_OK_PIN) == LOW)
  {
    btn_set_ok_pressed = true;
    bklight_off_time = millis() + app_config.bklight_ms;
  }
  else if(btn_set_ok_pressed)
  {
    btn_set_ok_pressed = false;
    btn_set_ok_released = true;
  }
  else
  {
    btn_set_ok_pressed = false;
    btn_set_ok_released = false;
  }

  if (digitalRead(START_PIN) == LOW)
  {
    btn_start_pressed = true;
    bklight_off_time = millis() + app_config.bklight_ms;
  }
  else if(btn_start_pressed)
  {
    btn_start_pressed = false;
    btn_start_released = true;
  }
  else
  {
    btn_start_pressed = false;
    btn_start_released = false;
  }
}

/*
 * Turn all LEDs on
 */
void ledsOn()
{
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, HIGH);
}

/*
 * Turn all LEDs off
 */
void ledsOff()
{
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, LOW);
}

/*
 * Turn the buzzer on
 */
void startBuzzer()
{
  ledsOn();
  
  pinMode(BUZZER_PIN, OUTPUT); // required when switching between digital and analog
  
  if (app_config.buzzer_vol == MAX_BUZZER_VOL)
  {
    digitalWrite(BUZZER_PIN, HIGH); // significantly louder than analog PWM
  }
  else
  {
    analogWrite(BUZZER_PIN, app_config.buzzer_vol);  
  }
  
  // digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LCD_BK_PIN, LOW);
}

// Turn the buzzer off
void stopBuzzer()
{
  ledsOff();
  if (app_config.buzzer_vol == MAX_BUZZER_VOL)
  {
    digitalWrite(BUZZER_PIN, LOW);
  }
  else
  {
    analogWrite(BUZZER_PIN, 0);  
  }

  digitalWrite(LCD_BK_PIN, HIGH);
}


/*
 * Displays time to 2 decimal places, i.e. 9.87
 */
void printTime(long ms)
{
  char buff[10];
  float secs;

  secs = (float)ms / 1000.0f;
  sprintf(buff, "%d.%02d", (int)secs, (int)abs(secs*100)%100);
  lcd.print(buff);
}

/*
 * Displays time to 1 decimal place, i.e. 9.8
 */
void printSeconds(uint32_t ms)
{
  char buff[10];
  float secs;

  secs = (float)ms / 1000.0f;
  sprintf(buff, "%d.%01d", (int)secs, (int)abs(secs*10)%10);
  lcd.print(buff);
}

/*
 * Detects a gunshot or other loud noise that exceeds the analog detection threshold.
 */
void detectShot()
{
  static uint32_t samples_below_thresh;
  int             sample = 0;
  
  sample = analogRead(MIC_PIN);             // Raw reading from mic 
  sample = abs(sample - 2048 - DC_OFFSET);  // Center on zero
  sample = (sample <= NOISE) ? 0 : (sample - NOISE); // Remove noise/hum
  mic_lvl = ((mic_lvl * 7) + sample) >> 3;   // "Dampened" reading (else looks twitchy)

  lcd.setCursor(0,1);
  lcd.print(mic_lvl);
  lcd.print("  ");

  if ((mic_lvl > app_config.detect_thresh) && !buzzer_start_on && (samples_below_thresh > app_config.min_interval_samples))
  {
    shot_count += 1;
    shot_time[shot_count] = millis();
    end_time = millis();
    samples_below_thresh = 0;

    ledsOn();
    lcd.setCursor(7, 1);
    lcd.print(shot_count);
    ledsOff();
  }
  else
  {
    samples_below_thresh += 1;
  }
}

/*
 * Display the app title
 */
void handleTitle()
{
  if (prev_app_mode != MODE_TITLE)
  {
    prev_app_mode = MODE_TITLE;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("2Bit Shot Timer");
  
    lcd.setCursor(0, 1);
    lcd.print("v1.0");
  }

  checkButtons();

  if (btn_menu_up_released)
  {
    app_mode = MODE_MENU1;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_MENU5;
  }

  if (btn_start_released)
  {
    app_mode = MODE_STARTING;
  }
}

/*
 * Menu1:  Start Mode (Delay, Fixed)  
 *         Min Delay
 *         Max Delay
 */
void handleMenu1()
{
  static uint8_t active_field;
  static uint8_t max_fields;
  
  if (prev_app_mode != MODE_MENU1)
  {
    prev_app_mode = MODE_MENU1;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Delay: ");
    if (app_config.start_mode == START_MODE_FIXED)
    {
      lcd.print("Fixed");
      
      lcd.setCursor(0, 1);
      lcd.print("Sec:");
      printSeconds(app_config.min_start_ms);
      
      max_fields = 2;
    }
    else
    {
      lcd.print("Random");
      
      lcd.setCursor(0, 1);
      lcd.print("Min:"); 
      printSeconds(app_config.min_start_ms);
      
      lcd.setCursor(8, 2);
      lcd.print("Max:");
      printSeconds(app_config.max_start_ms);
      
      max_fields = 3;
    }

    switch (active_field)
    {
      case 0: // start mode
        lcd.setCursor(7, 0);
        break;
      
      case 1: // min start delay ms
        lcd.setCursor(4, 1);
        break;

      case 2: // max start delay ms
        lcd.setCursor(12, 1);
        break;
    }

    lcd.cursor();
    lcd.blink();
  }

  checkButtons();

  if (btn_set_ok_released)
  {
    active_field = (active_field < max_fields - 1) ? active_field + 1 : 0;
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_up_released)
  {
    switch (active_field)
    {
      case 0: // start mode
        app_config.start_mode = (start_mode_type)((app_config.start_mode + 1) % 2);
        break;
      
      case 1: // min start delay ms
        app_config.min_start_ms = (app_config.min_start_ms < 9900) ? app_config.min_start_ms + 100 : 9900;
        app_config.max_start_ms = (app_config.max_start_ms < app_config.min_start_ms) ? app_config.min_start_ms : app_config.max_start_ms;
        break;

      case 2: // max start delay ms
        app_config.max_start_ms = (app_config.max_start_ms < 9900) ? app_config.max_start_ms + 100 : 9900;
        break;
    }

    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_dn_released)
  {
    switch (active_field)
    {
      case 0: // start mode
        app_config.start_mode = (start_mode_type)((app_config.start_mode + 1) % 2);
        break;
      
      case 1: // min start delay ms
        app_config.min_start_ms = (app_config.min_start_ms > 100) ? app_config.min_start_ms - 100 : 100;
        break;

      case 2: // max start delay ms
        app_config.max_start_ms = (app_config.max_start_ms > 100) ? app_config.max_start_ms - 100 : 100;
        app_config.min_start_ms = (app_config.min_start_ms < app_config.max_start_ms) ? app_config.min_start_ms : app_config.max_start_ms;
        break;
    }
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_menu_up_released)
  {
    app_mode = MODE_MENU2;
    return;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_RESULTS;
    writeConfig();
    return;
  }

  if (btn_start_released)
  {
    app_mode = MODE_STARTING;
  }
}

/*
 * Menu2: Par Times (count)
 *        Par Time Values (ms)
 */
void handleMenu2()
{
  static uint8_t active_field;
  static uint8_t active_par;
  
  if (prev_app_mode != MODE_MENU2)
  {
    prev_app_mode = MODE_MENU2;

    active_par = (active_field == 0) ? 0 : active_field - 1;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Par Times: ");
    lcd.print(app_config.par_count);
  
    lcd.setCursor(0, 1);
    lcd.print("#");
    lcd.print(active_par + 1);
    lcd.print(": ");
    printSeconds(app_config.par_ms[active_par]);

    switch (active_field)
    {
      case 0: // # of part times
        lcd.setCursor(11, 0);
        break;
      default: // par time ms
        lcd.setCursor(4, 1);
        break;
    }
    
    lcd.cursor();
    lcd.blink();
  }

  checkButtons();

  if (btn_set_ok_released)
  {
    // accept value and move to next field
    active_field = (active_field < 5) ? active_field + 1 : 0;
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_up_released)
  {
    switch (active_field)
    {
      case 0: // # par times
        app_config.par_count = (app_config.par_count < MAX_PARS) ? app_config.par_count + 1 : MAX_PARS;
        break;
      default: // active par time ms
        app_config.par_ms[active_par] = (app_config.par_ms[active_par] < 9900) ? app_config.par_ms[active_par] + 100 : 9900;
        break;
    }
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_dn_released)
  {
    switch (active_field)
    {
      case 0: // # par times
        app_config.par_count = (app_config.par_count > 0) ? app_config.par_count - 1 : 0;
        break;
      default: // active par time ms
        app_config.par_ms[active_par] = (app_config.par_ms[active_par] > 100) ? app_config.par_ms[active_par] - 100 : 100;
        break;
    }
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_menu_up_released)
  {
    app_mode = MODE_MENU3;
    return;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_MENU1;
    return;
  }

  if (btn_start_released)
  {
    app_mode = MODE_STARTING;
  }
}

/*
 * Menu3:  Shot sensitivity 
 *         Dead time between shots
 */
void handleMenu3()
{
  static uint8_t active_field;
  
  if (prev_app_mode != MODE_MENU3)
  {
    prev_app_mode = MODE_MENU3;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Shot Sens: ");
    lcd.print(app_config.detect_thresh);

    lcd.setCursor(0, 1);
    lcd.print("Dead Time: ");
    lcd.print(app_config.min_interval_samples);

    switch (active_field)
    {
      case 0: // shot sensitivity
        lcd.setCursor(11, 0);
        break;
      
      case 1: // min dead time interval samples
        lcd.setCursor(11, 1);
        break;
    }

    lcd.cursor();
    lcd.blink();
  }

  checkButtons();

  if (btn_set_ok_released)
  {
    active_field = (active_field < 1) ? active_field + 1 : 0;
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_up_released)
  {
    switch (active_field)
    {
      case 0: // shot sensitivity
        app_config.detect_thresh = (app_config.detect_thresh < MAX_DETECT_THRESH) ? app_config.detect_thresh + 10 : MAX_DETECT_THRESH;
        break;
      
      case 1: // min dead time interval samples
        app_config.min_interval_samples = (app_config.min_interval_samples < MAX_DEADTIME_SAMPLES) ? app_config.min_interval_samples + 10 : MAX_DEADTIME_SAMPLES;
        break;
    }

    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_dn_released)
  {
    switch (active_field)
    {
      case 0: // shot sensitivity
        app_config.detect_thresh = (app_config.detect_thresh > MIN_DETECT_THRESH) ? app_config.detect_thresh - 10 : MIN_DETECT_THRESH;
        break;
      
      case 1: // min dead time interval samples
        app_config.min_interval_samples = (app_config.min_interval_samples > MIN_DEADTIME_SAMPLES) ? app_config.min_interval_samples - 10 : MIN_DEADTIME_SAMPLES;
        break;
    }
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_menu_up_released)
  {
    app_mode = MODE_MENU4;
    return;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_MENU2;
    return;
  }

  if (btn_start_released)
  {
    app_mode = MODE_STARTING;
  }
}

/*
 * Menu4:  Speaker volume
 *         Backlight time
 */
void handleMenu4()
{
  static uint8_t active_field;
  
  if (prev_app_mode != MODE_MENU4)
  {
    prev_app_mode = MODE_MENU4;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Buzz Vol: ");
    lcd.print(app_config.buzzer_vol);
    
    lcd.setCursor(0, 1);
    lcd.print("Buzz Dur: ");
    printTime(app_config.buzzer_ms);

    

    switch (active_field)
    {
      case 0: // buzzer vol
        lcd.setCursor(10, 0);
        break;
        
      case 1: // buzzer duration ms
        lcd.setCursor(10, 1);
        break;
    }

    lcd.cursor();
    lcd.blink();
  }

  checkButtons();

  if (btn_set_ok_released)
  {
    active_field = (active_field < 1) ? active_field + 1 : 0;
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_up_released)
  {
    switch (active_field)
    {
      case 0: // buzzer volume
        app_config.buzzer_vol = (app_config.buzzer_vol < MAX_BUZZER_VOL) ? app_config.buzzer_vol + 25 : MAX_BUZZER_VOL;
        break;
        
      case 1: // buzzer duration ms
        app_config.buzzer_ms = (app_config.buzzer_ms < MAX_BUZZER_MS) ? app_config.buzzer_ms + 50 : MAX_BUZZER_MS;
        break;
    }

    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_dn_released)
  {
    switch (active_field)
    {
      case 0: // buzzer volume
        app_config.buzzer_vol = (app_config.buzzer_vol > MIN_BUZZER_VOL) ? app_config.buzzer_vol - 25 : MIN_BUZZER_VOL;
        break;
        
      case 1: // buzzer duration ms
        app_config.buzzer_ms = (app_config.buzzer_ms > MIN_BUZZER_MS) ? app_config.buzzer_ms - 50 : MIN_BUZZER_MS;
        break;
    }
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_menu_up_released)
  {
    app_mode = MODE_MENU5;
    return;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_MENU3;
    return;
  }

  if (btn_start_released)
  {
    app_mode = MODE_STARTING;
  }
}

/*
 * Menu5:  Backlight time
 */
void handleMenu5()
{
  static uint8_t active_field;
  
  if (prev_app_mode != MODE_MENU5)
  {
    prev_app_mode = MODE_MENU5;
    
    lcd.setCursor(0, 0);
    lcd.print("BakLight: ");
    printTime(app_config.bklight_ms);

    switch (active_field)
    {
      case 0: // backlight duration ms
        lcd.setCursor(10, 0);
        break;
    }

    lcd.cursor();
    lcd.blink();
  }

  checkButtons();

  if (btn_set_ok_released)
  {
    active_field = 0; //(active_field < 1) ? active_field + 1 : 0;
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_up_released)
  {
    switch (active_field)
    {
      case 0: // backlight on duration ms
        app_config.bklight_ms = (app_config.bklight_ms < MAX_BKLIGHT_MS) ? app_config.bklight_ms + 100 : MAX_BKLIGHT_MS;
        break;
    }

    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_set_dn_released)
  {
    switch (active_field)
    {
      case 0: // backlight on ms
        app_config.bklight_ms = (app_config.bklight_ms > MIN_BKLIGHT_MS) ? app_config.bklight_ms - 100 : MIN_BKLIGHT_MS;
        break;
    }
    
    prev_app_mode = MODE_REFRESH;
    return;
  }

  if (btn_menu_up_released)
  {
    app_mode = MODE_RESULTS;
    return;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_MENU4;
    return;
  }

  if (btn_start_released)
  {
    app_mode = MODE_STARTING;
  }
}

/*
 * Starting elay countdown and transition to recording
 */
void handleStarting()
{
  if (prev_app_mode != MODE_STARTING)
  {
    prev_app_mode = MODE_STARTING;

    lcd.noCursor();
    lcd.noBlink();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Starting...");
  
    lcd.setCursor(0, 1);
    lcd.print("Par: ");
    printTime(app_config.par_ms[0]);

    // Clear stats
    start_time = 0;
    end_time = 0;
    shot_count = 0;

    for (int i = 0; i < MAX_SHOTS; i++)
    {
      shot_time[i] = 0;
    }

    delay_end_time = millis() + random(app_config.min_start_ms, app_config.max_start_ms); 
  }

  checkButtons();

  if (millis() >= delay_end_time)
  {
    app_mode = MODE_RECORDING;
    handleRecording();
    return;
  }

  if (btn_menu_up_released)
  {
    app_mode = MODE_MENU1;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_MENU4;
  }
}

/*
 * 
 */
void handleRecording()
{
  static uint8_t cur_par = 0;
  
  if (prev_app_mode != MODE_RECORDING)
  {
    prev_app_mode = MODE_RECORDING;

    lcd.noCursor();
    lcd.noBlink();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Record...");
  
    lcd.setCursor(0, 1);
    lcd.print("Shots: 0");

    startBuzzer();
    buzzer_start_on = true;
    buzzer_off_time = millis() + app_config.buzzer_ms;

    cur_par = 0;
    start_time = millis();
    end_time = start_time;
    par_time = start_time + app_config.par_ms[0];
    shot_time[0] = start_time;
  }

  detectShot();

  long cur_time;
  cur_time = millis();
  
  if (buzzer_start_on && (cur_time > buzzer_off_time))
  {
    stopBuzzer();
    buzzer_start_on = false;
  }

  if (!buzzer_par_on && (cur_time > par_time) && (cur_time < par_time + app_config.buzzer_ms))
  {
    startBuzzer();  // TODO: different pitch?
    buzzer_off_time = cur_time + app_config.buzzer_ms;
    buzzer_par_on = true;

    if (cur_par < MAX_PARS)
    {
      cur_par += 1;
      par_time = cur_time + app_config.par_ms[cur_par];  
    }
  }
  else if (buzzer_par_on && (cur_time > buzzer_off_time))
  {
    buzzer_off_time = cur_time + 100000;
    stopBuzzer();
    buzzer_par_on = false;
  }

  if (digitalRead(START_PIN) == LOW)
  {
    app_mode = MODE_RESULTS;
  }
}


void handleResults()
{
  long elapsed_ms;
  static uint8_t cur_split = 1;
  static uint8_t prev_split = 1;
    
  if (prev_app_mode != MODE_RESULTS)
  {
    prev_app_mode = MODE_RESULTS;

    elapsed_ms = end_time - start_time;
    bklight_off_time = millis() + app_config.bklight_ms;

    lcd.noCursor();
    lcd.noBlink();
    lcd.clear();
    lcd.setCursor(0, 0);
    
    if (elapsed_ms <= 0)
    {
      lcd.print("No shots");
      lcd.setCursor(0, 1);
      lcd.print("detected");
    }
    else
    {
    
      // Total elapsed time from start to last shot
      lcd.print("Sec: ");
      printSeconds(elapsed_ms);
  
      // Over/under par time
      lcd.print(" ");
      if (elapsed_ms >= app_config.par_ms[0])
      {
        lcd.print("+");
        printSeconds(elapsed_ms - app_config.par_ms[0]);
      }
      else
      {
        lcd.print("-");
        printSeconds(app_config.par_ms[0] - elapsed_ms);
      }
  
      // Elapsed time from start to first shot
      lcd.setCursor(0, 1);
      lcd.print("1st: ");
  
      if (shot_time[1] > 0)
      {
        printSeconds(shot_time[1] - start_time);
      }
      else
      {
        lcd.print("NA");
      }
      
      // Number of detected shots
      lcd.print(" #");
      lcd.print(shot_count);
    }
  
      stopBuzzer();
      buzzer_start_on = false;
      buzzer_par_on = false;
  
      delay(500);
  }
  else if (cur_split != prev_split)
  {
    lcd.setCursor(0, 1);
    lcd.print("Split #");
    lcd.print(cur_split);
    lcd.print("  ");
    lcd.setCursor(10, 1);
    if (shot_time[cur_split] > 0)
    {
      printSeconds(shot_time[cur_split] - shot_time[cur_split - 1]);
    }
    else
    {
      lcd.print("NA");
    }

    lcd.print("  ");
    prev_split = cur_split;
  }

  checkButtons();

  if (btn_set_up_released)
  {
    cur_split = (cur_split > 1) ? cur_split - 1 : shot_count;
  }

  if (btn_set_dn_released || btn_set_ok_released)
  {
    cur_split = (cur_split < shot_count) ? cur_split + 1 : 1;
  }

  if (btn_start_released)
  {
    app_mode = MODE_STARTING;
  }

  if (btn_menu_up_released)
  {
    app_mode = MODE_MENU1;
  }

  if (btn_menu_dn_released)
  {
    app_mode = MODE_MENU4;
  }
}


void loop() 
{
  switch (app_mode)
  {
    case MODE_TITLE:
      handleTitle();
      break;
    case MODE_MENU1:
      handleMenu1();
      break;
    case MODE_MENU2:
      handleMenu2();
      break;
    case MODE_MENU3:
      handleMenu3();
      break;
    case MODE_MENU4:
      handleMenu4();
      break; 
    case MODE_MENU5:
      handleMenu5();
      break; 
    case MODE_STARTING:
      handleStarting();
      break;
    case MODE_RECORDING:
      handleRecording();
      break;
    case MODE_RESULTS:
      handleResults();
      break;
  }

  // Common logic
  if (bklight_off_time > millis())
  {
    digitalWrite(LCD_BK_PIN, HIGH);
  }
  else
  {
    digitalWrite(LCD_BK_PIN, LOW);
  }

  if (btn_menu_up_pressed && btn_menu_dn_pressed)
  {
    lcd.clear();
    lcd.println("Saving settings");
    Serial.println("Saving config settings...");
    ledsOn();
    delay(app_config.buzzer_ms);
    ledsOff();
    
    if (!writeConfig())
    {
      lcd.println("Unable to save");
      Serial.println("Starting EEPROM Dump...");
    
      displayEEPROM();
    }
    
    prev_app_mode = MODE_REFRESH;
  }
}
