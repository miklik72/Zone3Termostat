#include <Arduino.h>
#include <Math.h>

/* 3 zone wireless termostat used 433Mhz temperature sensors T25
Martin Mikala (2016) dev@miklik.cz

v2.0.0 14.1.2018 - new version with connection to net, removing configuration menu 
--------------------------------------------------------------------------------------------
v1.1.0 9.11.2016 - extension for set programs
v1.2.0 28.11.2016 - reset to initial state and EEPROM data structure version, more comments
v1.2.1 11.12.2016 - fix program set (validation for programs)
v1.3.0 1.1.2017 - open window detection
v1.3.1 1.1.2017 - fixed temperature history and calc heating
v1.3.2 3.1.2017 - added watchdog
v1.4.0 4.1.2017 - turn on/off extended functions (watchgod,OpenWindow)
v1.4.1 7.11.2017 - new DS3231 library
-------------------------------------------------------------------------------------------

todo:
1.5--   1.1.2017 - extra button for activate sensor
1.--- 18.12.2016 - debuging to serial console
1.---   1.1.2017 - merge{clean} variables c & s as c  - for channel or sensor and use s for program step
1.---   4.1.2017 - log to SD card
1.---   9.1.2017 - invalidate old temperatures (60 minutes)
2.---   1.1.2017 - add remote valve control
2.---   4.1.2017 - add ESP8266 Wifi Connection - control from WEB
----- 28.11.2016 - cut app to more libraries

Devices:
1x Arduino UNO
3x Sencor T25 433MHz sensor http://www.sencor.eu/wireless-sensor/sws-t25
1x RXB6 - 433MHz receiver
1x LCD keypad shield V0.1 LCD1602A+buttons
1x RTC DS3231
1x relay module HL-51 250V/10A
*/

// application version
#define APP_VERSION_MAIN 2
#define APP_VERSION_RELEASE 0
#define APP_VERSION_PATCH 0

#define DAY_STEP 6
#define PROGRAMS 5
#define MIN_TEMP 5
#define MAX_TEMP 26
#define UNFREEZ_TEMP MIN_TEMP - 2


// RF sensors https://github.com/miklik72/SensorT25/
#define IRQ_PIN 2      // RF input with irq
#define SENSORS 3
#include <SensorT25.h>

//LCD
#define PIN_RS 8     // arduino pin wired to LCD RS
#define PIN_EN 9     // arduino pin wired to LCD EN
#define PIN_d4 4     // arduino pin wired to LCD d4
#define PIN_d5 5     // arduino pin wired to LCD d5
#define PIN_d6 6     // arduino pin wired to LCD d7
#define PIN_d7 7     // arduino pin wired to LCD d8
#define PIN_BL 10    // LCD backlight control (see bug http://forum.arduino.cc/index.php?topic=96747.0)
#define SafeBLon(pin) pinMode(pin, INPUT)
#define SafeBLoff(pin) pinMode(pin, OUTPUT)
#define LCD_ROWS 2      // row of LCD
#define LCD_COLUMNS 16  // columns of LCD

// invert one
byte invert1[8] = {
  B11011,
  B10011,
  B11011,
  B11011,
  B11011,
  B11011,
  B10001,
  B11111,
};

// invert two
byte invert2[8] = {
  B10001,
  B01110,
  B11110,
  B11101,
  B11011,
  B10111,
  B00000,
  B11111
};

// invert three
byte invert3[8] = {
  B00000,
  B11101,
  B11011,
  B11101,
  B11110,
  B01110,
  B10001,
  B11111
};

// fire1
byte fire1[8] = {
	0b01000,
	0b01100,
	0b01010,
	0b01010,
	0b01001,
	0b10001,
	0b10001,
	0b01110
};

#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_RS,  PIN_EN,  PIN_d4,  PIN_d5,  PIN_d6,  PIN_d7);
byte lcdrow = 0;
byte lcdcol = 0;

//RTC I2C
#include <DS3231.h>
#include <Wire.h>
//DS3231 rtc(SDA,SCL);
DS3231 Clock;
RTClib rtc;
//Time t;
DateTime t;
unsigned int tmpYear;
byte tmpMonth;
byte tmpDate;
byte tmpHour;
byte tmpMinute;

//keypad
#include <DFR_KeyMM.h>
//DFR_KeyMM keypad(A0);               // A0 is wired in LCD keypad shield
DFR_KeyMM keypad;
int key;

//relay
#define RELAY_PIN 11
#define RELAY_DELAY 1000             // cheange relay state only every 1000 ms
long relay_oldtime = millis();

//EEPROM
#include <EEPROM.h>
#define EEPROM_VERSION 13
#define EEPROM_OFFSET 0                                   // ERRPROM OFFSET
#define EEPROM_OFFSET_VERSION EEPROM_OFFSET               // version of EEPROM data structure
#define EEPROM_OFFSET_DATA EEPROM_OFFSET + 1              // offset for data start
#define EEPROM_OFFSET_ACT EEPROM_OFFSET_DATA              // 1-3 ACTIVATED SENSORS    3B
#define EEPROM_OFFSET_PRG EEPROM_OFFSET_ACT + SENSORS     // 4-6 SENSORS PROGRAM      3B
#define EEPROM_OFFSET_DELAY EEPROM_OFFSET_PRG + SENSORS   // 7-15 DELAY DATE and HOUR  3x3B = DDMM+HH
#define EEPROM_OFFSET_PSET EEPROM_OFFSET_DELAY + 9       // 16 Termostat programs are in EEPROM 1B
#define EEPROM_OFFSET_PROGS EEPROM_OFFSET_PSET + 1       // 17 - 107 Termostat programs - 5x6x3B = 6 steps temperature + HHMM for programs
//EEPROM 11 extension for watchdog
#define EEPROM_OFFSET_BOOTS EEPROM_OFFSET_PROGS + (PROGRAMS * DAY_STEP * 3)  // 108-109 - boots counter 2B
#define EEPROM_OFFSET_BOOTTIME EEPROM_OFFSET_BOOTS + 2                       // 110-113 - boot time and date 4B
//EEPROM 12 extension for window open check
#define EEPROM_OFFSET_ACTWTDOG EEPROM_OFFSET_BOOTTIME + 4                   // 114 - active watchdog 1B
//EEPROM 13 extension for window open check
#define EEPROM_OFFSET_ACTWINDOW EEPROM_OFFSET_ACTWTDOG + 1                   // 114 - active window oen detection 1B
#define EEPROM_OFFSET_NEXT EEPROM_OFFSET_ACTWINDOW + 1                       // 1st free byte
boolean rom_change = false;

//watchdog
#include <avr/wdt.h>

//Application definition
  //LCD positions
    // Main screen
#define CH1_MAIN_R 0
#define CH1_MAIN_C 0
#define CH2_MAIN_R 0
#define CH2_MAIN_C 8
#define CH3_MAIN_R 1
#define CH3_MAIN_C 0
#define POS_TEMP 1
#define POS_STATE 6
#define T_MAIN_R 1
#define T_MAIN_C 8
#define H_MAIN_R 0
#define H_MAIN_C 15
    // Setup sensor screen
#define INF_SENS_R 0             //sensor static info row
#define CH_SENS_C 0
#define AGE_SENS_C 7
#define STATE_SENS_C 11
#define PRG_SENS_C 15
#define EXIT_TIME 10000           //time for automatic exit to main screen in ms
//#define MAX_TEMP 30               // Max temperature set
//#define MIN_TEMP 4                // Min temperature
    // Time set scree
#define TIME_ROW 0
#define TIME_HCOL 4
#define TIME_MCOL 7
#define TIME_SCOL 10
#define DATE_ROW 1
#define DATE_DCOL 6
#define DATE_MCOL 9
#define DATE_YCOL 12
byte cur_pos_c = TIME_HCOL;
byte cur_pos_r = TIME_ROW;

  // Control keys
#define KEYUP 3
#define KEYDOWN 4
#define KEYLEFT 2
#define KEYRIGHT 5
#define KEYSET 1

// watch heating issues
#define TEMP_HIST_STEPS 10                                  // keep 10 last tepm
#define TEMP_HIST_STEP 1                                    // time step for save temperature in minutes
#define TEMP_HIST_TIME TEMP_HIST_STEPS * TEMP_HIST_STEP     // whole time history keeping
#define DELTA_WINDOW_OPEN_C 1                               // temp delta for detect open window
#define DELTA_WINDOW_OPEN_TIME 5                            // time for detect open window in minutes
#define HEATING_PAUSE_WINDOW 30                             // how long will be heating stopped
#define DELTA_VALVE_CLOSE_C 1                               // temp delta for detect close valve
#define DELTA_VALVE_CLOSE_TIME 10                           // time for detect close valve in minutes
#define HEATING_PAUSE_VALVE 30                              // how long will be heating stopped in minutes
#define TEMP_LIMIT 1                                        // control temperature + - wanted temperature
bool active_window_pause = false;                            // activate window pause detection
float prg_temp_hist[SENSORS][TEMP_HIST_STEPS];              // temperature history for detect heating pause
long temp_hist_last_time = millis();                         // timestamp for calculate next temperature record
byte open_window[SENSORS]={0,0,0};                          // detect open window
byte close_valve[SENSORS]={0,0,0};                          // detect close valve
long open_window_time[SENSORS]={0,0,0};                     // detect open window
long close_valve_time[SENSORS]={0,0,0};                     // detect close valve

//next
#define PROG_SPACE 5                    // space between values in program screen


// Application variables
String act_screen = "main";
boolean sens_active[SENSORS]={false,false,false};         // is sensor control active
boolean sens_heating[SENSORS]={false,false,false};        // heating for sensor
byte sens_heating_pause[SENSORS]={0,0,0};                 // countdown heating stop when window open  or closed valve are detected, in minutes
boolean sens_heating_pause_active[SENSORS]={false,false,false};  // heating pause was activated
float sens_heating_pause_temp0[SENSORS]={0,0,0};          //save temperature when pause start
float sens_heating_pause_tempB[SENSORS]={0,0,0};          //save back related temperature for pause start
byte temp_back_step_window = (byte)ceil( (float)DELTA_WINDOW_OPEN_TIME / (float)TEMP_HIST_STEP);    // how old temp we are using for comparation if window is open
byte temp_back_step_head = (byte)ceil( (float)DELTA_VALVE_CLOSE_TIME / (float)TEMP_HIST_STEP);    // how old temp we are using for comparation if radiator head is closed, temperature is not growing
long sens_delay[SENSORS][3]={{0,0,0},{0,0,0},{0,0,0}};    // activate sensor delay DDMMHH (Day Month Hour)
boolean heating = false;                                  // control relay for turn on/off heating
byte sens_prg[SENSORS]={2,3,1};                           // number of program for sensor
long time_min_count = millis();                           // variable fo count heating pause
bool active_watchdog = true;                              // activate watchdog

const byte init_temp[PROGRAMS][DAY_STEP] =                // temperature for programs
{
  {23,21,22,20,0,0},
  {22,20,0,0,0,0},
  {21,0,0,0,0,0},
  {23,20,0,0,0,0},
  {22,19,0,0,0,0}
};
const unsigned int init_time[PROGRAMS][DAY_STEP] =        // time points for programs
{
  {600,1000,1500,2200,0,0},                              // number format HHMM, 0 = off
  {600,2300,0,0,0,0},
  {100,0,0,0,0,0},
  {600,2300,0,0,0,0},
  {600,2200,0,0,0,0}
};
byte prg_temp[PROGRAMS][DAY_STEP] =                           // temperature for programs
{
    {23,21,22,20,0,0},
    {22,20,0,0,0,0},
    {21,0,0,0,0,0},
    {23,20,0,0,0,0},
    {22,19,0,0,0,0}
};
unsigned int prg_time[PROGRAMS][DAY_STEP] =        // time points for programs
{
    {600,1000,1500,2200,0,0},                              // number format HHMM, 0 = off
    {600,2300,0,0,0,0},
    {100,0,0,0,0,0},
    {600,2300,0,0,0,0},
    {600,2200,0,0,0,0}
};

unsigned long deltime = 0;
boolean refreshtime = false;

/*****************************************************************************
*
*                          SETUP / LOOP
*
******************************************************************************/
void setup()
{
  //Serial console
  //Serial.begin(9600);
  // RF sensors
  SensorT25::enable(IRQ_PIN);

  //RTC
  //rtc.begin(); new lib
  Wire.begin();

  //LCD
  // set up the LCD's number of columns and rows
  lcd.createChar(0, invert1);
  lcd.createChar(1, invert2);
  lcd.createChar(2, invert3);
  lcd.createChar(7, fire1);
  //lcd.createChar(8, invert1);
  //lcd.createChar(8, fire2);
  lcd.begin(16, 2);
  SafeBLoff(PIN_BL);
  delay(100);
  SafeBLon(PIN_BL);
  lcd.setCursor(0, 0);
  lcd.print("Zone3TermostatN");
  lcd.setCursor(0, 1);
  lcd.print("by Martin Mikala");
  delay(500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("v");
  lcd.print(APP_VERSION_MAIN);
  lcd.print('.');
  lcd.print(APP_VERSION_RELEASE);
  lcd.print('.');
  lcd.print(APP_VERSION_PATCH);
  lcd.print("_");
  lcd.print(EEPROM_VERSION);
  lcd.setCursor(0, 1);
  lcd.print("B:");
  lcd.print(eeprom_read_boots());
  lcd.print(" T:");
  lcd.print(eeprom_read_boottime());

  delay(1500);
  lcd.clear();

  //relay
  pinMode(RELAY_PIN, OUTPUT);

  //EEPROM
  eeprom_init();
  //eeprom_init_progs();
  eeprom_load();

  //watchdog
  wdt_enable(WDTO_4S);  // watchdog counter to 4s

  //init variable
  //reset_temp_hist_all();
  //new boot, write info
  new_boot();
}

void loop()
{
    wdt_reset();
    calc_heating();
    set_relay();
    //temp_history();

    key = keypad.getKey();
    switch (key)
    {
        case KEYLEFT:
          sensor_set(0);                   // convert char of number to byte
          break;
        case KEYUP:
          sensor_set(1);                   // convert char of number to byte
          break;
        case KEYRIGHT:
          sensor_set(2);                   // convert char of number to byte
          break;
        case KEYSET:
          //set_termostat();
          //set_termostat2(); //v2 delete
          break;
    }
      if(lcd_refresh()) print_main_screen();

}

/*****************************************************************************
*
*        functions for recognize open window or closed radiators valve
*
******************************************************************************/

// execute every TEMP_HIST_STEP
// void temp_history()
// {
//     // execute with history step time
//     if(delta_minutes(temp_hist_last_time) >= TEMP_HIST_STEP)
//     {
//         //save_temp_history();
//         //if (active_window_pause) checkHeatingPause();
//         //sprint_temp_hist();
//
//         temp_hist_last_time = millis();
//     }
//
//     //execute every minutes
//     if(delta_minutes(time_min_count) >= 1)
//     {
//         countdownHeatingPause();
//
//         time_min_count = millis();
//     }
//
// }

// void sprint_temp_hist()
// {
//     Serial.print("TEMP HIST:");
//     //Serial.println(rtc.getTimeStr()); new lib
//     t = rtc.now();
//     Serial.print(t.hour(), DEC);
//     Serial.print(':');
//     Serial.print(t.minute(), DEC);
//     Serial.print(':');
//     Serial.print(t.second(), DEC);
//     Serial.println();
//     for(byte s = 0;s < SENSORS;s++)
//     {
//         Serial.print(s);
//         Serial.print('-');
//         for(byte i = 0;i < TEMP_HIST_STEPS;i++)
//         {
//             Serial.print(i);
//             Serial.print('_');
//             Serial.print(prg_temp_hist[s][i]);
//             Serial.print(',');
//         }
//         Serial.print(" T ");
//         Serial.print(SensorT25::getTemperature(s));
//         Serial.print(" - ");
//         Serial.print(getProgTempCurrent(s));
//         Serial.print(" W ");
//         Serial.print(temp_back_step_window);
//         Serial.print(" - ");
//         Serial.print(prg_temp_hist[s][temp_back_step_window] - prg_temp_hist[s][0]);
//         Serial.print(" - ");
//         Serial.print(isWindowOpen(s));
//         Serial.print(" , V ");
//         Serial.print(temp_back_step_head);
//         Serial.print(" - ");
//         Serial.print(prg_temp_hist[s][temp_back_step_head] - prg_temp_hist[s][0]);
//         Serial.print(" - ");
//         Serial.print(isHeadClose(s));
//         Serial.print(" : ");
//         Serial.print(sens_heating_pause[s]);
//         Serial.print(" - ");
//         Serial.print(sens_heating[s]);
//         Serial.println();
//     }
//
// }

//save few last temperatures for sensors in interval
// void save_temp_history()
// {
//     shift_temp_hist();
//     for(byte s = 0;s < SENSORS;s++)
//     {
//         prg_temp_hist[s][0] = SensorT25::getTemperature(s);
//     }
// }

// rotate temperature history
// void shift_temp_hist()
// {
//     for(byte s = 0;s < SENSORS;s++)
//     {
//         for(byte i = TEMP_HIST_STEPS - 1;i > 0;i--)
//         {
//             prg_temp_hist[s][i] = prg_temp_hist[s][i-1];
//         }
//     }
// }

// calc minutes from parameter time
// unsigned int delta_minutes(unsigned long last_lime)
// {
//     unsigned long ctime = millis();
//     unsigned long delta_time = (last_lime < ctime) ? ctime - last_lime : ctime + !(last_lime) + 1;
//     return millis2min(delta_time);
// }

// convert miliseconds to minutes
// int millis2min(long m)
// {
//     return m/60000;
// }

// reset all temperature history
// void reset_temp_hist_all()
// {
//     for(byte s = 0; s < SENSORS; s++)
//     {
//             reset_temp_hist(s);
//     }
// }

// reset one sensor history
// void reset_temp_hist(byte s)
// {
//         for(byte i = 0;i < TEMP_HIST_STEPS;i++)
//         {
//             prg_temp_hist[s][i] = 0.0f;
//         }
// }

//check is temperature go down for room where is heating
boolean isWindowOpen(byte s)
{
        // true if delta temp is larger then treshold and older temp is not 0
        if((prg_temp_hist[s][temp_back_step_window] != 0) && (prg_temp_hist[s][temp_back_step_window] - prg_temp_hist[s][0]) > DELTA_WINDOW_OPEN_C)
        {
            return true;
        }
        else return false;
}

//check is temperature go down for room where is heating
boolean isHeadClose(byte s)
{
        // true if delta temp is smaller then treshold and comparasion temperature is not 0
        if((prg_temp_hist[s][temp_back_step_head] != 0) && ((prg_temp_hist[s][temp_back_step_head] - prg_temp_hist[s][0]) < DELTA_VALVE_CLOSE_C) && (getProgTempCurrent(s) - TEMP_LIMIT > SensorT25::getTemperature(s)))
        {
            return true;
        }
        else return false;
}

// set it only if room is heating and heatin pause is not set and delta temp is larger then treshold
// void setHeatingPause(byte s)
// {
//     Serial.print("BP setHP "); Serial.print(s); Serial.print(' ');
//     if(sens_heating[s] && (sens_heating_pause[s] == 0) && isWindowOpen(s))
//     {
//         Serial.print("true W");
//         sens_heating_pause[s] = HEATING_PAUSE_WINDOW;
//         sens_heating_pause_active[s] = true;                                   // pause is activated
//     }
// }

// check all sensors for heating pause
// void checkHeatingPause()
// {
//     for(byte s = 0; s < SENSORS; s++)
//     {
//             setHeatingPause(s);
//     }
// }

// count down heating pauses
// void countdownHeatingPause()
// {
//     for(byte s = 0; s < SENSORS; s++)
//     {
//         if(sens_heating_pause[s] != 0)
//         {
//             sens_heating_pause[s]--;
//         }
//         // turn of heatin pause status and reset history
//         if(sens_heating_pause[s] == 0 && sens_heating_pause_active[s] == true)
//         {
//             sens_heating_pause_active[s] = false;
//             reset_temp_hist(s);
//         }
//     }
// }

// get if is sensor paused
boolean isSensorPaused(byte s)
{
    return (sens_heating_pause[s] > 0) ? true :false;
}

/*****************************************************************************
*
*                           EEPROM functions
*
******************************************************************************/


//check EEPROM data structure version and initialize it or migrate
void eeprom_init()
{
    byte eeprom_version = EEPROM.read(EEPROM_OFFSET_VERSION);           // read EEPROM version from eeprom
    if(eeprom_version == 0 || eeprom_version == 255)                    // default init value in EEPROM, format EEPROM to initial state
    {
        //eeprom_format(eeprom_version);
        eeprom_format();
    }
    else if (eeprom_version < EEPROM_VERSION)                          // new EEPROM data structure version
    {
        if(eeprom_version < 12)
        {
            eeprom_format_boot();   // chnage word dword read/write EEPROM by eeprom.h function
        }
        eeprom_set_version();
    }
    else if (eeprom_version > EEPROM_VERSION)                          // older EEPROM data structure version
    {
        //eeprom_format(eeprom_version);
        eeprom_set_version();
    }
}

// format EEPROM to initial state and values
//void eeprom_format(byte v)
void eeprom_format()
{
    eeprom_format_act();
    eeprom_format_prg();
    eeprom_format_delay();
    eeprom_format_progs();
    eeprom_format_boot();
    eeprom_set_version();
}

// set initial active values for sensors to false
void eeprom_format_act()
{
    for(byte s = 0; s < SENSORS;s++)
    {
      EEPROM.write(EEPROM_OFFSET_ACT + s,0);
    }
}

// set initial program for sensors to 0 = a
void eeprom_format_prg()
{
    for(byte s = 0; s < SENSORS;s++)
    {
      EEPROM.write(EEPROM_OFFSET_PRG + s,0);
    }
}

// reset delay date for sensors
void eeprom_format_delay()
{
    for(byte s = 0; s < SENSORS;s++)
    {
      for(byte i = 0; i < 3;i++)
      {
       EEPROM.write(EEPROM_OFFSET_DELAY + (s * 3) + i,0);
      }
    }
}

// set initial values for programs 0-4 or a-e
void eeprom_format_progs()
{
    for(byte p = 0;p < PROGRAMS;p++)
    {
        eeprom_reset_progs(p);
    }
    EEPROM.write(EEPROM_OFFSET_PSET, 1);
}

// set initial values for booting log
void eeprom_format_boot()
{
    eeprom_write_boots(0);
    eeprom_write_boottime(0);
}

// set EEPROM structure version
void eeprom_set_version()
{
    EEPROM.write(EEPROM_OFFSET_VERSION, EEPROM_VERSION);
}

//set initial values for program
void eeprom_init_progs()
{
    if(EEPROM.read(EEPROM_OFFSET_PSET) != 1)
    {
        eeprom_format_progs();
    } else
    {
        for(byte p = 0;p < PROGRAMS;p++)
        {
            eeprom_load_progs(p);
        }
    }
}

// load programs values from EEPROM
void eeprom_load_progs(byte p)
{
    for(byte s = 0;s < DAY_STEP;s++)
    {
        prg_temp[p][s] = EEPROM.read(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3));
        prg_time[p][s] = EEPROM.read(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 1) * 100;
        prg_time[p][s] += EEPROM.read(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 2);
    }
}

// write programs to EEPROM
void eeprom_write_progs(byte p)
{
    for(byte s = 0;s < DAY_STEP;s++)
    {
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3), prg_temp[p][s]);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 1, prg_time[p][s] / 100);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 2, prg_time[p][s] % 100);
    }
}

// set initial values for programs
void eeprom_reset_progs(byte p)
{
    for(byte s = 0;s < DAY_STEP;s++)
    {
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3), init_temp[p][s]);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 1, init_time[p][s] / 100);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 2, init_time[p][s] % 100);
    }
}

// load values from EEPROM to variables , example during start
void eeprom_load()
{
    // restore active sensors
    for(byte s = 0; s < SENSORS;s++)
    {
      sens_active[s] = EEPROM.read(EEPROM_OFFSET_ACT + s);
    }
    //restore sensors program
    for(byte s = 0; s < SENSORS;s++)
    {
      sens_prg[s] = EEPROM.read(EEPROM_OFFSET_PRG + s);
    }
    //restore sensors delay
    for(byte s = 0; s < SENSORS;s++)
    {
      for(byte i = 0; i < 3;i++)
      {
       sens_delay[s][i] = EEPROM.read(EEPROM_OFFSET_DELAY + (s * 3) + i);
      }
    }
    //restore programs values
    for(byte s = 0; s < SENSORS;s++)
    {
      eeprom_load_progs(s);
    }

    // EEPROM 12,13
    byte a = 0;
    a = eeprom_read_actwindow();
    if(a == 0 || a == 1) active_window_pause = a;

    a = eeprom_read_actwtdog();
    if(a == 0 || a == 1) active_watchdog = a;
}

// save sensor activation to EEPROM
void eeprom_save_active(byte c)
{
  EEPROM.write(EEPROM_OFFSET_ACT + c,sens_active[c]);
}

// save program for sensor to EEPROM
void eeprom_save_prg(byte c)
{
  EEPROM.write(EEPROM_OFFSET_PRG + c,sens_prg[c]);
}

// save delay for sensor to EEPROM
void eeprom_save_delay(byte c)
{
  EEPROM.write(EEPROM_OFFSET_DELAY + (c*3),sens_delay[c][0]);
  EEPROM.write(EEPROM_OFFSET_DELAY + (c*3)+1,sens_delay[c][1]);
  EEPROM.write(EEPROM_OFFSET_DELAY + (c*3)+2,sens_delay[c][2]);
}

//new boot
void new_boot()
{
    eeprom_inc_boots();
    //eeprom_write_boottime(rtc.getUnixTime(rtc.getTime())); new rtc lib
    t = rtc.now();
    eeprom_write_boottime(t.unixtime());
}

//increment boots counter
void eeprom_inc_boots()
{
    eeprom_write_boots(eeprom_read_boots() + 1);
}

//read boots counter
unsigned int eeprom_read_boots()
{
    return eeprom_read_word(EEPROM_OFFSET_BOOTS);
}

//write to boots counter
void eeprom_write_boots(int v)
{
    eeprom_write_word(EEPROM_OFFSET_BOOTS,v);
}

//read boottime
long eeprom_read_boottime()
{
    return eeprom_read_dword(EEPROM_OFFSET_BOOTTIME);
}


//write boot time
void eeprom_write_boottime(long v)
{
    eeprom_write_dword(EEPROM_OFFSET_BOOTTIME,v);
}

//read window open activation
boolean eeprom_read_actwindow()
{
    return EEPROM.read(EEPROM_OFFSET_ACTWINDOW);
}

//write window open activation
void eeprom_write_actwindow(bool v)
{
    EEPROM.write(EEPROM_OFFSET_ACTWINDOW,v);
}

//read window open activation
boolean eeprom_read_actwtdog()
{
    return EEPROM.read(EEPROM_OFFSET_ACTWTDOG);
}

//write window open activation
void eeprom_write_actwtdog(bool v)
{
    EEPROM.write(EEPROM_OFFSET_ACTWTDOG,v);
}



/*****************************************************************************
*
*                           Set termostat menu and functions
*
******************************************************************************/

//v2 delete
//String menu_settings[]={"PROGRAMY","CAS","OKNO","CASOVAC",""};
//String menu_settings[]={"CAS",""};
//v2 delete
// void stmenu_print(String* menu_array, byte position)
// {
//     lcd.clear();
//     lcd.setCursor(0,0);
//     lcd.print(menu_array[position]);
//     lcd.setCursor(0,1);
//     lcd.print(menu_array[position + 1]);
// }


//v2 delete
//setting of termostart menu , called by SET key
// void set_termostat2()
// {
//     boolean goloop = true;
//     byte menu_position = 0;
//     byte menu_lenght = 4;
//     byte menu_row = 0;
//     bool menu_refresh = false;
//     //stmenu_print(menu_settings,menu_position);
//     lcd.setCursor(0,menu_row);
//     lcd.blink();
//     // loop for choise from menu
//     long etime = millis();
//     while ((goloop == true) && ((millis() - etime) < EXIT_TIME))
//     {
//         wdt_reset();
//         key = keypad.getKey();
//         switch (key)
//         {
//             case KEYLEFT:
//             case KEYRIGHT:
//                 menu_position = 255;
//                 goloop = false;
//                 break;
//             case KEYUP:
//                 menu_position--;
//                 menu_refresh = true;
//                 break;
//             case KEYDOWN:
//                 menu_position++;
//                 menu_refresh = true;
//                 break;
//           case KEYSET:
//             goloop = false;
//             break;
//         }
//         if(key > 0) etime = millis();
//         if(menu_refresh)
//         {
//             if(menu_position > menu_lenght) menu_position = menu_lenght - 1;
//             if(menu_position == menu_lenght) menu_position = 0;
//             //stmenu_print(menu_settings,menu_position);
//             lcd.setCursor(0,menu_row);
//             lcd.blink();
//             menu_refresh = false;
//         }
//     }
//     switch (menu_position)
//     {
//         case 0:
//             break;
//         case 1:
//             //set_time();
//             break;
//         case 2:
//             break;
//         case 3:
//             break;
//         case 255:
//             break;
//     }
//     lcd.clear();
// }

// set time in RTC
// void set_time()
// {
//   t = rtc.now();
//   // time variables for change time
//   tmpYear = t.year();
//   tmpMonth = t.month();
//   tmpDate = t.day();
//   tmpHour = t.hour();
//   tmpMinute = t.minute();
//
//   boolean goloop = true;
//   cur_pos_c = TIME_HCOL;
//   cur_pos_r = TIME_ROW;
//   lcd.clear();
//   //print_set_time(0,0);
//   //print_set_date(0,1);
//   lcd.setCursor(cur_pos_c,cur_pos_r);
//   lcd.blink();
//   long etime = millis();
//   while (goloop && ((millis() - etime) < EXIT_TIME))
//   {
//       wdt_reset();
//       key = keypad.getKey();
//       switch (key)
//       {
//         case KEYLEFT:
//         case KEYRIGHT:
//           //select_number(key);
//           break;
//         case KEYUP:
//         case KEYDOWN:
//           //set_value(key);
//           break;
//         case KEYSET:
//           goloop = false;
//           break;
//       }
//       if(key > 0) etime = millis();
//   }
//   //write_time();                 // write time into RTC
// }

// write time into RTC with approve
void write_time()
{
  // boolean goloop = true;
  // lcd.setCursor(TIME_SCOL + 3,TIME_ROW);
  // lcd.print("SET");
  // delay(1000);
  // lcd.setCursor(TIME_SCOL + 3,TIME_ROW);
  // lcd.blink();
  // long etime = millis();
  // while (goloop && ((millis() - etime) < EXIT_TIME))
  // {
  //     wdt_reset();
  //     key = keypad.getKey();
  //     switch (key)
  //     {
  //       case KEYLEFT:
  //       case KEYRIGHT:
  //       case KEYUP:
  //       case KEYDOWN:
  //         goloop = false;
  //         break;
  //       case KEYSET:
  //         goloop = false;
          // lcd.clear();
          // lcd.print("Set time");
          Clock.setHour(tmpHour);
          Clock.setMinute(tmpMinute);
          Clock.setSecond(0);
          Clock.setDate(tmpDate);
          Clock.setMonth(tmpMonth);
          Clock.setYear(tmpYear - 2000);
  //         break;
  //     }
  //     if(key > 0) etime = millis();
  // }
}

// select number in set time for RTC
// void select_number(byte k)
// {
//   if(cur_pos_r == 0)
//   {
//     switch (cur_pos_c)
//     {
//       case TIME_HCOL:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c = DATE_YCOL + 3;
//             cur_pos_r =1;
//             break;
//           case KEYRIGHT:
//             cur_pos_c++;
//             break;
//         }
//         break;
//       case TIME_HCOL + 1:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c--;
//             break;
//           case KEYRIGHT:
//             cur_pos_c+=2;
//             break;
//         }
//         break;
//       case TIME_MCOL:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c-=2;
//             break;
//           case KEYRIGHT:
//             cur_pos_c++;
//             break;
//         }
//         break;
//       case TIME_MCOL + 1:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c--;
//             break;
//           case KEYRIGHT:
//             cur_pos_c = DATE_DCOL;
//             cur_pos_r = 1;
//             break;
//         }
//         break;
//       }
//   }
//   else
//   {
//     switch (cur_pos_c)
//     {
//       case DATE_DCOL:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c = TIME_MCOL + 1;
//             cur_pos_r =0;
//             break;
//           case KEYRIGHT:
//             cur_pos_c++;
//             break;
//         }
//         break;
//       case DATE_DCOL + 1:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c--;
//             break;
//           case KEYRIGHT:
//             cur_pos_c = DATE_MCOL + 1;
//             break;
//         }
//         break;
//       case DATE_MCOL + 1:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c = DATE_DCOL + 1;
//             break;
//           case KEYRIGHT:
//             cur_pos_c = DATE_YCOL + 3;
//             break;
//         }
//         break;
//       case DATE_YCOL + 3:
//         switch (k)
//         {
//           case KEYLEFT:
//             cur_pos_c = DATE_MCOL + 1;
//             break;
//           case KEYRIGHT:
//             cur_pos_c = TIME_HCOL;
//             cur_pos_r = 0;
//             break;
//         }
//         break;
//       }
//
//   }
//   lcd.setCursor(cur_pos_c,cur_pos_r);
// }

// set value of digit for time
// void set_value(byte k)
// {
//   if(cur_pos_r == 0)
//   {
//     switch (cur_pos_c)
//     {
//       case TIME_HCOL:
//         switch (k)
//         {
//           case KEYUP:
//             tmpHour+=10; //newlib
//             if(tmpHour > 23) tmpHour = 0;
//             break;
//           case KEYDOWN:
//             tmpHour-=10;
//             if(tmpHour > 23) tmpHour = 23;
//             break;
//         }
//         break;
//       case TIME_HCOL + 1:
//         switch (k)
//         {
//           case KEYUP:
//             tmpHour++;
//             if(tmpHour > 23) tmpHour = 0;
//             break;
//           case KEYDOWN:
//             tmpHour--;
//             if(tmpHour > 23) tmpHour = 23;
//             break;
//         }
//         break;
//       case TIME_MCOL:
//         switch (k)
//         {
//           case KEYUP:
//             tmpMinute+=10;
//             if(tmpMinute > 59) tmpMinute = 0;
//             break;
//           case KEYDOWN:
//             tmpMinute-=10;
//             if(tmpMinute > 59) tmpMinute = 59;
//             break;
//         }
//         break;
//       case TIME_MCOL + 1:
//         switch (k)
//         {
//           case KEYUP:
//             tmpMinute++;
//             if(tmpMinute > 59) tmpMinute = 0;
//             break;
//           case KEYDOWN:
//             tmpMinute--;
//             if(tmpMinute > 59) tmpMinute = 59;
//             break;
//         }
//         break;
//       }
//   }
//   else
//   {
//     switch (cur_pos_c)
//     {
//       case DATE_DCOL:
//         switch (k)
//         {
//           case KEYUP:
//             tmpDate+=10;
//             if(tmpDate > 31) tmpDate = 1;
//             break;
//           case KEYDOWN:
//             tmpDate-=10;
//             if(tmpDate > 31) tmpDate = 31;
//             break;
//         }
//         break;
//       case DATE_DCOL + 1:
//         switch (k)
//         {
//           case KEYUP:
//             tmpDate++;
//             if(tmpDate > 31) tmpDate = 1;
//             break;
//           case KEYDOWN:
//             tmpDate--;
//             if(tmpDate > 31) tmpDate = 31;
//             break;
//         }
//         break;
//       case DATE_MCOL + 1:
//         switch (k)
//         {
//           case KEYUP:
//             tmpMonth++;
//             if(tmpMonth > 12) tmpMonth = 1;
//             break;
//           case KEYDOWN:
//             tmpMonth--;
//             if(tmpMonth > 12 || tmpMonth < 1) tmpMonth = 12;
//             break;
//         }
//         break;
//       case DATE_YCOL + 3:
//         switch (k)
//         {
//           case KEYUP:
//             tmpYear++;
//             if(tmpYear > 3000) tmpYear = 2000;
//             break;
//           case KEYDOWN:
//             tmpYear--;
//             if(tmpYear > 3000) tmpYear = 2999;
//             break;
//         }
//         break;
//       }
//   }
//   delay(100);
//   //print_set_time(0,0);
//   //print_set_date(0,1);
//   lcd.setCursor(cur_pos_c,cur_pos_r);
//   lcd.blink();
// }

// print time for set
// void print_set_time(byte c, byte r)
// {
//   lcd.setCursor(c,r);
//   lcd.print("Cas:");
//   lcd.setCursor(c+TIME_HCOL,r);
//   if(tmpHour < 10) lcd.print('0');
//   lcd.print(tmpHour);
//   lcd.print(':');
//   lcd.setCursor(c+TIME_MCOL,r);
//   if(tmpMinute < 10) lcd.print('0');
//   lcd.print(tmpMinute);
//   lcd.print(':');
//   lcd.setCursor(c+TIME_SCOL,r);
//   lcd.print("00");
// }

// print date for set
// void print_set_date(byte c, byte r)
// {
//   lcd.setCursor(c,r);
//   lcd.print("Datum:");
//   lcd.setCursor(c+DATE_DCOL,r);
//   if(tmpDate < 10) lcd.print('0');
//   lcd.print(tmpDate);
//   lcd.print('/');
//   lcd.setCursor(c+DATE_MCOL,r);
//   if(tmpMonth < 10) lcd.print('0');
//   lcd.print(tmpMonth);
//   lcd.print('/');
//   lcd.setCursor(c+DATE_YCOL,r);
//   if(tmpYear < 2000) tmpYear = 2000;
//   lcd.print(tmpYear);
// }

/*****************************************************************************
*
*                      termostat automation functions
*
******************************************************************************/

// check if must heating
void calc_heating()
{
    for(byte c = 0; c < SENSORS; c++)          // loop for all chanes
    {
      if (getProgTempCurrent(c) > SensorT25::getTemperature(c) && sens_active[c] && SensorT25::isValid(c) && (! isSensorDelay(c)) && (! isSensorPaused(c)))
      {
        sens_heating[c] = true;
      }
      else
      {
        sens_heating[c] = false;
      }

      //antifreez - under unfreez temperature
      if(SensorT25::getTemperature(c) < UNFREEZ_TEMP && SensorT25::isValid(c)) sens_heating[c] = true;
    }
    heating = false;
    for(byte c = 0; c < SENSORS; c++)          // loop for all chanels
    {
      if(sens_heating[c]) heating = true;
    }
}

// set relay on or off
void set_relay()
{
  if(millis() - relay_oldtime > RELAY_DELAY) // delay for prevent many changes
  {
    relay_oldtime = millis();
    digitalWrite(RELAY_PIN, ! heating);  // LOW is ON
  }
}

// get if is delay for sensor start
boolean isSensorDelay(byte c)
{
  t = rtc.now();
  long d = (sens_delay[c][1]*10000)+(sens_delay[c][0]*100)+sens_delay[c][2];
  long z = (t.month()*10000L)+(t.day()*100L)+t.hour();
  return (d > z) ? true :false;
}

/*****************************************************************************
*
*                     main sensor operations and screen
*
******************************************************************************/

//print main screen
void print_main_screen()
{
  for(int c = 0; c < SENSORS; c++)          // for each chanels/sensors
  {
    switch (c)
    {
      case 0:
        lcdcol = CH1_MAIN_C;
        lcdrow = CH1_MAIN_R;
        break;
      case 1:
        lcdcol = CH2_MAIN_C;
        lcdrow = CH2_MAIN_R;
        break;
      case 2:
        lcdcol = CH3_MAIN_C;
        lcdrow = CH3_MAIN_R;
        break;
    }

    lcd.setCursor(lcdcol, lcdrow);
    print_chanel(c);                                        // print chanel number , invert mean active


    if(SensorT25::isValid(c))                               // print chanel value if is valid
    {
      print_temperature(c,lcdcol+POS_TEMP,lcdrow);
    }

    print_state(c,lcdcol+POS_STATE,lcdrow);                 // print program of chanel state, off heating - small leter, on heating - uppercase, z - delay
  }
  print_time(T_MAIN_C, T_MAIN_R);                           // print current time from RTC

  print_heating(H_MAIN_C,H_MAIN_R);                         // heating ON by all channels
}

//sensor state and seting
void sensor_set(byte c)
{
  lcd.clear();
  boolean goloop = true;
  print_sensor_state(c);
  long etime = millis();
  while (goloop && ((millis() - etime) < EXIT_TIME))
  {
      wdt_reset();
      key = keypad.getKey();
      switch (key)
      {
        case KEYLEFT:                        //change program
        case KEYRIGHT:
          sensor_prog_change(c,key);
          lcd.clear();
          print_sensor_state(c);
          break;
        case KEYSET:                        // activate sensor
          sensor_activate(c);
          eeprom_save_active(c);
          print_sensor_state(c);
          break;
        case KEYDOWN:                     //back
          goloop = false;
          break;
      }
      if(key > 0) etime = millis();
    if(lcd_refresh()) print_sensor_state(c);
  }
  lcd.clear();
}

// screen for activate sensor and set delay time and date
void sensor_activate(byte c)
{
  lcd.clear();
  print_sensor_activate(c);
  boolean goloop = true;
  delay(1000);
  long etime = millis();
  while (goloop && ((millis() - etime) < EXIT_TIME))
  {
      wdt_reset();
      key = keypad.getKey();
      switch (key)
      {
        case KEYLEFT:            // date delay
        case KEYRIGHT:
            set_date_delay(c,key);
            print_sensor_activate(c);
          break;
        case KEYSET:               //activate-deactivate
          sens_active[c] = (sens_active[c]) ? false : true;
          print_sensor_activate(c);
          break;
        case KEYUP:              //hour delay
            set_time_delay(c);
            print_sensor_activate(c);
          break;
        case KEYDOWN:
          goloop = false;
          break;
      }
      if(key > 0) etime = millis();
    if(lcd_refresh()) print_sensor_activate(c);
  }
  lcd.clear();
  eeprom_save_delay(c);
}

// set delay date
void set_date_delay(byte c, byte k)
{
  t = rtc.now();
  if((sens_delay[c][0] < 1 || sens_delay[c][0] > 31) || (sens_delay[c][1] < 1 || sens_delay[c][1] > 12))
  {
     sens_delay[c][0] = t.day();
     sens_delay[c][1] = t.month();
  }

  switch (k)
  {
    case KEYRIGHT:
      sens_delay[c][0]++;
      if(sens_delay[c][0] > 31)
      {
        sens_delay[c][0] = 1;
        sens_delay[c][1]++;
      }
      if(sens_delay[c][1] > 12) sens_delay[c][1] = 1;
      break;
    case KEYLEFT:
      sens_delay[c][0]--;
      if(sens_delay[c][0] < 1)
      {
        sens_delay[c][0] = 31;
        sens_delay[c][1]--;
      }
      if(sens_delay[c][1] < 1) sens_delay[c][1] = 12;
      break;
  }
}

// set delay time
void set_time_delay(byte c)
{
  t = rtc.now();
  if(sens_delay[c][2] > 23) sens_delay[c][2] = t.hour();
  sens_delay[c][2]++;
  if(sens_delay[c][2] > 23) sens_delay[c][2] = 1;
}

//get time in format HHMM as integer
int getTimeHHMM()
{
  t = rtc.now();
  return t.hour()*100+t.minute();
}

// print current program step for sensor
void print_prog_step(byte c,byte col, byte row)
{
  byte p = sens_prg[c];
  byte s = getProgStep(c);

  lcd.setCursor(col, row);
  lcd.print(prg_time[p][s]);
  lcd.print(' ');
  lcd.print(prg_temp[p][s]);
  lcd.print(' ');
  // print next prog time
  s = getNextProgStep(c);
  lcd.print(prg_time[p][s]);
  lcd.print(' ');
  lcd.print(prg_temp[p][s]);
}

// eveluate currently used program step
byte getProgStep(byte c)
{
  byte p = sens_prg[c];
  unsigned int ct = getTimeHHMM();
  byte i;
  for(i = 0; i < DAY_STEP; i++)
  {
    if (prg_time[p][i] > ct || prg_time[p][i] == 0) // check that prog step time is larger then current time or not set
    {
      break;
    }
  }
  if(i == 0)  // if is got step 0 go during program backward to last valid (not null)
  {
    i = DAY_STEP - 1;
    for(int j = i;j >= 0; j--)
    {
      if(prg_time[p][j] != 0)
      {
        i = j;
        break;
      }
    }
  }
  else i--;  // go back to current prog step
  return i;
}

// next program step for sensor
byte getNextProgStep(byte c)
{
  byte p = sens_prg[c];
  byte i = getProgStep(c);
  i++;
  if(prg_time[p][i] == 0 || i >= DAY_STEP) i = 0;  // set i to day start if step value is 0 or over flow
  return i;
}

//get currently wanted temperature
int getProgTempCurrent(byte c)
{
    return prg_temp[sens_prg[c]][getProgStep(c)];
}

// convert time as int number to HH:MM
String prgInt2Time(int time)
{
  String h = String(time/100);
  String m = String(time%100);
  //if(h.length() < 2) h = ' '+h;
  if(m.length() < 2) m = '0'+m;
  return h+':'+m;
}

// chnage program of sensor
void sensor_prog_change(byte c, int k)
{
  switch (k)
  {
    case KEYLEFT:
      sens_prg[c]--;
      break;
    case KEYRIGHT:
      sens_prg[c]++;
      break;
  }

  if(sens_prg[c] == 255 ) {sens_prg[c] = PROGRAMS-1;}
  else if(sens_prg[c] >= PROGRAMS) {sens_prg[c] = 0;}

  eeprom_save_prg(c);
}

// print sensor state screen
void print_sensor_state(byte c)
{
  print_sensor_state_R0(c);
  print_prog_step(c,0,1);
}

// print sensor state - row 0
void print_sensor_state_R0(byte c)
{
  lcdrow=INF_SENS_R;
  lcdcol=CH_SENS_C;
  lcd.setCursor(lcdcol, lcdrow);
  print_chanel(c);
  print_temperature(c,lcdcol+POS_TEMP,lcdrow);
  lcdcol=AGE_SENS_C;
  lcd.setCursor(lcdcol, lcdrow);
  lcd.print(SensorT25::getValueAge(c));
  lcd.print('s');
  lcd.print(sens_heating_pause[c]);
  lcd.print('m');
  lcdcol=PRG_SENS_C-2;
  lcd.setCursor(lcdcol, lcdrow);
  if(isSensorDelay(c)) lcd.print('Z');
  lcdcol=PRG_SENS_C;
  lcd.setCursor(lcdcol, lcdrow);
  lcd.print(getSensorProg(c));
}

// print sensor state
void print_sensor_activate(byte c)
{
  print_sensor_state_R0(c);
  lcd.setCursor(0,1);
  print_sensor_delay(c);
}

// print delay values for sensor
void print_sensor_delay(byte c)
{
  if(sens_delay[c][0] < 10) lcd.print('0');
  lcd.print(sens_delay[c][0]);
  lcd.print('/');
  if(sens_delay[c][1] < 10) lcd.print('0');
  lcd.print(sens_delay[c][1]);
  lcd.print(' ');
  if(sens_delay[c][2] < 10) lcd.print('0');
  lcd.print(sens_delay[c][2]);
  lcd.print(':');
  lcd.print("00");
}

//refresh lcd delay
boolean lcd_refresh()
{
  if(millis() - deltime > 1000)
  {
    deltime = millis();
    return true;
  }
  else
  {
    return false;
  }
}

// print simbol of heating, relay is ON
void print_heating(byte col, byte row)
{
  lcd.setCursor(col, row);
  if(heating)
  {
    lcd.write(0x07);
    lcd.setCursor(col, row);
    lcd.blink();
  }
  else
  {
    lcd.write(' ');
    lcd.noBlink();
  }
}

//print sensor state and program
void print_state(byte c, byte col, byte row)
{
    char x;
    lcd.setCursor(col, row);
    if(isSensorDelay(c)) x = 'z';
    else if(isSensorPaused(c)) x = 'p';
    else x = getSensorProg(c);
    lcd.print(x);
}

// Return char for program number A or a to E or e
char getSensorProg(byte c)
{
  return (sens_heating[c]) ? sens_prg[c]+65 : sens_prg[c]+97;
}

void print_chanel(byte c)
{
  lcd.write((sens_active[c]) ? c : (c+49));
}

// print temperature for sensor on LCD
void print_temperature(byte c, byte col, byte row)
{
  if (SensorT25::getTemperature(c) >= 0) {col++;}
  if (SensorT25::getTemperature(c) <= 9) {col++;}
  lcd.setCursor(col, row);
  lcd.print(SensorT25::getTemperature(c),1);
}

// print time on LCD
void print_time(byte col, byte row)
{
  lcd.setCursor(col, row);
  t = rtc.now();
  if(t.hour() < 10) lcd.print(' ');
  lcd.print(t.hour(), DEC);
  lcd.print(':');
  if(t.minute() < 10) lcd.print('0');
  lcd.print(t.minute(), DEC);
  lcd.print(':');
  if(t.second() < 10) lcd.print('0');
  lcd.print(t.second(), DEC);
}
