#include <Arduino.h>
#include <Math.h>
#include <SensorT25.h>
#include <LiquidCrystal.h>
#include <DS3231.h>
#include <Wire.h>
#include <DFR_KeyMM.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <WiFiEsp.h>
#include <WifiAuth.h>              //local library with authentication to WiFi
//char ssid[] = "SIDD";                     // your network SSID (name)
//char pass[] = "...";                      // your network password
#include <SoftwareSerial.h>


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
1x ESP-01 ESP8266
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
//#include <SensorT25.h>
#define IRQ_PIN 2      // RF input with irq
#define SENSORS 3

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

//#include <SoftwareSerial.h>
SoftwareSerial Serial1(A3, A2); // RX, TX

//web
char webserver[] = "www.miklik.cz";
int webport = 80;
String content;

// Initialize the Ethernet client object
int status = WL_IDLE_STATUS;                // the Wifi radio's status
WiFiEspClient client;

//#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_RS,  PIN_EN,  PIN_d4,  PIN_d5,  PIN_d6,  PIN_d7);
byte lcdrow = 0;
byte lcdcol = 0;

//RTC I2C
//#include <DS3231.h>
//#include <Wire.h>
DS3231 Clock;
RTClib rtc;
DateTime t;
DateTime webtime;
unsigned int tmpYear;
byte tmpMonth;
byte tmpDate;
byte tmpHour;
byte tmpMinute;

//keypad
//#include <DFR_KeyMM.h>
DFR_KeyMM keypad;
int key;

//relay
#define RELAY_PIN 11
#define RELAY_DELAY 1000             // cheange relay state only every 1000 ms
long relay_oldtime = millis();

//EEPROM
//#include <EEPROM.h>
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
//const *uint16_t EEPROM_OFFSET_BOOTS = EEPROM_OFFSET_PROGS + (PROGRAMS * DAY_STEP * 3);  // 108-109 - boots counter 2B
#define EEPROM_OFFSET_BOOTTIME EEPROM_OFFSET_BOOTS + 2                       // 110-113 - boot time and date 4B
//EEPROM 12 extension for window open check
#define EEPROM_OFFSET_ACTWTDOG EEPROM_OFFSET_BOOTTIME + 4                   // 114 - active watchdog 1B
//EEPROM 13 extension for window open check
#define EEPROM_OFFSET_ACTWINDOW EEPROM_OFFSET_ACTWTDOG + 1                   // 114 - active window oen detection 1B
#define EEPROM_OFFSET_NEXT EEPROM_OFFSET_ACTWINDOW + 1                       // 1st free byte
boolean rom_change = false;

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

  // Control keys
#define KEYUP 3
#define KEYDOWN 4
#define KEYLEFT 2
#define KEYRIGHT 5
#define KEYSET 1

  //wifi comunication
#define WEB_REFRESH 5                     // comunication in minutes
unsigned long webdelay;

// Application variables
boolean sens_active[SENSORS]={false,false,false};         // is sensor control active
boolean sens_heating[SENSORS]={false,false,false};        // heating for sensor
long sens_delay[SENSORS][3]={{0,0,0},{0,0,0},{0,0,0}};    // activate sensor delay DDMMHH (Day Month Hour)
boolean heating = false;                                  // control relay for turn on/off heating
byte sens_prg[SENSORS]={2,3,1};                           // number of program for sensor

byte prg_temp[PROGRAMS][DAY_STEP] =                           // temperature for programs
{
    {23,21,22,20,0,0},
    {22,20,0,0,0,0},
    {21,0,0,0,0,0},
    {23,20,0,0,0,0},
    {22,19,0,0,0,0}
};
byte (*init_temp)[PROGRAMS][DAY_STEP] = &prg_temp;

unsigned int prg_time[PROGRAMS][DAY_STEP] =        // time points for programs
{
    {600,1000,1500,2200,0,0},                              // number format HHMM, 0 = off
    {600,2300,0,0,0,0},
    {100,0,0,0,0,0},
    {600,2300,0,0,0,0},
    {600,2200,0,0,0,0}
};
unsigned int (*init_time)[PROGRAMS][DAY_STEP] = &prg_time;

unsigned long deltime = 0;
boolean refreshtime = false;


/*****************************************************************************
*
*                           EEPROM functions
*
******************************************************************************/

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

// set initial values for programs
 void eeprom_reset_progs(byte p)
{
     for(byte s = 0;s < DAY_STEP;s++)
     {
         EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3), (*init_temp)[p][s]);
         EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 1, (*init_time)[p][s] / 100);
         EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 2, (*init_time)[p][s] % 100);
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

//write to boots counter
void eeprom_write_boots(uint16_t v)
{
    eeprom_write_word(EEPROM_OFFSET_BOOTS,v);
}

//write boot time
void eeprom_write_boottime(long v)
{
    eeprom_write_dword(EEPROM_OFFSET_BOOTTIME,v);
}

// set EEPROM structure version
void eeprom_set_version()
{
    EEPROM.write(EEPROM_OFFSET_VERSION, EEPROM_VERSION);
}

// set initial values for booting log
void eeprom_format_boot()
{
    eeprom_write_boots(0);
    eeprom_write_boottime(0);
}

// format EEPROM to initial state and values
void eeprom_format()
{
    eeprom_format_act();
    eeprom_format_prg();
    eeprom_format_delay();
    eeprom_format_progs();
    eeprom_format_boot();
    eeprom_set_version();
}


//check EEPROM data structure version and initialize it or migrate
void eeprom_init()
{
    byte eeprom_version = EEPROM.read(EEPROM_OFFSET_VERSION);           // read EEPROM version from eeprom
    if(eeprom_version == 0 || eeprom_version == 255)                    // default init value in EEPROM, format EEPROM to initial state
    {
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
        eeprom_set_version();
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

//read boots counter
unsigned int eeprom_read_boots()
{
    return eeprom_read_word(EEPROM_OFFSET_BOOTS);
}

//increment boots counter
void eeprom_inc_boots()
{
    eeprom_write_boots(eeprom_read_boots() + 1);
}

//new boot
void new_boot()
{
    eeprom_inc_boots();
    t = rtc.now();
    eeprom_write_boottime(t.unixtime());
}


//read boottime
long eeprom_read_boottime()
{
    return eeprom_read_dword(EEPROM_OFFSET_BOOTTIME);
}

/*****************************************************************************
*
*                           Set termostat menu and functions
*
******************************************************************************/

// write time into RTC with approve
void write_time()
{
          Clock.setHour(tmpHour);
          Clock.setMinute(tmpMinute);
          Clock.setSecond(0);
          Clock.setDate(tmpDate);
          Clock.setMonth(tmpMonth);
          Clock.setYear(tmpYear - 2000);
}

/*****************************************************************************
*
*                           FUNCTIONS
*
******************************************************************************/

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

// convert time as int number to HH:MM
String prgInt2Time(int time)
{
  String h = String(time/100);
  String m = String(time%100);
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

// get if is delay for sensor start
boolean isSensorDelay(byte c)
{
  t = rtc.now();
  long d = (sens_delay[c][1]*10000)+(sens_delay[c][0]*100)+sens_delay[c][2];
  long z = (t.month()*10000L)+(t.day()*100L)+t.hour();
  return (d > z) ? true :false;
}

// Return char for program number A or a to E or e
char getSensorProg(byte c)
{
  return (sens_heating[c]) ? sens_prg[c]+65 : sens_prg[c]+97;
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
  lcdcol=PRG_SENS_C-2;
  lcd.setCursor(lcdcol, lcdrow);
  if(isSensorDelay(c)) lcd.print('Z');
  lcdcol=PRG_SENS_C;
  lcd.setCursor(lcdcol, lcdrow);
  lcd.print(getSensorProg(c));
}

// print sensor state screen
void print_sensor_state(byte c)
{
  print_sensor_state_R0(c);
  print_prog_step(c,0,1);
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

// print sensor state
void print_sensor_activate(byte c)
{
  print_sensor_state_R0(c);
  lcd.setCursor(0,1);
  print_sensor_delay(c);
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
    //else if(isSensorPaused(c)) x = 'p';
    else x = getSensorProg(c);
    lcd.print(x);
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

//get currently wanted temperature
int getProgTempCurrent(byte c)
{
    return prg_temp[sens_prg[c]][getProgStep(c)];
}

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
      if (getProgTempCurrent(c) > SensorT25::getTemperature(c) && sens_active[c] && SensorT25::isValid(c) && (! isSensorDelay(c)))
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

void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void init_wifi()
{
  lcd.setCursor(0, 0);
  lcd.print("Start wifi");
  //delay(500);

  //Wifi
  lcd.setCursor(0, 1);
  WiFi.init(&Serial1);
  if (WiFi.status() == WL_NO_SHIELD) {
    lcd.print("WiFi not present");
    // don't continue
    return;
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    lcd.print("Try wifi:");
    lcd.print(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  //printWifiStatus();

  // you're connected now, so print out the data
  lcd.setCursor(0, 1);
  lcd.print("Connected ");
  lcd.print(ssid);
  delay(500);
  lcd.clear();
}

/*****************************************************************************
*
*                          WEB methods
*
******************************************************************************/

void webget_config()
{
  SensorT25::disable(IRQ_PIN);
  String page = "GET /config.php";
  //Serial.println("GET page");
  if (client.connect(webserver, webport)) {
    //Serial.println("Connected to server");
    client.print(page);
    //client.print("?");
    //client.print(content);
    client.println(" HTTP/1.1");
    client.println("Host: z3t.miklik.cz");
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Connection: close");
    client.println();
  }
  // loop for response from WEB
  String webreq;
  String webdata;
  bool webdata_valid = false;
  while(client.connected()) {
    while(client.available()) {
      webreq = client.readStringUntil('\n');
      //Serial.println(webreq);
      //Serial.write(client.read());
      if (webreq.length() == 1 && ! webdata_valid) {
        webreq = client.readStringUntil('\n');
        //Serial.println(webreq);
        //webdata_lenght = webreq.toInt();
        webdata = client.readStringUntil('\n');
        //Serial.println(webdata);
        webdata_valid = true;
      }
      //wdt_reset();
      }
      //Serial.print('#');
      Serial.println(webdata);
      webtime = webdata.toDouble();
      //webtime = rtc.now();
      // Serial.println(webtime.year());
      // Serial.println(webtime.month());
      // Serial.println(webtime.day());
      // Serial.println(webtime.hour());
      // Serial.println(webtime.minute());
      // Serial.println(webtime.second());
      // Serial.println(webtime.unixtime());
      // set time in RTC
      Clock.setHour(webtime.hour());
      Clock.setMinute(webtime.minute());
      Clock.setSecond(webtime.second());
      Clock.setDate(webtime.day());
      Clock.setMonth(webtime.month());
      Clock.setYear(webtime.year() - 2000);

  }
  if (!client.connected()) {
    Serial.println();
    Serial.println("Disconnecting from server...");
    client.stop();
  }
  SensorT25::enable(IRQ_PIN);
}

/*****************************************************************************
*
*                          SETUP / LOOP
*
******************************************************************************/
void setup()
{
  //Serial console
  Serial.begin(9600);
  // SW serial console
  Serial1.begin(9600);  // ESP must be set to the same speed AT+UART_DEF=9600,8,1,0,1

  //RTC
  Wire.begin();

  //LCD
  // set up the LCD's number of columns and rows
  lcd.createChar(0, invert1);
  lcd.createChar(1, invert2);
  lcd.createChar(2, invert3);
  lcd.createChar(7, fire1);
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
  eeprom_load();

  //init variable
  new_boot();

  // initialize connection to Wifi
  init_wifi();

  // get time from web
  webget_config();

  //watchdog
  wdt_enable(WDTO_4S);  // watchdog counter to 4s

  // RF sensors
  SensorT25::enable(IRQ_PIN);
}

void loop()
{
    wdt_reset();
    calc_heating();
    set_relay();

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
          break;
    }
      if(lcd_refresh()) print_main_screen();

}
