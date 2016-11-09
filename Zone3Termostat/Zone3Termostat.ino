#include <Arduino.h>

/* 3 zone wireless termostat used 433Mhz temperature sensors T25
Martin Mikala (2016) dev@miklik.cz

v1.1 9.11.2016 - extension for set programs

Devices:
1x Arduino UNO
3x Sencor T25 433MHz sensor http://www.sencor.eu/wireless-sensor/sws-t25
1x RXB6 - 433MHz receiver
1x LCD keypad shield V0.1 LCD1602A+buttons
1x RTC DS3231
1x relay module HL-51 250V/10A

*/

#define DAY_STEP 6
#define PROGRAMS 5

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

// fire2
/*
byte fire2[8] = {
	0b00000,
	0b00000,
	0b00010,
	0b00010,
	0b00101,
	0b01001,
	0b01001,
	0b00110
};
*/

#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(PIN_RS,  PIN_EN,  PIN_d4,  PIN_d5,  PIN_d6,  PIN_d7);
byte lcdrow = 0;
byte lcdcol = 0;

//RTC I2C
#include <DS3231.h>
DS3231 rtc(SDA,SCL);
Time t;

//keypad
//#define KEY_APIN A0
#include <DFR_KeyMM.h>
//DFR_KeyMM keypad(KEY_APIN);
DFR_KeyMM keypad;               // A0 is wired in LCD keypad shield
int key;

//relay
#define RELAY_PIN 11
#define RELAY_DELAY 1000             // cheange relay state only every 1000 ms
long relay_oldtime = millis();

//EEPROM
#include <EEPROM.h>
//#define EEPROM_BYTES 7           // 1x on/off, 3x active sensor, 3x program sensor
#define EEPROM_OFFSET 0                                   // ERRPROM OFFSET
#define EEPROM_OFFSET_DATA EEPROM_OFFSET + 1              // offset for data start
#define EEPROM_OFFSET_ACT EEPROM_OFFSET_DATA              // 1-3 ACTIVATED SENSORS    3B
#define EEPROM_OFFSET_PRG EEPROM_OFFSET_ACT + SENSORS     // 4-6 SENSORS PROGRAM      3B
#define EEPROM_OFFSET_DELAY EEPROM_OFFSET_PRG + SENSORS   // 7-15 DELAY DATE and HOUR  3x3B = DDMM+HH
#define EEPROM_OFFSET_PSET EEPROM_OFFSET_DELAY + 9       // 16 Termostat programs are in EEPROM 1B
#define EEPROM_OFFSET_PROGS EEPROM_OFFSET_PSET + 1       // 17 - 107 Termostat programs - 5x6x3B = 6 steps temperature + HHMM for programs
#define EEPROM_OFFSET_NEXT EEPROM_OFFSET_PROGS + (PROGRAMS * DAY_STEP * 3)  // 1st free byte
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
#define MAX_TEMP 30               // Max temperature set
#define MIN_TEMP 4                // Min temperature
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

//next
#define PROG_SPACE 5

// Application variables
String act_screen = "main";
boolean sens_active[SENSORS]={false,false,false};         // is sensor control active
boolean sens_heating[SENSORS]={false,false,false};        // heating for sensor
long sens_delay[SENSORS][3]={{0,0,0},{0,0,0},{0,0,0}};    // activate sensor delay DDMMHH (Day Month Hour)
boolean heating = false;                                  // control relay for turn on/off heating
byte sens_prg[SENSORS]={2,3,1};                           // number of program for sensor

const byte init_temp[PROGRAMS][DAY_STEP] =                           // temperature for programs
{
  {23,21,23,20,0,0},
  {22,20,21,18,0,0},
  {22,0,0,0,0,0},
  {22,21,22,21,20,18},
  {18,0,0,0,0,0}
};
const unsigned int init_time[PROGRAMS][DAY_STEP] =        // time points for programs
{
  {600,1000,1500,2300,0,0},                              // number format HHMM, 0 = off
  {600,1000,1500,2300,0,0},
  {100,0,0,0,0,0},
  {600,900,1800,2000,2200,2300},
  {100,0,0,0,0,0}
};

byte prg_temp[PROGRAMS][DAY_STEP] =                           // temperature for programs
{
  {23,21,23,20,0,0},
  {22,20,21,18,0,0},
  {22,0,0,0,0,0},
  {22,21,22,21,20,18},
  {18,0,0,0,0,0}
};
unsigned int prg_time[PROGRAMS][DAY_STEP] =        // time points for programs
{
  {600,1000,1500,2300,0,0},                              // number format HHMM, 0 = off
  {600,1000,1500,2300,0,0},
  {100,0,0,0,0,0},
  {600,900,1800,2000,2200,2300},
  {100,0,0,0,0,0}
};

unsigned long deltime = 0;
boolean refreshtime = false;

void setup()
{
  // RF sensors
  SensorT25::enable(IRQ_PIN);
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
  delay(500);
  SafeBLon(PIN_BL);
  lcd.setCursor(0, 0);
  lcd.print("3ZoneTermostat");
  lcd.setCursor(0, 1);
  lcd.print("by Martin Mikala");
  delay(2000);
  lcd.clear();

  //RTC
  rtc.begin();

  //relay
  pinMode(RELAY_PIN, OUTPUT);

  //EEPROM
  eeprom_init_progs();
  eeprom_load();
}

void loop()
{
  calc_heating();
  set_relay();
  //if (keypad.isKey())
  //{
    key = keypad.getKey();
    //keypad.buttonRelease();
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
          //delay(1000);
          set_termostat();
          break;
    }
  //}
  //else
  //{
      if(lcd_refresh()) print_main_screen();
  //}
}

void eeprom_init_progs()
{
    if(EEPROM.read(EEPROM_OFFSET_PSET) != 1)
    {
        for(byte p = 0;p < PROGRAMS;p++)
        {
            eeprom_reset_prog(p);
        }
        EEPROM.write(EEPROM_OFFSET_PSET, 1);
    } else
    {
        for(byte p = 0;p < PROGRAMS;p++)
        {
            eeprom_load_prog(p);
        }
    }
}

void eeprom_load_prog(byte p)
{
    for(byte s = 0;s < DAY_STEP;s++)
    {
        prg_temp[p][s] = EEPROM.read(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3));
        prg_time[p][s] = EEPROM.read(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 1) * 100;
        prg_time[p][s] += EEPROM.read(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 2);
    }
}

void eeprom_write_prog(byte p)
{
    for(byte s = 0;s < DAY_STEP;s++)
    {
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3), prg_temp[p][s]);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 1, prg_time[p][s] / 100);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 2, prg_time[p][s] % 100);
    }
}

void eeprom_reset_prog(byte p)
{
    for(byte s = 0;s < DAY_STEP;s++)
    {
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3), init_temp[p][s]);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 1, init_time[p][s] / 100);
        EEPROM.write(EEPROM_OFFSET_PROGS + (p * DAY_STEP * 3) + (s * 3) + 2, init_time[p][s] % 100);
    }
}

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
}

void eeprom_save_active(byte c)
{
  EEPROM.write(EEPROM_OFFSET_ACT + c,sens_active[c]);
}

void eeprom_save_prg(byte c)
{
  EEPROM.write(EEPROM_OFFSET_PRG + c,sens_prg[c]);
}

void eeprom_save_delay(byte c)
{
  EEPROM.write(EEPROM_OFFSET_DELAY + (c*3),sens_delay[c][0]);
  EEPROM.write(EEPROM_OFFSET_DELAY + (c*3)+1,sens_delay[c][1]);
  EEPROM.write(EEPROM_OFFSET_DELAY + (c*3)+2,sens_delay[c][2]);
}

void set_termostat()
{
    boolean goloop = true;
    byte menu_position = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PROGRAMS");
    lcd.setCursor(0,1);
    lcd.print("TIME");
    lcd.setCursor(0,0);
    lcd.blink();
    long etime = millis();
    while (goloop && ((millis() - etime) < EXIT_TIME))
    {
        key = keypad.getKey();
        switch (key)
        {
            case KEYLEFT:
            case KEYRIGHT:
                menu_position = 255;
                goloop = false;
                break;
            case KEYUP:
                menu_position = 0;
                lcd.setCursor(0,menu_position);
                lcd.blink();
                break;
            case KEYDOWN:
                menu_position = 1;
                lcd.setCursor(0,menu_position);
                lcd.blink();
                break;
          case KEYSET:
            goloop = false;
            break;
        }
        if(key > 0) etime = millis();
      //}
    }
    switch (menu_position)
    {
        case 0:
            set_programs();
            break;
        case 1:
            set_time();
            break;
        case 255:
            break;
    }
    lcd.clear();
}

void set_programs()
{
    boolean goloop = true;
    byte p = 0;     // program numbers
    byte s = 0;     // program step
    byte r = 0;     // row of display
    byte d1 = 0;     // roll display temp variable
    byte d2 = 0;
    lcd.clear();
    p = select_program();
    reset_program(p);
    print_program(p);
    lcd.setCursor(s,r);
    lcd.blink();
    r = 1;      // row for temperature
    long etime = millis();
    // change times for programs
    while (goloop && ((millis() - etime) < EXIT_TIME*2))
    {
        key = keypad.getKey();
        switch (key)
        {
            case KEYLEFT:
                s--;
                if(s > DAY_STEP) s = DAY_STEP - 1;
                break;
            case KEYRIGHT:
                s++;
                if(s > DAY_STEP - 1) s = 0;
                break;
            case KEYUP:
                prg_temp[p][s]++;
                if(prg_temp[p][s] > MAX_TEMP) prg_temp[p][s] = MAX_TEMP;
                if(prg_temp[p][s] < MIN_TEMP) prg_temp[p][s] = MIN_TEMP;
                lcd.print(prg_temp[p][s]);
                if(prg_temp[p][s] < 10) lcd.print(' ');
                break;
            case KEYDOWN:
                prg_temp[p][s]--;
                if(prg_temp[p][s] < MIN_TEMP) prg_temp[p][s] = 0;
                if(prg_temp[p][s] > MAX_TEMP) prg_temp[p][s] = 0;
                if(s == 0 & prg_temp[p][s] == 0) prg_temp[p][s] = MIN_TEMP;
                lcd.print(prg_temp[p][s]);
                if(prg_temp[p][s] < 10) lcd.print(' ');
                break;
          case KEYSET:
            goloop = false;
            break;
        }
        if(key > 0) etime = millis();
        lcd.setCursor(s*PROG_SPACE,r);

        //move screen to second part
        if(s > 2 & d1 == 0)
        {
            for(int i = 0;i < 3*PROG_SPACE;i++)
            {
                lcd.scrollDisplayLeft();
            }
            d1 = 1;
        }

        //move screen to first part
        if(s < 3 & d1 == 1)
        {
            for(int i = 0;i < 3*PROG_SPACE;i++)
            {
                lcd.scrollDisplayRight();
            }
            d1 = 0;
        }
    }

    // set program times
    goloop = true;
    r = 0;      // row for time
    etime = millis();
    // change times for programs
    while (goloop && ((millis() - etime) < EXIT_TIME*2))
    {
        key = keypad.getKey();
        switch (key)
        {
            case KEYLEFT:
                s--;
                if(s > DAY_STEP) s = DAY_STEP - 1;
                break;
            case KEYRIGHT:
                s++;
                if(s > DAY_STEP - 1) s = 0;
                break;
            case KEYUP:
                prg_time[p][s] += 100;
                if(prg_time[p][s] > 2300) prg_time[p][s] = 0;
                // time cannot be larger than next time
                if(s < DAY_STEP - 1 & prg_time[p][s] > prg_time[p][s + 1]) prg_time[p][s] = prg_time[p][s + 1];
                lcd.print(prg_time[p][s]);
                if(prg_time[p][s] < 1000) lcd.print(' ');
                break;
            case KEYDOWN:
                prg_time[p][s] -= 100;
                if(prg_time[p][s] > 2300) prg_time[p][s] = 2300;
                // time cannot be smaller than previous times
                if(s > 0 & prg_time[p][s] < prg_time[p][s - 1]) prg_time[p][s] = prg_time[p][s - 1];
                lcd.print(prg_time[p][s]);
                if(prg_time[p][s] < 1000) lcd.print(' ');
                break;
          case KEYSET:
            goloop = false;
            break;
        }
        if(key > 0) etime = millis();
        lcd.setCursor(s*PROG_SPACE,r);

        //move screen to second part
        if(s > 2 & d1 == 0)
        {
            for(int i = 0;i < 3*PROG_SPACE;i++)
            {
                lcd.scrollDisplayLeft();
            }
            d1 = 1;
        }

        //move screen to first part
        if(s < 3 & d1 == 1)
        {
            for(int i = 0;i < 3*PROG_SPACE;i++)
            {
                lcd.scrollDisplayRight();
            }
            d1 = 0;
        }
    }

    eeprom_write_prog(p);
}

void reset_program(byte p)
{
    boolean goloop = true;
    byte menu_position = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("SET");
    lcd.setCursor(0,1);
    lcd.print("RESET");
    lcd.setCursor(0,0);
    lcd.blink();
    long etime = millis();
    while (goloop && ((millis() - etime) < EXIT_TIME))
    {
        key = keypad.getKey();
        switch (key)
        {
            case KEYUP:
                menu_position = 0;
                lcd.setCursor(0,menu_position);
                lcd.blink();
                break;
            case KEYDOWN:
                menu_position = 1;
                lcd.setCursor(0,menu_position);
                lcd.blink();
                break;
          case KEYSET:
            goloop = false;
            break;
        }
        if(key > 0) etime = millis();
      //}
    }
    switch (menu_position)
    {
        case 0:
            break;
        case 1:
            eeprom_reset_prog(p);
            eeprom_load_prog(p);
            break;
        case 255:
            break;
    }
    lcd.clear();
}

byte select_program()
{
    boolean goloop = true;
    byte p = 0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("PROGRAM: ");
    for (byte i = 0; i < PROGRAMS; i++)
    {
        lcd.setCursor(i,1);
        lcd.print(char(i+97));        // print char of program a - e,
    }

    lcd.blink();
    long etime = millis();
    while (goloop && ((millis() - etime) < EXIT_TIME))
    {
        key = keypad.getKey();
        switch (key)
        {
            case KEYLEFT:
            case KEYDOWN:
                p--;
                if(p > PROGRAMS) p = PROGRAMS - 1;
                break;
            case KEYRIGHT:
            case KEYUP:
                p++;
                if(p > PROGRAMS - 1) p = 0;
                break;
          case KEYSET:
            goloop = false;
            break;
        }
        if(key > 0) etime = millis();
        lcd.setCursor(p,1);
    }
    return p;
}

// set RTC
void set_time()
{
  t = rtc.getTime();
  boolean goloop = true;
  cur_pos_c = TIME_HCOL;
  cur_pos_r = TIME_ROW;
  lcd.clear();
  print_set_time(0,0);
  print_set_date(0,1);
  lcd.setCursor(cur_pos_c,cur_pos_r);
  //lcd.cursor();
  lcd.blink();
  //delay(5000);
  long etime = millis();
  while (goloop && ((millis() - etime) < EXIT_TIME))
  {
    //if (keypad.isKey())
    //{
      key = keypad.getKey();
      //keypad.buttonRelease();
      //delay(50);
      switch (key)
      {
        case KEYLEFT:
        case KEYRIGHT:
          select_number(key);
          break;
        case KEYUP:
        case KEYDOWN:
          set_value(key);
          break;
        case KEYSET:
          //delay(1000);   // ?????
          goloop = false;
          break;
      }
      if(key > 0) etime = millis();
    //}
  }
  write_time();
}

void write_time()
{
  boolean goloop = true;
  lcd.setCursor(TIME_SCOL + 3,TIME_ROW);
  lcd.print("SET");
  delay(1000);
  lcd.setCursor(TIME_SCOL + 3,TIME_ROW);
  lcd.blink();
  long etime = millis();
  while (goloop && ((millis() - etime) < EXIT_TIME))
  {
    //if (keypad.isKey())
    //{
      key = keypad.getKey();
      //keypad.buttonRelease();
      //delay(50);
      switch (key)
      {
        case KEYLEFT:
        case KEYRIGHT:
        case KEYUP:
        case KEYDOWN:
          goloop = false;
          break;
        case KEYSET:
          goloop = false;
          lcd.clear();
          lcd.print("Set time");
          rtc.setTime(t.hour,t.min,0);
          rtc.setDate(t.date,t.mon,t.year);
          //delay(2000);
          break;
      }
      if(key > 0) etime = millis();
    //}
  }
}

void select_number(byte k)
{
  if(cur_pos_r == 0)
  {
    switch (cur_pos_c)
    {
      case TIME_HCOL:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c = DATE_YCOL + 3;
            cur_pos_r =1;
            break;
          case KEYRIGHT:
            cur_pos_c++;
            break;
        }
        break;
      case TIME_HCOL + 1:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c--;
            break;
          case KEYRIGHT:
            cur_pos_c+=2;
            break;
        }
        break;
      case TIME_MCOL:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c-=2;
            break;
          case KEYRIGHT:
            cur_pos_c++;
            break;
        }
        break;
      case TIME_MCOL + 1:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c--;
            break;
          case KEYRIGHT:
            cur_pos_c = DATE_DCOL;
            cur_pos_r = 1;
            break;
        }
        break;
      }
  }
  else
  {
    switch (cur_pos_c)
    {
      case DATE_DCOL:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c = TIME_MCOL + 1;
            cur_pos_r =0;
            break;
          case KEYRIGHT:
            cur_pos_c++;
            break;
        }
        break;
      case DATE_DCOL + 1:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c--;
            break;
          case KEYRIGHT:
            cur_pos_c = DATE_MCOL + 1;
            break;
        }
        break;
      /*
      case DATE_MCOL:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c-=2;
            break;
          case KEYRIGHT:
            cur_pos_c++;
            break;
        }
        break;
      */
      case DATE_MCOL + 1:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c = DATE_DCOL + 1;
            break;
          case KEYRIGHT:
            cur_pos_c = DATE_YCOL + 3;
            break;
        }
        break;
      /*
      case DATE_YCOL:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c-=2;
            break;
          case KEYRIGHT:
            cur_pos_c++;
            break;
        }
        break;
      case DATE_YCOL + 1:
      case DATE_YCOL + 2:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c--;
            break;
          case KEYRIGHT:
            cur_pos_c++;
            break;
        }
        break;
      */
      case DATE_YCOL + 3:
        switch (k)
        {
          case KEYLEFT:
            cur_pos_c = DATE_MCOL + 1;
            break;
          case KEYRIGHT:
            cur_pos_c = TIME_HCOL;
            cur_pos_r = 0;
            break;
        }
        break;
      }

  }
  lcd.setCursor(cur_pos_c,cur_pos_r);
}

void set_value(byte k)
{
  if(cur_pos_r == 0)
  {
    switch (cur_pos_c)
    {
      case TIME_HCOL:
        switch (k)
        {
          case KEYUP:
            t.hour+=10;
            if(t.hour > 23) t.hour = 0;
            break;
          case KEYDOWN:
            t.hour-=10;
            if(t.hour > 23) t.hour = 23;
            break;
        }
        break;
      case TIME_HCOL + 1:
        switch (k)
        {
          case KEYUP:
            t.hour++;
            if(t.hour > 23) t.hour = 0;
            break;
          case KEYDOWN:
            t.hour--;
            if(t.hour > 23) t.hour = 23;
            break;
        }
        break;
      case TIME_MCOL:
        switch (k)
        {
          case KEYUP:
            t.min+=10;
            if(t.min > 59) t.min = 0;
            break;
          case KEYDOWN:
            t.min-=10;
            if(t.min > 59) t.min = 59;
            break;
        }
        break;
      case TIME_MCOL + 1:
        switch (k)
        {
          case KEYUP:
            t.min++;
            if(t.min > 59) t.min = 0;
            break;
          case KEYDOWN:
            t.min--;
            if(t.min > 59) t.min = 59;
            break;
        }
        break;
      }
  }
  else
  {
    switch (cur_pos_c)
    {
      case DATE_DCOL:
        switch (k)
        {
          case KEYUP:
            t.date+=10;
            if(t.date > 31) t.date = 1;
            break;
          case KEYDOWN:
            t.date-=10;
            if(t.date > 31) t.date = 31;
            break;
        }
        break;
      case DATE_DCOL + 1:
        switch (k)
        {
          case KEYUP:
            t.date++;
            if(t.date > 31) t.date = 1;
            break;
          case KEYDOWN:
            t.date--;
            if(t.date > 31) t.date = 31;
            break;
        }
        break;
      /*
      case DATE_MCOL:
        switch (k)
        {
          case KEYUP:
            t.mon+=10;
            if(t.mon > 12) t.mon = 1;
            break;
          case KEYDOWN:
            t.mon-=10;
            if(t.mon > 12 || t.mon < 1) t.mon = 12;
            break;
        }
        break;
      */
      case DATE_MCOL + 1:
        switch (k)
        {
          case KEYUP:
            t.mon++;
            if(t.mon > 12) t.mon = 1;
            break;
          case KEYDOWN:
            t.mon--;
            if(t.mon > 12 || t.mon < 1) t.mon = 12;
            break;
        }
        break;
      /*
      case DATE_YCOL:
        switch (k)
        {
          case KEYUP:
            t.year+=1000;
            if(t.year > 3000) t.year = 2000;
            break;
          case KEYDOWN:
            t.year-=1000;
            if(t.year > 3000) t.year = 2999;
            break;
        }
        break;
      case DATE_YCOL + 1:
        switch (k)
        {
          case KEYUP:
            t.year+=100;
            if(t.year > 2999) t.year-=900;
            break;
          case KEYDOWN:
            t.year-=100;
            if(t.year > 2099) t.year+=900;
            break;
        }
        break;
      case DATE_YCOL + 2:
        switch (k)
        {
          case KEYUP:
            t.year+=10;
            if(t.year > 2909) t.year = 2000;
            break;
          case KEYDOWN:
            t.year-=10;
            if(t.year > 3000) t.year = 2999;
            break;
        }
        break;
      */
      case DATE_YCOL + 3:
        switch (k)
        {
          case KEYUP:
            t.year++;
            if(t.year > 3000) t.year = 2000;
            break;
          case KEYDOWN:
            t.year--;
            if(t.year > 3000) t.year = 2999;
            break;
        }
        break;
      }
  }
  delay(100);
  print_set_time(0,0);
  print_set_date(0,1);
  lcd.setCursor(cur_pos_c,cur_pos_r);
  lcd.blink();
}

void print_set_time(byte c, byte r)
{
  lcd.setCursor(c,r);
  lcd.print("Cas:");
  lcd.setCursor(c+TIME_HCOL,r);
  if(t.hour < 10) lcd.print('0');
  lcd.print(t.hour);
  lcd.print(':');
  lcd.setCursor(c+TIME_MCOL,r);
  if(t.min < 10) lcd.print('0');
  lcd.print(t.min);
  lcd.print(':');
  lcd.setCursor(c+TIME_SCOL,r);
  lcd.print("00");
}

void print_set_date(byte c, byte r)
{
  lcd.setCursor(c,r);
  lcd.print("Datum:");
  lcd.setCursor(c+DATE_DCOL,r);
  if(t.date < 10) lcd.print('0');
  lcd.print(t.date);
  lcd.print('/');
  lcd.setCursor(c+DATE_MCOL,r);
  if(t.mon < 10) lcd.print('0');
  lcd.print(t.mon);
  lcd.print('/');
  lcd.setCursor(c+DATE_YCOL,r);
  if(t.year < 2000) t.year = 2000;
  lcd.print(t.year);
}


// check if must heating
void calc_heating()
{
    int ct = getTimeHHMM();               // current time HHMM
    for(byte c = 0; c < SENSORS; c++)          // loop for all chanes
    {
      byte p = sens_prg[c];                    // program for channel
      byte s = getProgStep(c);

      if (prg_temp[p][s] > SensorT25::getTemperature(c) && sens_active[c] && SensorT25::isValid(c) && ! isSensorDelay(c)) // prog temperature is higher and sensor is active
      {
        sens_heating[c] = true;
      }
      else
      {
        sens_heating[c] = false;
      }

      //antifreez - under 5°C
      if(SensorT25::getTemperature(c) < 5 && SensorT25::isValid(c)) sens_heating[c] = true;

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
    digitalWrite(RELAY_PIN, ! heating);
  }
}

//print main screen
void print_main_screen()
{
  for(int c = 0; c < SENSORS; c++)
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
    print_chanel(c);


    if(SensorT25::isValid(c))
    {
      print_temperature(c,lcdcol+POS_TEMP,lcdrow);
    }

    print_state(c,lcdcol+POS_STATE,lcdrow);
  }
  print_time(T_MAIN_C, T_MAIN_R);

  print_heating(H_MAIN_C,H_MAIN_R);
}

//sensor state and seting
void sensor_set(byte c)
{
  lcd.clear();
  boolean goloop = true;
  print_sensor_state(c);
  //delay(100);
  long etime = millis();
  while (goloop && ((millis() - etime) < EXIT_TIME))
  {
    //if (keypad.isKey())
    //{
      key = keypad.getKey();
      //keypad.buttonRelease();
      switch (key)
      {
        case KEYLEFT:                        //change program
        case KEYRIGHT:
          sensor_prog_change(c,key);
          lcd.clear();
          print_sensor_state(c);
          break;
        case KEYSET:                        // activate sensor
          //sens_active[c] = (sens_active[c]) ? false : true;
          sensor_activate(c);
          eeprom_save_active(c);
          print_sensor_state(c);
          break;
        //case KEYUP:                        // print program
          //sens_heating[c] = sens_heating[c] ? false : true;
          //print_program(c);
          //break;
        case KEYDOWN:                     //back
          goloop = false;
          break;
      }
      if(key > 0) etime = millis();
    //}
    if(lcd_refresh()) print_sensor_state(c);
  }
  lcd.clear();
}

// get if is delay for sensor start
boolean isSensorDelay(byte c)
{
  t = rtc.getTime();
  long d = (sens_delay[c][1]*10000)+(sens_delay[c][0]*100)+sens_delay[c][2];
  long z = (t.mon*10000L)+(t.date*100L)+t.hour;
  return (d > z) ? true :false;
}

void sensor_activate(byte c)
{
  lcd.clear();
  print_sensor_activate(c);
  boolean goloop = true;
  delay(1000);
  long etime = millis();
  while (goloop && ((millis() - etime) < EXIT_TIME))
  {
    //if (keypad.isKey())
    //{
      key = keypad.getKey();
      //keypad.buttonRelease();
      switch (key)
      {
        case KEYLEFT:            // date delay
        case KEYRIGHT:
          //if(sens_active[c])
          //{
            set_date_delay(c,key);
            print_sensor_activate(c);
            //delay(100);
          //}
          break;
        case KEYSET:               //activate-deactivate
          sens_active[c] = (sens_active[c]) ? false : true;
          //delay(1000);
          /*
          if(sens_active[c])
          {
            t = rtc.getTime();
            sens_delay[c][0] = t.date;
            sens_delay[c][1] = t.mon;
            sens_delay[c][2] = t.hour;
          }
          else
          {
            sens_delay[c][0] = 0;
            sens_delay[c][1] = 0;
            sens_delay[c][2] = 0;
          }
          */
          print_sensor_activate(c);
          break;
        case KEYUP:              //hour delay
          //if(sens_active[c])
          //{
            set_time_delay(c);
            print_sensor_activate(c);
            //delay(100);
          //}
          break;
        case KEYDOWN:
          goloop = false;
          break;
      }
      if(key > 0) etime = millis();
    //}
    if(lcd_refresh()) print_sensor_activate(c);
  }
  lcd.clear();
  //if(sens_active[c]) eeprom_save_delay(c);
  eeprom_save_delay(c);
}

void set_date_delay(byte c, byte k)
{
  t = rtc.getTime();
  if((sens_delay[c][0] < 1 || sens_delay[c][0] > 31) || (sens_delay[c][1] < 1 || sens_delay[c][1] > 12))
  {
     sens_delay[c][0] = t.date;
     sens_delay[c][1] = t.mon;
  }
  /*
  if(((sens_delay[c][1]*100) + sens_delay[c][0]) < ((t.mon * 100) + t.date))
  {
    sens_delay[c][0] = t.date;
    sens_delay[c][1] = t.mon;
  }
  */

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

void set_time_delay(byte c)
{
  t = rtc.getTime();
  if(sens_delay[c][2] > 23) sens_delay[c][2] = t.hour;
  sens_delay[c][2]++;
  if(sens_delay[c][2] > 23) sens_delay[c][2] = 1;
}

//get time in format HHMM as integer
int getTimeHHMM()
{
  t = rtc.getTime();
  return t.hour*100+t.min;
}

//print current program step for sensor
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

byte getProgStep(byte c)
{
  byte p = sens_prg[c];
  int ct = getTimeHHMM();
  byte i;
  for(i = 0; i < DAY_STEP; i++)
  {
    if (prg_time[p][i] > ct || prg_time[p][i] == 0) // check that prog step time is larger then current time or not set
    {
      break;
    }
  }
  if(i == 0)
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

byte getNextProgStep(byte c)
{
  byte p = sens_prg[c];
  byte i = getProgStep(c);
  i++;
  if(prg_time[p][i] == 0 || i >= DAY_STEP) i = 0;  // set i to day start if step value is 0 or over flow
  return i;
}



void print_program(byte p)
{
  lcd.clear();
  for(byte s = 0;s < DAY_STEP;s++)
  {
    lcd.setCursor(s*PROG_SPACE, 0);
    //lcd.print(prgInt2Time(prg_time[p][s]));
    lcd.print(prg_time[p][s]);
    lcd.print(' ');
    lcd.setCursor(s*PROG_SPACE, 1);
    lcd.print(prg_temp[p][s]);
  }
}


String prgInt2Time(int t)
{
  String h = String(t/100);
  String m = String(t%100);
  //if(h.length() < 2) h = ' '+h;
  if(m.length() < 2) m = '0'+m;
  return h+':'+m;
}


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
  /*
  lcd.setCursor(0,1);
  long d = (sens_delay[c][1]*10000)+(sens_delay[c][0]*100)+sens_delay[c][2];
  long z = (t.mon*10000L)+(t.date*100L)+t.hour;
  lcd.print(d);
  lcd.print(' ');
  lcd.print(z);
  */
}

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

void print_sensor_activate(byte c)
{
  print_sensor_state_R0(c);
  lcd.setCursor(0,1);
  print_sensor_delay(c);
}

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
  lcd.setCursor(col, row);
  lcd.print((isSensorDelay(c)) ? 'z' : getSensorProg(c));
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
  lcd.print(rtc.getTimeStr());
}
