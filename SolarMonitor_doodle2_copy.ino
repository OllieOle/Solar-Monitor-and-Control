/*
  File:        /Users/XxxxxYyyyy/Library/Mobile Documents/com~apple~CloudDocs/Solar Power/SolarMonitor_doodle2
  Name:             'SolarMonitor_doodle2'
  Created:          November 15, 2023 
                    from SolarMonitor Version November 12, 2023 at 09:03
  LastRun:          
  WIP:        run to TEST
              calc with int PV_Voltage  ????
              update description
*/

String Modified = "Version Apr19 2024 12:42";  //$09
/*   
Find "lcd.print("V . . ." and change Version  ~line 164
     
Description:

The solar battery allows for the inverter to operate in an 
AUTO hydro saving mode when there is sufficient solar insolation
or when time ON and time OFF are set.
It is desirable to maintain 80% or more of battery capacity 
in case of utiliy outage.. 

Setup the AUTO / MANUAL modes by first switching the inverter 
receptacle A/C power switch to OFF. Select AUTO or MANUAL by 
pressing the button switch. 
AUTO: LED turns ON and SSR turns OFF (CHECK). 
MANUAL: LED turns OFF and SSR turns ON (CHECK).

The solar battery also provides household power when 
utility hydro power fails.
This feature is available in either the AUTO or MANUAL mode. 
When utility hydro fails, the monitor sounds an audible alarm.

For utility hydro saving, in AUTO mode, the monitor switches the 
solar battery ON and OFF when available PV panel power 
is adequate (i.e. when 10 minute PV panel voltage 
averages above 20 volts [setable]). 
Optionally, the hydro saving mode may be entered by 
selecting ON and OFF times.[tbd]

The battery is kept above 80% charged by the monitor, 
automatically switching to MANUAL; the inverter recharges 
the battery for one hour [setable] 
when batt voltage reaches a set minimum (e.g. 12.0 volts [setable] 
under load).

When connected to a computer, the function of the monitor is output, 
including date and time real time clock (RTC)output.
*/

/* 
  FUNCTIONS:
      void setup() {                      $01
      void loop()                         $02
      set pin assignments                 $03
      set constants                       $04
      set the RTC current time            $05
      void setDS3231time( ... )           $05
      void CheckSchedule( ... )           $06 delete
      check battery charge state          $07
      bool buttonPressed(void)            $08
      set 'Last Modified'                 $09
.     set RTC On and OFF times            $10
      void readDS3231time( ... )          $11
      void LCD_Display()                  $12
      void displayTime()                  $13
      byte decToBcd(byte val)             $14
      byte decToBcd(byte val)             $15
      check PV ON times                   $16
      void measure_hydro(int analog_pin)  $17
      void SSR_Control()                  $18
      void Tone( ... )                    $19
      void measure_volts()                $21

  // Real Time Clock (RTC) connection:
  //    +       : 3.3V-5V
  //    D (SDA) : A4
  //    C (SCL) : A5
  //    NC      : Connect to GND
  //    -       : GND

  // Pushbutton connection:
      one side to arduino input pin (INPUT_PULLUP) and other side to ground.
*/

#include "Wire.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
#define DS3231_I2C_ADDRESS 0x68
long int Schedule[2][4] = { { 9, 0, 5, 0 } };
int SolPwr_is_ON = 1;
unsigned long int deltaPressedMillis = 0;
static unsigned long lastMillis = 0;
int mode = 0;
int modeStatus = 0;
bool newPress = false;
bool longPress, shortPress;
int pressNum = 0;
const byte pb_Pin = 2;  //$03 pb pin; tggl AUTO/MANUAL;btn OPEN -> pin HIGH LED
int buttonState = 0;
int lastButtonState = 0;
const byte SSR_Pin = 3;            //driver to SSR input
const byte LED_Pin = 4;            //driver to LED
const byte Sound_pin = 6;          //piezo driver (via 2N3904)
const byte utility_hydro_pin = 7;  //sense utility outage; resistor ladder tap
unsigned long int hydro_alarm_time;
unsigned long int hydro_alarm_repeat_time = 2000;
//int value = 0;
unsigned long tmillis = 0;
//unsigned long dtmillis = 60000;//TEST
unsigned long dtmillis = 500;  //$04time between data printouts
float battVoltage, PV_Voltage;
//int battVoltage;
int battVoltData, PV_VoltData;
float battSensorCal = 12.49 / 3.89;  //$04SET THIS (3.31)
//float battSensorCal = 13.3 / 4.19;     //$04SET THIS (3.17)
float PV_SensorCal = 26.33 * 84 / 82;  //$04SET THIS
//to bypass low batt turn off, enter 0.  'batt_Low_Set'
float batt_Low_Set = 12.0;  //$04SET THIS
float Low_Batt_Rcovr = 12.2;
bool ONchargeFlag1;
bool ONchargeFlag2 = 0;
unsigned long int T_charge = 36000000;  //$04 battery recharge time
unsigned long int T_Start_charge;
//to bypass PV turn ON/OFF, enter 200. as 'PV_Low_Set'
float PV_Low_Set = 20.0;  //$04SET THIS
float PV_Voltage_Avg3, PV_Voltage_Avg10;
bool PV_Volts_OK = false;
bool Batt_Volts_OK = true;
bool Time_of_Day_OK = false;
bool ToD_ON = true;
int InverterOnUtlPwr = 0;  // 0=>off hydro;  1=>on hydro
//to bypass time of day turn on, enter 25. as PV_ON_Time, -1. as PV_OFF_Time
float PV_ON_Time = 10. + 0 / 60.;    //$04SET THIS
float PV_OFF_Time = 23. + 45 / 60.;  //$04SET THIS
float PV_Time;
float PV_arr[61] = {};
int Solar_ON;
const char* OFF_ON[] = { "OFF", "ON" };
//const char* Mode[] = { "MANU", "MANU", "AUTO" };
const char* Mode[] = { "MANU Solar OFF", "MANU Solar ON", "AUTO" };
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
byte second_last = 99;
int v_printFlag1 = 0;  //volt data print
int v_printFlag2 = 0;
int v_printFlag3 = 0;
int u_printFlag1 = 0;  //utility data print

int UtlPwr_is_ON = 0;
int hydroON_changed = 0;

void setup() {  //$01
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\n\n\n/Users/jamesrobar/Documents/Arduino/Solar Power/SolarMonitor_doodle2");

  // ----- LCD display***
  lcd.begin(20, 4);
  lcd.backlight();  //Turn the LCD display on
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SolarMonitor_doodle2");
  lcd.setCursor(0, 1);
  lcd.print("V Apr13 2024 09:03");
  Serial.println(Modified);  //$09
  pinMode(2, INPUT_PULLUP);  //$03tggl pb; AUTO/MANUAL; button OPEN pin HIGH;
  pinMode(3, OUTPUT);        //driver to SSR input
  pinMode(4, OUTPUT);        //driver to LED_Pin
  pinMode(6, OUTPUT);        //piezo driver (via 2N3904)
  pinMode(7, INPUT);         //sense utility hydro outage; resistor ladder tap

  //>>>>>set the RTC current time here<<<<<// $05
  // DS3231 seconds, minutes, hours, day, date, month, year
  //UNCOMMENT then upload with correct time, then COMMENT OUT and upload again
  //setDS3231time(0,6,15,6,26,1,24);  //<------

  digitalWrite(utility_hydro_pin, 1);  //setup to measure utility hydro
  ONchargeFlag1 = 1;
  digitalWrite(LED_Pin, 1);  //RESETs to AUTO ON mode
  //>>>>>setup the PV voltage array
  PV_VoltData = analogRead(A1);
  PV_Voltage = PV_SensorCal * (5.02 * PV_VoltData / 1023);  //$04 "5.02" is Vcc
  for (int i = 0; i < 60; i++) {
    PV_arr[i] = PV_Voltage;  //initial values of 'PV_arr[]'
  }

  //>>>>>obtain 10 minute 'PV_Voltage' average every 10 seconds (60 samples)
  PV_Voltage_Avg10 = 0;
  for (int i = 0; i < 60; i++) {  //select last 18 (3 min) PV voltages
    PV_Voltage_Avg10 += PV_arr[i];
  }
  PV_Voltage_Avg10 = PV_Voltage_Avg10 / 60.;

  displayTime();
  Serial.println();
  Serial.print("Solar ON OFF Time: ");
  Serial.print(PV_ON_Time);
  Serial.print("  ");
  Serial.println(PV_OFF_Time);
  Serial.print("AUTO is ");
  //  Serial.println(OFF_ON[SolPwr_is_ON]);
  measure_utility_hydro();
  //Serial.print("Utility Hydro Available: ");
  //Serial.println(OFF_ON[UtlPwr_is_ON]);
  Serial.println("\n\n");
  delay(2000);
}

void loop() {  //$02
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  //>>>>>test for hydro outage
  measure_utility_hydro();  //sound alarm if OUT
  measure_volts();          //measure batt and PV voltage
  buttonPressed();
  SSR_Control();
}

//>>>>>push button check<<<<<//$08
void buttonPressed() {
  //controller states are: 'MANU Solar OFF', 'MANU Solar ON', 'AUTO'
  //'longPress' not used atm; poss use to set e.g. solar ON time of day

  buttonState = digitalRead(pb_Pin);
  //Serial.print("buttonState  ");  //xxx
  //Serial.println(buttonState);    //xxx
  if (HIGH == buttonState) {   //'HIGH' => btn rlsd
    if (true == shortPress) {  // 'true' btn changed fm not rlsd to rlsd
      pressNum += 1;
      if (pressNum >= 3) pressNum = 0;
      lcd.setCursor(11, 1);
      lcd.print("    ");
      lcd.setCursor(0, 1);
      lcd.print(Mode[pressNum]);
      //Serial.println(Mode[pressNum]);  //xxx
    }
    //setup for next btn press
    longPress = shortPress = false;
    deltaPressedMillis = 0;  //holds msecs button is pressed

  } else {  //buttonState is 'LOW' => btn not rlsd; chk if valid, short, long
    if (lastButtonState != buttonState) lastMillis = millis();
    deltaPressedMillis = deltaPressedMillis + (millis() - lastMillis);
    lastMillis = millis();
    if (deltaPressedMillis > 100) {  //check if valid button (i.e. not chatter)
      if (deltaPressedMillis > 2000) {
        longPress = true;
      } else {
        shortPress = true;
      }
    }
  }
  lastButtonState = buttonState;
  /*
  Serial.print("longPress ");     //XXX
  Serial.print(longPress);        //XXX
  Serial.print("\tshortPress ");  //XXX
  Serial.print(shortPress);       //XXX
  Serial.print("\tpressNum ");    //XXX
  Serial.print(pressNum);         //XXX
  //Serial.print("\tmode ");        //XXX
  //Serial.print(mode);             //XXX
  //Serial.print("\tmodeStatus ");  //XXX
  //Serial.print(modeStatus);       //XXX

  Serial.println();  //XXX
  */
}

void AverageOverTime(int period){
      //>>>>>obtain 60 'PV_Voltage' samples over 10 minutes => 10s sample interval
    //>>>>>sample every 10  seconds
    if ((second % period == 0) && (second != second_last)) {
      second_last = second;
      /*
      Serial.print("PV0 ");
      Serial.print("\tPV10 ");
      Serial.print(PV_Voltage_Avg10);
      Serial.print("\tSolarBatt ");
      Serial.print(OFF_ON[!InverterOnUtlPwr]);
      Serial.print("\t\t");
      displayTime();
      Serial.println();
      LCD_Display();  //update LCD every 10 seconds
      //>>>>>shift out oldest datum (@ posn 59)
*/

      for (int i = 59; i >= 1; i--) {
        PV_arr[i] = PV_arr[i - 1];
      }
      //>>>>>enter newest 'PV_Voltage' into array posn 0
      PV_arr[0] = PV_Voltage;
      /*//maybe need this tdt min batt volts
      //>>>>>obtain 3 minute 'PV_Voltage' average every 10 seconds (18 samples)
      PV_Voltage_Avg3 = 0;
      for (int i = 0; i < 18; i++) {  //select last 18 (3 min) PV voltages
        PV_Voltage_Avg3 += PV_arr[i];
      }
      PV_Voltage_Avg3 = PV_Voltage_Avg3 / 18.;
*/
      //>>>>>obtain 10 minute 'PV_Voltage' average every 10 seconds (60 samples)
      PV_Voltage_Avg10 = 0;
      for (int i = 0; i < 60; i++) {  //select last 18 (3 min) PV voltages
        PV_Voltage_Avg10 += PV_arr[i];
      }
      PV_Voltage_Avg10 = PV_Voltage_Avg10 / 60.;
    }
  }
  return;  //$04 comment to allow printout
           //
  float Tprint = hour + minute / 60.;
  Serial.print(Tprint);
  Serial.print(", ");
  Serial.print(battVoltage, 2);
  Serial.print(", ");
  Serial.print(PV_Voltage, 2);
  Serial.print(", ");
  Serial.print(PV_Voltage_Avg10, 2);
  Serial.print(", Solar ");
  Serial.print(OFF_ON[Solar_ON]);
}

}


//>>>>>measure battery & PV voltage<<<<<//$21
void measure_volts() {
  //>>>>>read battery
  battVoltData = analogRead(A0);
  battVoltage = battSensorCal * (4.96 * battVoltData / 1023);
  if (battVoltage < batt_Low_Set) {
    Batt_Volts_OK = false;
  } else {
    if ((Batt_Volts_OK == false) && (battVoltage >= Low_Batt_Rcovr)) {
      Batt_Volts_OK = true;
    }
  }
  //>>>>>read PV panel
  PV_VoltData = analogRead(A1);
  PV_Voltage = PV_SensorCal * (5.02 * PV_VoltData / 1023);
  //>>>>>set printout period
  if (millis() > tmillis) v_printFlag1 = 0;  //set condition to printout new data
  if (millis() > tmillis && v_printFlag1 == 0) {
    v_printFlag1 = 1;  //set condition to block addl printouts of old data
    tmillis = millis() + dtmillis;
    //>>>>>obtain 60 'PV_Voltage' samples over 10 minutes => 10s sample interval
    //>>>>>sample every 10  seconds
    if ((second % 10 == 0) && (second != second_last)) {
      second_last = second;
      ///*//
      Serial.print("PV0 ");
      Serial.print("\tPV10 ");
      Serial.print(PV_Voltage_Avg10);
      Serial.print("\tSolarBatt ");
      Serial.print(OFF_ON[!InverterOnUtlPwr]);
      Serial.print("\t\t");
      displayTime();
      Serial.println();
      LCD_Display();  //update LCD every 10 seconds
      //>>>>>shift out oldest datum (@ posn 59)
      for (int i = 59; i >= 1; i--) {
        PV_arr[i] = PV_arr[i - 1];
      }
      //>>>>>enter newest 'PV_Voltage' into array posn 0
      PV_arr[0] = PV_Voltage;
      /*//maybe need this tdt min batt volts
      //>>>>>obtain 3 minute 'PV_Voltage' average every 10 seconds (18 samples)
      PV_Voltage_Avg3 = 0;
      for (int i = 0; i < 18; i++) {  //select last 18 (3 min) PV voltages
        PV_Voltage_Avg3 += PV_arr[i];
      }
      PV_Voltage_Avg3 = PV_Voltage_Avg3 / 18.;
*/
      //>>>>>obtain 10 minute 'PV_Voltage' average every 10 seconds (60 samples)
      PV_Voltage_Avg10 = 0;
      for (int i = 0; i < 60; i++) {  //select last 18 (3 min) PV voltages
        PV_Voltage_Avg10 += PV_arr[i];
      }
      PV_Voltage_Avg10 = PV_Voltage_Avg10 / 60.;
    }
  }
  return;  //$04 comment to allow printout
           //
  float Tprint = hour + minute / 60.;
  Serial.print(Tprint);
  Serial.print(", ");
  Serial.print(battVoltage, 2);
  Serial.print(", ");
  Serial.print(PV_Voltage, 2);
  Serial.print(", ");
  Serial.print(PV_Voltage_Avg10, 2);
  Serial.print(", Solar ");
  Serial.print(OFF_ON[Solar_ON]);
}

//>>>>>hydro outage check<<<<<//$17
//>>>>>measure voltage from 12V wall bud to test for hydro outage
void measure_utility_hydro() {
  if (digitalRead(utility_hydro_pin) != UtlPwr_is_ON) {
    hydroON_changed = 1;
  } else {
    hydroON_changed = 0;
  }
  UtlPwr_is_ON = digitalRead(utility_hydro_pin);
  if (UtlPwr_is_ON == 1) {       //utility ON; I/P pin 7
    digitalWrite(Sound_pin, 0);  // O/P piezo driver pin 6;hydro ON
  } else {
    //Sound_pin, Freq, Duration, time Off, nmbr Repeats
    Tone(Sound_pin, 4000, 2500, 1000, 2);                   //Sound_pin = 6
    hydro_alarm_time = millis() + hydro_alarm_repeat_time;  //NOTNEEDEDNOT USED
  }

  if (hydroON_changed == 1) u_printFlag1 = 0;  //condition to print when new data
  if (hydroON_changed == 1 && u_printFlag1 == 0) {
    u_printFlag1 = 1;  //set condition to block addl printouts of old data
    Serial.print("Utility Hydro: ");
    Serial.println(OFF_ON[UtlPwr_is_ON]);
  }
}

//>>>>>control SSR<<<<<//$18
void SSR_Control() {
  //force solarBatt OFF when batt voltage is low for MANU and AUTO
  //digitalWrite(SSR_Pin, !Batt_Volts_OK);//uncommented to TEST
  //>>>>>test if in MANU mode


  Serial.print("pressNum  ");                    //xxx
  Serial.print(pressNum);                        //xxx
  Serial.print("    SSR  ");                     //xxx
  Serial.print(digitalRead(SSR_Pin));          //xxx
  Serial.print("    ");  //xxx
  Serial.println(Mode[pressNum]);                //xxx

  if (pressNum < 2) {
    //pressNum(SSR)== 0 Solar ON;pressNum(SSR) == 1 Solar OFF
    digitalWrite(SSR_Pin, (bool)pressNum);
    //delay(1000);                         //xxx
    return;  //xxx
  }
  //is in AUTO mode
  else {  //pressNum==2 -->is in AUTO mode
    PV_Volts_OK = (PV_Voltage_Avg10 >= PV_Low_Set);
    digitalWrite(SSR_Pin, !PV_Volts_OK);  //Solar ON if batt voltage OK
  }
  lcd.setCursor(5, 1);
  lcd.print("Solar ");
  lcd.setCursor(11, 1);
  lcd.print(OFF_ON[digitalRead(SSR_Pin)]);
  //lcd.setCursor(16, 1);
  //lcd.print("SSR");  //need this o/p?
  //lcd.setCursor(19, 1);
  //lcd.print(digitalRead(SSR_Pin));

  /*
  if (v_printFlag2 == 0) {
    Serial.print("Solar Battery: ");
    if (SolPwr_is_ON == 1) {
      Serial.println(OFF_ON[!InverterOnUtlPwr]);
    } else {
      Serial.println(" MANUAL utility switch");
    }
    v_printFlag2 = 1;
  }
*/

  //>>>>>>>>>>$16 check PV ON times
  if (ToD_ON) {  //if true, allow Time of Day control of solar batt
    PV_Time = hour + minute / 60.;
    Time_of_Day_OK = (PV_Time >= PV_ON_Time) && (PV_Time < PV_OFF_Time);
    digitalWrite(SSR_Pin, !Time_of_Day_OK);
  }
}

//>>>>>Generate Tone<<<<<//$19
//https://itp.nyu.edu/physcomp/labs/labs-arduino-digital-and-analog/tone-output-using-an-arduino/

void Tone(int Sound_pin, int Freq, int Duration, int Off, int Repeat) {
  //xxx
  for (int i = 0; i < Repeat; i++) {  // number of beeps
    //tone(Sound_pin, Freq, Duration);  //'Duration' of beep
    //delay(Off);  // 'Off' time between beeps
    digitalWrite(Sound_pin, HIGH);
    delay(2000);
    digitalWrite(Sound_pin, LOW);
    delay(2000);
  }
}

//>>>>>following functions are for RTC Real Time Clock<<<<<// $05*
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
  // sets time and date data to DS3231  $5
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);                     // set next input to start at the seconds register
  Wire.write(decToBcd(second));      // set seconds
  Wire.write(decToBcd(minute));      // set minutes
  Wire.write(decToBcd(hour));        // set hours
  Wire.write(decToBcd(dayOfWeek));   // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth));  // set date (1 to 31)
  Wire.write(decToBcd(month));       // set month
  Wire.write(decToBcd(year));        // set year (0 to 99)
  Wire.endTransmission();
}

void readDS3231time(byte* second,
                    byte* minute,
                    byte* hour,
                    byte* dayOfWeek,
                    byte* dayOfMonth,
                    byte* month,
                    byte* year) {  // addresses are passed to 'readDS3231time(address for 'second', for 'minute', ... $11
  // addresses are rec'd by 'readDS3231time(... )'
  // '*second' gets the value at the address sent to 'readDS3231time(... )' and pointed to by 'second' etc.
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);  // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);  // 0x7f--> 1111111 //get bcd from RTC then convert to decimal
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);  // 0x3f-->  111111
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val) {  //$14
  return ((val / 16 * 10) + (val % 16));
}

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val) {  //$15
  return ((val / 10 * 16) + (val % 10));
}

void displayTime() {  //$13
  // retrieve data from DS3231
  // send the address '&second' for 'second' to fcn 'readDS3231time(&second... )
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  Serial.print(dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(month, DEC);
  Serial.print("/");
  Serial.print(year, DEC);
  Serial.print(" day ");
  Serial.print(dayOfWeek);
  Serial.print("\t");
  Serial.print(hour, DEC);
  // convert the byte variable to a decimal number when displayed
  Serial.print(":");
  if (minute < 10) {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second < 10) {
    Serial.print("0");
  }
  Serial.print(second, DEC);
}

//+++++LCD_Display +++++// $12
//update every 10 seconds; called from 'measure_volts()'
void LCD_Display() {
  lcd.clear();
  //row 0
  lcd.setCursor(0, 0);
  lcd.print("B");
  lcd.setCursor(1, 0);
  lcd.print(batt_Low_Set, 1);
  lcd.setCursor(6, 0);
  lcd.print("P");  //<--
  lcd.setCursor(7, 0);
  lcd.print(PV_Low_Set, 0);
  lcd.setCursor(10, 0);
  lcd.print("T");
  lcd.setCursor(11, 0);
  lcd.print(PV_ON_Time, 1);
  lcd.setCursor(16, 0);
  lcd.print(PV_OFF_Time, 1);
  //row 1
  lcd.setCursor(0, 1);
  lcd.print(Mode[pressNum]);

  //lcd.setCursor(5, 1);//WIP adjust locn
  //lcd.print("Utl");//WIP
  //lcd.setCursor(8, 1);//WIP adjust locn
  //lcd.print(OFF_ON[UtlPwr_is_ON]);//WIP
  //lcd.setCursor(12, 1);
  //lcd.print(hour);
  //lcd.setCursor(14, 1);
  //lcd.print(":");
  //lcd.setCursor(15, 1);
  //lcd.print(minute);
  //row 2
  lcd.setCursor(0, 2);
  lcd.print("PV0");
  lcd.setCursor(4, 2);
  lcd.print(PV_Voltage, 0);
  lcd.setCursor(9, 2);
  lcd.print("PV10");
  lcd.setCursor(14, 2);
  lcd.print(PV_Voltage_Avg10, 0);
  //row 3
  lcd.setCursor(0, 3);
  lcd.print("Batt");
  lcd.setCursor(4, 3);
  lcd.print(OFF_ON[!InverterOnUtlPwr]);
  lcd.setCursor(9, 3);
  lcd.print("Volt");
  lcd.setCursor(14, 3);
  lcd.print(battVoltage, 2);
}
