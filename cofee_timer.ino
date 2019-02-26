#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <TimerOne.h>
#include <Bounce2.h>
#include <avr/pgmspace.h>
#include "TM1637.h"
#define ON 1
#define OFF 0
#define SETTINGS_ADDR 0

struct config_c
{
  unsigned char time_prebrew = 20;
  unsigned char time_break = 50;
  unsigned char time_brew = 250;
} settings;

boolean _flag_settings_saved = false;
boolean _flag_left_inactive_menu = false;
int8_t TimeDisp[] = {0x00,0x00,0x00,0x00};
unsigned char ClockPoint = 1;
unsigned char Update = OFF;
unsigned int eepromaddr;
boolean Flag_ReadTime;
boolean _flag_brewing_status = false;
boolean _flag_brewing_paused = false;
boolean _flag_brewing_beeping = false;

long minute = 60000;
boolean timePassed = false;
unsigned char second;
unsigned char _second;
unsigned char _stage;
unsigned char brew_time_10, prebrew_time_10, break_time_10;
int menu_brew_time_10, menu_prebrew_time_10, menu_break_time_10;
unsigned char dsec;

int bv1, bv2, bv3;
unsigned long bv3_first, bv3_previous = HIGH;
unsigned long in_menu_last_pressed;
boolean _flag_inside_menu = false;
boolean inside_second_menu = false;
int beep_count;   // How long the button was held (secs)
int menu_level = 0;
int menu_digit = 0;
unsigned long previousMillis = 0;

#define B1 2
#define B2 3
#define B3 4
#define BUZ 8
#define PUMP 7

#define CLK 6//pins definitions for TM1637 and can be changed to other ports        
#define DIO 5
TM1637 tm1637(CLK,DIO);

Bounce b3_debouncer = Bounce(); 

void setup()
{
  Serial.begin(9600);
  
  tm1637.set(BRIGHT_TYPICAL);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  tm1637.init();

//  BUTTONS
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT_PULLUP);
  
  digitalWrite(B1, HIGH);
  digitalWrite(B2, HIGH);
  digitalWrite(B3, HIGH);

  pinMode(BUZ, OUTPUT);
  digitalWrite(BUZ, LOW);
  pinMode(PUMP, OUTPUT);
  digitalWrite(PUMP, LOW);

  b3_debouncer.attach(B3);
  b3_debouncer.interval(100); 

// INTERRUPTS
  attachInterrupt(digitalPinToInterrupt( B1 ), runBrewingISR, FALLING);
  attachInterrupt(digitalPinToInterrupt( B2 ), breakBrewingISR, FALLING);

// buzzer BIP
  digitalWrite(BUZ, HIGH);
  delay(500);
  digitalWrite(BUZ, LOW);

  readSettings();
  displayCurrentSettings();
  
  
  
// default display
  tm1637.point(POINT_ON);
  TimeDisp[0] = 15;
  TimeDisp[1] = 16;
  TimeDisp[2] = 17;
  TimeDisp[3] = 18;
  tm1637.display(TimeDisp);
}
void loop()
{

//  button values
//  bv1 = digitalRead(B1);
//  bv2 = digitalRead(B2);
  

 

  if( _flag_inside_menu == true ) {
    
    tm1637.point(POINT_OFF);
    b3_debouncer.update();
    
    if( b3_debouncer.fell() ) {
      menu_digit++;
      if( menu_digit == 2)
      {
        menu_digit = 0;
        menu_level ++;
      }
      if(menu_level > 2) {
        menu_level = 0;
      }
      in_menu_last_pressed = millis();
    }
    

    if(millis() - in_menu_last_pressed > 10000) {
      // leaving menu after 10 seconds of inactivity
      
      _flag_inside_menu = false;
      _flag_left_inactive_menu = true;
    }
  
    switch(menu_level) {
      case 0:
        prepareTimeDisp(menu_prebrew_time_10, menu_level, menu_digit);
      break;
      case 1:
        prepareTimeDisp(menu_break_time_10, menu_level, menu_digit);
      break;
      case 2:
        prepareTimeDisp(menu_brew_time_10, menu_level, menu_digit);
      break;
      default: break;
    }
    Serial.println("IN TD");
    tm1637.display(TimeDisp);
  } else {
    bv3 = digitalRead(B3);

    if(_flag_inside_menu == false && _flag_brewing_status == false) {
      checkTimePassed();
    }
    
    
    // RUN MENU
    if (bv3 == LOW && bv3_previous == HIGH && millis()- bv3_first > 200){
      bv3_first = millis();    // if the buttons becomes press remember the time 
    }
  
    if (bv3 == LOW && ((millis() - bv3_first) % 1000) < 20 && millis() - bv3_first > 500){
      beep_count++;
      if(beep_count < 5) {
        ledblink(1, 50, BUZ);
      }
      
    }
    if (bv3 == HIGH && bv3_previous == LOW && beep_count >= 3 && beep_count < 4){
      delay(500);
      ledblink(3,100,BUZ);
      in_menu_last_pressed = millis();
      tm1637.clearDisplay();

      // set temporary menu values
      menu_prebrew_time_10 = prebrew_time_10;
      menu_break_time_10 = break_time_10;
      menu_brew_time_10 = brew_time_10;

      _flag_inside_menu = true;
    }
    if (bv3 == HIGH && bv3_previous == LOW && beep_count >= 4 && beep_count < 5){
      delay(500);
      ledblink(4,100,BUZ);
      in_menu_last_pressed = millis();
      tm1637.clearDisplay();
      
      inside_second_menu = true;
    }
  
    if (bv3 == HIGH){ // reset the counter if the button is not pressed
      beep_count = 0;
    }
  
    bv3_previous = bv3;
  // RUN MENU END
  }
 



  if( _flag_brewing_beeping == true )
  {
    finishBeep();
  }
  if(Update == ON && _flag_brewing_status == true)
  {
    TimeUpdate();
    tm1637.display(TimeDisp);
  }

  if( _flag_settings_saved == true ) {
    menu_level = 0;
    menu_digit = 0;
    displayCurrentSettings();
    saveSettings();
    tm1637.clearDisplay();
    TimeDisp[0] = 20; TimeDisp[1] = 10; TimeDisp[2] = 21; TimeDisp[3] = 14;
    tm1637.display(TimeDisp);
    ledblink(1, 1000, BUZ);
    tm1637.clearDisplay();
    
    _flag_settings_saved = false;
  }

  if( _flag_left_inactive_menu == true ) {
    Serial.println("Leaving menu after inactivity");
    menu_level = 0;
    menu_digit = 0;
    ledblink(5, 50, BUZ);
    tm1637.point(OFF);
    tm1637.clearDisplay();
  
    
    _flag_left_inactive_menu = false;
  }
}



void checkTimePassed(void) {
  unsigned long current = millis();
  
  if( ( current > (minute * 10L) ) && ( current < ((minute * 10L) + 100) ) ) {
    // inform me after 10 minutes passed
    ledblink(1, 1000, BUZ);
  }
  if( ( current > (minute * 20L) ) && ( current < ((minute * 20L) + 100) ) ) {
    // inform me after 20 minutes passed
    ledblink(2, 2000, BUZ);
  }
  if( ( current > (minute * 30L) ) && ( current < ((minute * 30L) + 100) ) ) {
    // inform me after 30 minutes passed
    ledblink(3, 3000, BUZ);
  }
  if( ( current > (minute * 45L) ) && ( current < ((minute * 45L) + 1000) ) ) {
    // inform me after 45 minutes passed - constant sign
    digitalWrite(BUZ, HIGH);
    timePassed = true;
  }
}

void prepareTimeDisp(int TD, int8_t menu_level, int menu_digit) {
  unsigned long currentMillis = millis();
  if( currentMillis - previousMillis >= 200 ) {
    previousMillis = currentMillis;
    
    if(TimeDisp[menu_digit] == 19) {
        TimeDisp[0] = TD / 100;
        TimeDisp[1] = (TD / 10) % 10;
    } else {
      TimeDisp[menu_digit] = 19;
    }
  }
  
  TimeDisp[2] = 19;
  TimeDisp[3] = (16 + menu_level);
}


void ledblink(int times, int lengthms, int pinnum){
  for (int x=0; x<times;x++){
    digitalWrite(pinnum, HIGH);
    delay (lengthms);
    digitalWrite(pinnum, LOW);
    delay(lengthms);
  }
}



void runBrewingISR()
{
  
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if( interrupt_time - last_interrupt_time > 200 )
  {
    if(_flag_inside_menu == true) {
      in_menu_last_pressed = millis();
      switch(menu_level) {
        case 0:
          if( menu_digit == 0 ) {
            if(menu_prebrew_time_10 / 100 == 9) menu_prebrew_time_10 -= 900;
            else menu_prebrew_time_10 += 100;
          } else {
            if( (menu_prebrew_time_10 / 10) % 10 == 9 ) menu_prebrew_time_10 -= 90;
            else menu_prebrew_time_10 += 10;
          }
        break;
        case 1:
          if( menu_digit == 0 ) {
            if(menu_break_time_10 / 100 == 9) menu_break_time_10 -= 900;
            else menu_break_time_10 += 100;
          } else {
            if( (menu_break_time_10 / 10) % 10 == 9 ) menu_break_time_10 -= 90;
            else menu_break_time_10 += 10;
          }
        break;
        case 2:
          if( menu_digit == 0 ) {
            if(menu_brew_time_10 / 100 == 9) menu_brew_time_10 -= 900;
            else menu_brew_time_10 += 100;
          } else {
            if( (menu_brew_time_10 / 10) % 10 == 9 ) menu_brew_time_10 -= 90;
            else menu_brew_time_10 += 10;
          }
        break;
        default: break;
      }
      
    } else if(timePassed == true) {
      timePassed = false;
      digitalWrite(BUZ, LOW);
      Serial.println("Stop buzzer");
      
    } else {
      if( _flag_brewing_status == false ) {
        Serial.println("Start brewing");
        _flag_brewing_status = true;
        swStart();
      } else {
        if( _flag_brewing_paused == true )
        {
          swResume();
          _flag_brewing_paused = false;
        }
        else
        {
          swPause();
          _flag_brewing_paused = true;
        }
      }
    }
    
  }
  last_interrupt_time = interrupt_time;
}

void breakBrewingISR()
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if( interrupt_time - last_interrupt_time > 500 )
  {
    if( _flag_brewing_status == true )
    {
      
      swStop();
    }
    if( _flag_inside_menu == true )
    {
      _flag_settings_saved = true;
      _flag_inside_menu = false;
    }
  }
  last_interrupt_time = interrupt_time;
}


void swStart()
{
  dsec = 0;
  tm1637.clearDisplay();
  Timer1.initialize(100000);//timing for 100ms
  Timer1.attachInterrupt(TimingDown);//declare the interrupt serve routine:TimingISR
}



void TimingDown()
{
  Update = ON;
  if( dsec % 5 == 0 )
  {
    ClockPoint = (~ClockPoint) & 0x01;
  }
  
  dsec ++;
  
  if( dsec == 10) {
    dsec = 0;
  }
  

  if( prebrew_time_10 > 0)
  {
    second = prebrew_time_10 --;
    
    if( dsec % 5 != 0 ) _stage = 16;
    else _stage = 19;
    
    digitalWrite(PUMP, HIGH);
  }
  else
  {
    second = break_time_10;
    if( break_time_10 > 0)
    {
      second = break_time_10 --;
      if( dsec % 5 != 0 ) _stage = 17;
      else _stage = 19;
      digitalWrite(PUMP, LOW);
    }
    else
    {
      second = brew_time_10;
      if( brew_time_10 > 0)
      {
        brew_time_10 --;
        if( dsec % 5 != 0 ) _stage = 18;
        else _stage = 19;
        digitalWrite(PUMP, HIGH);
      }
      else
      {
        
        digitalWrite(PUMP, LOW);
        swStop();
      }
    }
  }
  // assign value to pass to display in TimeUpdate
  _second = second;

  
}


void TimeUpdate(void)
{
  if(ClockPoint) tm1637.point(POINT_OFF);//POINT_ON = 1,POINT_OFF = 0;
  else tm1637.point(POINT_ON); 
  
  TimeDisp[0] = _second / 100;
  TimeDisp[1] = (_second / 10) % 10;
  TimeDisp[2] = _second % 10;
  TimeDisp[3] = _stage;
  
  Update = OFF;
}
void swResume()
{
  Flag_ReadTime = 0;
  Timer1.resume();
}

void swPause()//timer1 off if [CS12 CS11 CS10] is [0 0 0].
{
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
}
void swStop()
{
  Timer1.stop();
  Timer1.restart();
  Timer1.detachInterrupt();
  
  tm1637.clearDisplay();
  tm1637.point(OFF);
  readSettings();
  
  _flag_brewing_status = false;
  dsec = 0;
  
  _flag_brewing_beeping = true;
  
//  Update = ON;
}

void readSettings(void) {
  EEPROM_readAnything(SETTINGS_ADDR, settings);
  brew_time_10 = settings.time_brew * 10;
  break_time_10 = settings.time_break * 10;
  prebrew_time_10 = settings.time_prebrew * 10;
  Serial.println();
  Serial.println("Settings read");
  Serial.println(prebrew_time_10);
  Serial.println(break_time_10);
  Serial.println(brew_time_10);
  Serial.println();
}

void saveSettings(void) {
  
  settings.time_prebrew = menu_prebrew_time_10 / 10;
  settings.time_break = menu_break_time_10 / 10;
  settings.time_brew = menu_brew_time_10 / 10;
  EEPROM_writeAnything(SETTINGS_ADDR, settings);
  Serial.println();
  Serial.println("Settings saved");
  Serial.println(settings.time_prebrew);
  Serial.println(settings.time_break);
  Serial.println(settings.time_brew);
  Serial.println();
  readSettings();
}

void displayCurrentSettings(void) {
  Serial.println();
  Serial.println("displayCurrentSettings");
  Serial.println(prebrew_time_10);
  Serial.println(break_time_10);
  Serial.println(brew_time_10);
  Serial.println();
}
void finishBeep()
{
  digitalWrite(PUMP, LOW);
  digitalWrite(BUZ, HIGH);
  TimeDisp[0] = 18; TimeDisp[1] = 18; TimeDisp[2] = 18; TimeDisp[3] = 18;
  tm1637.display(TimeDisp);
  delay(1000);
  
  digitalWrite(BUZ, LOW);
  tm1637.clearDisplay();
  delay(500);
  
  digitalWrite(BUZ, HIGH);
  TimeDisp[0] = 18; TimeDisp[1] = 18; TimeDisp[2] = 18; TimeDisp[3] = 18;
  tm1637.display(TimeDisp);
  delay(1000);
  
  digitalWrite(BUZ, LOW);
  tm1637.clearDisplay();
  delay(500);
  digitalWrite(BUZ, HIGH);
  TimeDisp[0] = 18; TimeDisp[1] = 18; TimeDisp[2] = 18; TimeDisp[3] = 18;
  tm1637.display(TimeDisp);
  delay(1000);
  
  digitalWrite(BUZ, LOW);
  tm1637.clearDisplay();
  _flag_brewing_beeping = false;
}
