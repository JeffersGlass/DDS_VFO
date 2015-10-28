#include <Encoder.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <si5351.h>


//-----------Variables & Declarations---------------

long currFreq = 1000000; 

//long steps[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000};
//const int MAX_STEP_INDEX = 15;
//char* stepNames[] = {"  1 hz", "  2 hz", "  5 hz", " 10 hz", " 20 hz", " 50 hz", "100 hz", "200 hz", "500 hz", " 1 Khz", " 2 Khz", " 5 Khz", "10 Khz", "20 Khz", "50 Khz", "100Khz"};

long steps[] = {1, 10, 100, 1000, 10000, 100000, 500000, 1000000, 5000000, 10000000};
const int MAX_STEP_INDEX = 9;
char* stepNames[] = {"  1 hz", "  10 hz", "100 hz", "  1Khz", " 10KHZ", "100Khz", "500Khz", "  1Mhz", "  5Mhz", " 10Mhz"};

int stepIndex = 0;

long lastButtonPress[] = {0,0,0,0,0};
boolean buttonActive[] = {false, false, false, false, false};

long encoderPosition = 0;
boolean displayNeedsUpdate;

const long MIN_FREQ = 8500;
const long MAX_FREQ = 150000000;

//---------LCD SETUP-------//
int PIN_RS = 7;
int PIN_EN = 8;
int PIN_DB4 = 9;
int PIN_DB5 = 10;
int PIN_DB6 = 11;
int PIN_DB7 = 12;
LiquidCrystal lcd(PIN_RS, PIN_EN, PIN_DB4, PIN_DB5, PIN_DB6, PIN_DB7);

//--------Si5351 Declaration---------------//

Si5351 si5351;
//SDA is on pin A4 for Arduino Uno
//SCL is on pin A5 for Arduino Uno

//--------Tuning Knob Interrupt Pins-------//
//Encoder knob(2, 3), pushbutton on 1

Encoder encoder(2, 3);
const int PIN_BUTTON_ENCODER = 1;

//Button Pins//
const int PIN_BUTTON_UP = 4;
const int PIN_BUTTON_DOWN = 5;
const int PIN_BUTTON_STEP = 6;
const int BUTTON_DEBOUNCE_TIME = 10; //milliseconds

void setup(){
  // inialize LCD, display welcome message
  lcd.begin(20, 4);
  delay(250);
  lcd.setCursor(4, 1);
  lcd.print("VFO STARTING");
  
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0);
  si5351.set_freq(currFreq * 100ULL, 0ULL, SI5351_CLK0);
  delay(750);
  si5351.set_freq(7000000 * 100ULL, 0ULL, SI5351_CLK1);
  delay(500);
  
    //knob.write(0);
  pinMode(PIN_BUTTON_ENCODER, INPUT);
  digitalWrite(PIN_BUTTON_ENCODER, HIGH);
  
  pinMode(PIN_BUTTON_UP, INPUT);
  digitalWrite(PIN_BUTTON_UP, HIGH);
  pinMode(PIN_BUTTON_DOWN, INPUT);
  digitalWrite(PIN_BUTTON_DOWN, HIGH);
  pinMode(PIN_BUTTON_STEP, INPUT);
  digitalWrite(PIN_BUTTON_STEP, HIGH);
 

  lcd.clear();
  lcd.setCursor(2, 7);
  lcd.print("WELCOME!");
  delay(500);
  displayInfo();
}

void loop(){
  
  if (displayNeedsUpdate) {displayInfo();}
  delay(50);

  //detect whether encoder has changed position
  long reading = encoder.read();
  long encoderChange = reading - encoderPosition;
  encoderPosition = reading;

  displayNeedsUpdate = false;
  
  //step up or down or change step size, for either button presses or encoder turns
  if (checkButtonPress(PIN_BUTTON_UP) || (encoderChange > 0)){currFreq += steps[stepIndex]; currFreq = min(currFreq, MAX_FREQ); si5351.set_freq(currFreq * 100ULL, 0ULL, SI5351_CLK0); displayNeedsUpdate = true;}
  if (checkButtonPress(PIN_BUTTON_DOWN) || (encoderChange < 0)){currFreq -= steps[stepIndex]; currFreq = max(currFreq, MIN_FREQ); si5351.set_freq(currFreq * 100ULL, 0ULL, SI5351_CLK0); displayNeedsUpdate = true;}
  if (checkButtonPress(PIN_BUTTON_STEP) || checkButtonPress(PIN_BUTTON_ENCODER)){stepIndex = (stepIndex + 1) % (MAX_STEP_INDEX+1); displayNeedsUpdate = true;}

}

void displayInfo(){
  lcd.clear();
  
  // frequency information should take up the first 11 spaces on the first line:
  if (currFreq >= 100000000) lcd.setCursor(4, 1);
  else if (currFreq > 10000000) lcd.setCursor(5, 1);
  else lcd.setCursor(6, 1);
  int mhz = int(currFreq/ 1000000);
  int khz = int((currFreq - (mhz*1000000)) / 1000);
  int hz =  int(currFreq % 1000);
  
  int khzPad = 0;
  if (khz < 100) khzPad++;
  if (khz < 10) khzPad++;
  
  int hzPad = 0;
  if (hz < 100) hzPad++;
  if (hz < 10) hzPad++;
  
  lcd.print(mhz);
  lcd.print(".");
  for (int i = 0; i < khzPad; i++) lcd.print("0");
  lcd.print(khz); 
  lcd.print(".");
  for (int i = 0; i < hzPad; i++) lcd.print("0");
  lcd.print(hz);
  
  //Step  Information should take the first 11 spaces on the 2nd line
  //The first 5 symbols are "STEP:", leaving 6 chars for step info.
  lcd.setCursor(4, 3);
  lcd.print("STEP:");
  lcd.print(stepNames[stepIndex]);
}

boolean checkButtonPress(int pin){
  long time = millis();
  if (buttonActive[pin] && digitalRead(pin) == HIGH){
    buttonActive[pin] = false;
    lastButtonPress[pin] = time;
  }
  else if (digitalRead(pin) == LOW && !buttonActive[pin] && time > lastButtonPress[pin] + BUTTON_DEBOUNCE_TIME){
    buttonActive[pin] = true;
    lastButtonPress[pin] = time;
    return true;
  }
  return false;
}
