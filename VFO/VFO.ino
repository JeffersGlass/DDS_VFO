#include <Encoder.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <si5351.h>


//-----------Variables & Declarations---------------
/*
 * The current and desired LISTENING FREQUENCY, which is not always the frequency being output by the Si5351.
 * In 'testing' and 'basic' modes, the output freqeuncy is equal to currFreq
 * In 'polyakov' mode, the output frequency is half of curFreq
 * In BFO mode, .........
 * These adjustments are mode in the setFrequency_5351 function depending on the current mode held in currMode
 */

long currFreq = 1800000; 

//-----Enumerations of frequency steps and their labels for each mode----//

enum modes{mode_testing = 0, mode_basic, mode_polyakov, mode_bfo};
const int NUM_MODES = 4;
int currMode = mode_basic;

char* modeNames[NUM_MODES] = {"TEST", "VFO", "POLYA", "BFO"};

long steps[][10] = { //don't forget to update the MAX_STEPS_INDEX array below
  {10000000, 5000000, 1000000, 500000, 100000, 10000, 1000, 10, 1}, //testing
  {10000, 1000, 100, 10}, //basic
  {1000, 100, 10, 1}, //polyakov
  {1000, 100, 10, 1}, //bfo
};

const int NUM_STEP_OPTIONS[NUM_MODES] = {
  10, //testing
  4, //basic
  4, //polyakov
  4, //bfo
};
char* stepNames[][10] = {
  {" 10MHz", "  5MHz", "  1MHz", "500Khz", "100KHz", " 10KHz", "  1KHz", " 100Hz", "  10Hz", "  1 Hz"}, //basic
  {" 10KHz", "  1KHz", " 100 Hz", " 10 Hz"}, //basic
  {"  1KHz", " 100 Hz", " 10 Hz", "  1 Hz"}, //polyakov
  {"  1KHz", " 100 Hz", " 10 Hz", "  1 Hz"} //BFO
};

int stepIndex = 0; // holds the index of the currently selected step value

//-----AMATEUR BAND DEFININTIONS----------------//
//See function "getCurrentBand" below as well
const int NUM_BANDS = 9;
char* bandNames[NUM_BANDS] = {"160m", "80m", "40m", "30m", "20m", "17m", "15m", "12m", "10m"};
char* OUT_OF_BAND_LABEL = "OOB";

long bandEdges[NUM_BANDS][2] = {
  {1800000, 2000000}, //160m
  {3500000, 4000000}, //80m
  {7000000, 7300000}, //40m
  {10100000, 10150000}, //30m
  {14000000, 14350000}, //20m
  {18068000, 18168000}, //17m
  {21000000, 21450000}, //15m
  {24890000, 24990000}, //12m
  {28000000, 29700000} //10m
};

/*
 * Holds the last-seen frequency within each band. The list below is also the default location at bootup.
 * This array is updated when the BAND button is used to change between bands. 
 * If the used has scrolled outside of a defined band and then presses the BAND button, they will
 * still be advanced to the next band, but the band-return location will not be updated
 */

long lastBandFreq[NUM_BANDS] = {
  1800000, //160m
  3500000, //80m
  7000000, //40m
  10100000, //30m
  14000000, //20m
  18068000, //17m
  21000000, //15m
  24890000, //12m
  28000000 //10m
};

/*Information on bandplan permissions and recommended communication modes is contained in the 
 * methods getPermission and getBandplanModes below
 */

//---------------------------------------------

long lastButtonPress[] = {0,0,0,0,0,0,0}; //holds the last timestamp, from millis(), that a pin changed state. Directly references the arduino output pin numbers, length may need to be increased
boolean buttonActive[] = {false, false, false, false, false, false, false};

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
const int PIN_BUTTON_MODE = 4;
const int PIN_BUTTON_BAND = 0;
const int BUTTON_DEBOUNCE_TIME = 10; //milliseconds

void setup(){
  // inialize LCD, display welcome message
  lcd.begin(20, 4);
  delay(250);
  lcd.setCursor(4, 1);
  lcd.print("VFO STARTING");
  
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0);
  si5351.set_freq(currFreq * 100ULL, 0ULL, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  delay(750);
  
    //knob.write(0);
  pinMode(PIN_BUTTON_ENCODER, INPUT);
  digitalWrite(PIN_BUTTON_ENCODER, HIGH);
  
  pinMode(PIN_BUTTON_MODE, INPUT);
  digitalWrite(PIN_BUTTON_MODE, HIGH);
  pinMode(PIN_BUTTON_BAND, INPUT);
  digitalWrite(PIN_BUTTON_BAND, HIGH); 

  lcd.clear();
  lcd.setCursor(2, 7);
  lcd.print("WELCOME!");
  delay(500);
  displayInfo();
}

void loop(){
  if (displayNeedsUpdate) {displayInfo();}
  delay(80);

  //detect whether encoder has changed position
  long reading = encoder.read();
  long encoderChange = reading - encoderPosition;
  encoderPosition = reading;

  displayNeedsUpdate = false;
  
  //step up or down or change step size, for either button presses or encoder turns
  if ((encoderChange > 0)){currFreq += steps[currMode][stepIndex]; currFreq = min(currFreq, MAX_FREQ); setFrequency_5351(currFreq); displayNeedsUpdate = true;}
  if ((encoderChange < 0)){currFreq -= steps[currMode][stepIndex]; currFreq = max(currFreq, MIN_FREQ); setFrequency_5351(currFreq); displayNeedsUpdate = true;}
  
  //pressing the encoder button increments through the possible step sizes for each mode
  if (checkButtonPress(PIN_BUTTON_ENCODER)){stepIndex = (stepIndex + 1) % (NUM_STEP_OPTIONS[currMode]); displayNeedsUpdate = true;}

  //pressing the mode button cycles through the available modes
  if (checkButtonPress(PIN_BUTTON_MODE)){currMode = (currMode+1) % NUM_MODES; stepIndex = 0; setFrequency_5351(currFreq); displayNeedsUpdate = true;}

  /*The mode button: if currFreq is inside an amateur band, save that frequency as the one to return to when
   * the user returns to this band, and jump to the return frequency for the next higher band. Otherwise,
   * just jump to the next higher band
  */
  if (checkButtonPress(PIN_BUTTON_BAND)){
    int currBand = getCurrentBand();
    if (currBand >= 0){
      lastBandFreq[currBand] = currFreq;
      currFreq = lastBandFreq[(getCurrentBand() + 1) % NUM_BANDS];
      setFrequency_5351(currFreq);
    }
    else if (currBand == -2 || currBand == -3){
      currFreq = lastBandFreq[0];
      setFrequency_5351(currFreq);
    }
    else if (currBand == -1){
      for (int i = 0; i < NUM_BANDS; i++){
        if (currFreq < lastBandFreq[i]){currFreq = lastBandFreq[i]; setFrequency_5351(currFreq); break;}
      }
    }
    displayNeedsUpdate = true;
  }
}

void displayInfo(){
  lcd.clear();
  
  // frequency information be centeredw within 11 spaces on the second line:
  if (currFreq >= 100000000) lcd.setCursor(3, 0);
  else if (currFreq > 10000000) lcd.setCursor(4, 0);
  else lcd.setCursor(5, 0);
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

  //The current amateur band is printed in the top-right corner
  int currBand = getCurrentBand();
  if (currBand >= 0){
    char* currBandName = bandNames[currBand];
    lcd.setCursor(20-strlen(currBandName), 0);
    lcd.print(currBandName);
  }
  else{
    lcd.setCursor(20-strlen(OUT_OF_BAND_LABEL), 0);
    lcd.print(OUT_OF_BAND_LABEL);
  }

  //The license needed to operate on this frequency (ARRL, USA ONLY) is printed just below the band label
  lcd.setCursor (19, 1);
  lcd.print(getPermission());
  
  //Step  Information should take the middle 11 spaces on the 3nd line
  //The first 5 symbols are "STEP:", leaving 6 chars for step info.
  lcd.setCursor(4, 2);
  lcd.print("STEP:");
  lcd.print(stepNames[currMode][stepIndex]);

  //Callsign is printed at the beginning of the 4th line
  lcd.setCursor(0, 3);
  lcd.print("KK9JEF");

  //The mode is printed on the 4th line with no label
  //lcd.setCursor(6, 3);
  lcd.setCursor(20-strlen(modeNames[currMode]), 3);
  lcd.print(modeNames[currMode]);

  //DEBUG
  lcd.setCursor(0,0);
  lcd.print(getCurrentBand());
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

void setFrequency_5351(long newFreq){
  switch (currMode){
    case mode_testing:
      si5351.set_freq(newFreq * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_basic:
      si5351.set_freq(newFreq * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_polyakov:
      si5351.set_freq((newFreq / 2) * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_bfo:
      si5351.set_freq(newFreq * 100ULL, 0ULL, SI5351_CLK0);
      break;
  }
}

//Returns the index of the current amateur radio band based on currFreq. Does not include the 60m band
//Returns -1 if out of band, but within the HF amateur turning range
//returns -2 if out of band and lower than the lowest defined band
//returns -3 if out of band and higher than the highest defined band
int getCurrentBand(){
 if (currFreq < bandEdges[0][0]) return -2; //we are lower than the lower edge of the lowest defined band
 if (currFreq > bandEdges[NUM_BANDS-1][1]) return -3; //We are higher than the upper edge of the highest defined band
 for (int i = 0; i < NUM_BANDS; i++){
  if (currFreq >= bandEdges[i][0] && currFreq <= bandEdges[i][1]){return i;} //We are within a band
 } 
 return -1;
}

char getPermission(){
  if (getCurrentBand() < 0) return ' ';
  
  //160m
  if (currFreq >= 1800000 && currFreq <= 2000000) return 'G';

  //80m
  if  (currFreq >= 3525000 && currFreq <= 3600000) return 'T';
  if ((currFreq >= 3525000 && currFreq <= 3600000) || (currFreq >= 3800000 && currFreq <= 4000000)) return 'G';
  if ((currFreq >= 3525000 && currFreq <= 3600000) || (currFreq >= 3700000 && currFreq <= 4000000)) return 'A';
  if  (currFreq >= 3500000 && currFreq <= 4000000) return 'E';

  //40m
  if  (currFreq >= 7025000 && currFreq <= 7125000) return 'T';
  if ((currFreq >= 7025000 && currFreq <= 7125000) || (currFreq >= 7175000 && currFreq <= 7300000)) return 'G';
  if  (currFreq >= 7025000 && currFreq <= 7300000) return 'A';
  if  (currFreq >= 7000000 && currFreq <= 7300000) return 'E';

  //30m
  if (currFreq >= 10100000 && currFreq <= 10150000) return 'G';

  //20m
  if ((currFreq >= 14025000 && currFreq <= 14150000) || (currFreq >= 14225000 && currFreq <= 14350000)) return 'G';
  if ((currFreq >= 14025000 && currFreq <= 14150000) || (currFreq >= 14175000 && currFreq <= 14350000)) return 'A';
  if  (currFreq >= 14000000 && currFreq <= 14350000) return 'E';

  //17m
  if (currFreq >= 18068000 && currFreq <= 18168000) return 'G';

  //15m
  if  (currFreq >= 21025000 && currFreq <= 21200000) return 'T';
  if ((currFreq >= 21025000 && currFreq <= 21200000) || (currFreq >= 21275000 && currFreq <= 21450000)) return 'G';
  if ((currFreq >= 21025000 && currFreq <= 21200000) || (currFreq >= 21225000 && currFreq <= 21450000)) return 'A';
  if  (currFreq >= 21000000 && currFreq <= 21450000) return 'E';

  //12m
  if (currFreq >= 24890000 && currFreq <= 24990000) return 'G';

  //10m
  if (currFreq >= 28000000 && currFreq <= 28500000) return 'T';
  if (currFreq >= 28000000 && currFreq <= 29700000) return 'G';

  return 'X';
}

