#include <EEPROM.h>
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

//FOR CQ MODE:
char CQ[] = "-.-. --.-";
char DE[] = "-.. .";
char morseCallsign[] = "-.- -.- ----. .--- . ..-.";
int morseCallsignLength = 25;
long morseElementLength = 70; //ms

//FOR WSPR MODE

double correctionFactor = 0; //adjusts the offset of the Si5351, in parts per million
double prevCorrectionFactor = correctionFactor; //allows us to track whether the CF has changed, to save on EEPROM writes
int WSPR_TRANSMISSION_DATA[] = { //KK9JEF EN61 30
      3,3,2,0,0,0,0,0,3,0,0,2,1,1,1,0,0,2,1,2,2,1,0,3,1,3,1,2,2,0,
      0,0,2,2,1,0,2,3,0,1,2,0,2,2,2,2,3,2,3,1,2,0,1,3,2,3,2,0,2,1,
      1,0,1,0,0,2,2,1,1,0,1,0,3,0,1,2,1,0,2,3,0,0,1,0,1,1,2,2,2,3,
      1,0,1,2,3,2,2,0,1,2,2,0,2,0,1,2,2,3,0,2,1,1,1,0,1,3,2,2,3,1,
      0,1,0,2,2,1,1,1,2,0,0,0,0,3,0,1,0,2,3,1,2,2,2,2,0,2,2,3,1,0,
      1,2,1,3,2,0,2,3,3,2,0,2};

/*
int WSPR_TRANSMISSION_DATA[] = { //KK9JEF EN61 27
      3,3,2,0,0,2,0,2,3,0,0,2,1,3,1,2,0,2,1,0,2,3,0,1,1,1,1,0,2,2,
      0,0,2,2,1,2,2,3,0,1,2,2,2,2,2,0,3,0,3,1,2,0,1,3,2,1,2,0,2,3,
      1,0,1,2,0,2,2,1,1,0,1,0,3,2,1,0,1,2,2,3,0,0,1,0,1,1,2,2,2,3,
      1,2,1,2,3,0,2,0,1,0,2,0,2,0,1,2,2,3,0,0,1,1,1,0,1,1,2,0,3,3,
      0,3,0,0,2,1,1,3,2,0,0,2,0,1,0,3,0,2,3,1,2,2,2,0,0,2,2,3,1,0,
      1,0,1,1,2,0,2,1,3,0,0,2};
*/

//-----Enumerations of frequency steps and their labels for each mode----//

enum modes{mode_testing = 0, mode_basic, mode_polyakov, mode_bfo, mode_WSPR, mode_CQ, mode_calibrate};
const int NUM_MODES = 7;
int currMode = mode_basic;

char* modeNames[NUM_MODES] = {"TEST", "VFO", "POLYA", "BFO", "WSPR", "CQ", "CAL"};

long steps[][10] = { //don't forget to update the NUM_STEP_OPTIONS array below
  {10000000, 5000000, 1000000, 500000, 100000, 10000, 1000, 10, 1}, //testing
  {10000, 1000, 100, 10}, //basic
  {1000, 100, 10, 1}, //polyakov
  {1000, 100, 10, 1}, //bfo
  {5}, //WSPR
  {500}, //CQ
  {1} //calibrate
};

const int NUM_STEP_OPTIONS[NUM_MODES] = {
  10, //testing
  4, //basic
  4, //polyakov
  4, //bfo
  1, //wspr
  1, //cq
  1  //calibrate
};
char* stepNames[][10] = {
  {" 10MHz", "   5MHz", "  1MHz", "500Khz", "100KHz", " 10KHz", "  1KHz", " 100Hz", "  10Hz", "  1 Hz"}, //basic
  {" 10KHz", "   1KHz", " 100Hz", " 10 Hz"}, //basic
  {"  1KHz", " 100 Hz", " 10 Hz", "  1 Hz"}, //polyakov
  {"  1KHz", " 100 Hz", " 10 Hz", "  1 Hz"}, //BFO
  {"  5 Hz"}, //WSPR
  {" 500Hz"}, //CQ
  {" 1 ppm"} //Calibrate
};

int stepIndex = 0; // holds the index of the currently selected step value

//-----AMATEUR BAND DEFININTIONS----------------//
//See function "getCurrentBand" below as well
const int NUM_BANDS = 10;
char* bandNames[NUM_BANDS] = {"160m", "80m", "60m", "40m", "30m", "20m", "17m", "15m", "12m", "10m"};
char* OUT_OF_BAND_LABEL = "OOB";

long bandEdges[NUM_BANDS][2] = {
  {1800000, 2000000}, //160m
  {3500000, 4000000}, //80m
  {5288600, 5288800},
  {7000000, 7300000}, //40m
  {10100000, 10150000}, //30m
  {14000000, 14350000}, //20m
  {18068000, 18168000}, //17m
  {21000000, 21450000}, //15m
  {24890000, 24990000}, //12m
  {28000000, 29700000} //10m
};

long WSPRbandEdges[NUM_BANDS][2] = {
  {1838000, 1838200}, //160m
  {3594000, 3594200}, //80m
  {5288600, 5288800}, //60m
  {7040000, 7040200}, //40m
  {10140100, 10140300}, //30m
  {14097000, 14097200}, //20m
  {18106000, 18106200}, //17m
  {21096000, 21096200}, //15m
  {24926000, 24926200}, //12m
  {28126000, 28126200}, //10m
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
  5288600, //60m
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

//Onboard LED Steup
const int PIN_LED = 13;

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

//SWR Sensor Pins
const int PIN_SWR_FORWARD = A1;
const int PIN_SWR_REVERSE = A0;

void setup(){
  // inialize LCD, display welcome message
  lcd.begin(20, 4);
  delay(250);
  lcd.setCursor(4, 1);
  lcd.print("VFO STARTING");
  
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
      
  pinMode(PIN_BUTTON_ENCODER, INPUT);
  digitalWrite(PIN_BUTTON_ENCODER, HIGH);
  
  pinMode(PIN_BUTTON_MODE, INPUT);
  digitalWrite(PIN_BUTTON_MODE, HIGH);
  pinMode(PIN_BUTTON_BAND, INPUT);
  digitalWrite(PIN_BUTTON_BAND, HIGH); 

  pinMode(PIN_SWR_FORWARD, INPUT);
  pinMode(PIN_SWR_REVERSE, INPUT);

  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("READING CALIBRATION");

  double tempDouble;
  EEPROM.get(0, correctionFactor);
  //if (tempDouble < 0.00001 && tempDouble > -0.00001){correctionFactor = tempDouble;}
  //else {correctionFactor = 0.00f;}
  lcd.setCursor(0, 2);
  lcd.print("*");
  lcd.print(tempDouble);
  lcd.setCursor(0, 3);
  lcd.print("=");
  lcd.print(correctionFactor);
  delay(1000);

  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0);
  si5351.set_freq((currFreq * (1 + correctionFactor)) * 100ULL, 0ULL, SI5351_CLK0);
  enableOutput();
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  
  si5351.output_enable(SI5351_CLK1, 0);
  si5351.output_enable(SI5351_CLK2, 0);
  delay(300);

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
  
  //step up or down or change step size, for encoder turns
  if (currMode != mode_calibrate){
    if ((encoderChange > 0)){currFreq += steps[currMode][stepIndex]; currFreq = min(currFreq, MAX_FREQ); setFrequency_5351(currFreq); displayNeedsUpdate = true;}
    if ((encoderChange < 0)){currFreq -= steps[currMode][stepIndex]; currFreq = max(currFreq, MIN_FREQ); setFrequency_5351(currFreq); displayNeedsUpdate = true;}
  }
  else{
    if (encoderChange > 0){correctionFactor += 0.0000001; setFrequency_5351(currFreq); displayNeedsUpdate = true;}
    if (encoderChange < 0){correctionFactor -= 0.0000001; setFrequency_5351(currFreq); displayNeedsUpdate = true;}
  }
  
  //pressing the encoder button increments through the possible step sizes for each mode;
  //in WSPR or CQ modes, the encoder button triggers the transmission of WSPR or a CQ, respectively.
  if (checkButtonPress(PIN_BUTTON_ENCODER)){
    if (currMode == mode_testing || currMode == mode_basic || currMode == mode_polyakov || currMode == mode_bfo) {
      stepIndex = (stepIndex + 1) % (NUM_STEP_OPTIONS[currMode]); 
      displayNeedsUpdate = true;
    }
    else if (currMode == mode_WSPR){
      transmitWSPR();
    }
    else if (currMode == mode_CQ){
      transmitMorseWord(CQ);
      transmitSpace();
      transmitMorseWord(CQ);
      transmitSpace();
      transmitMorseWord(DE);
      transmitSpace();
      transmitMorseWord(morseCallsign);
      transmitSpace();
      transmitMorseWord(morseCallsign);
    }
    else if (currMode == mode_calibrate){
      correctionFactor = 0.00;
    }
  }

  //pressing the mode button cycles through the available modes
  if (checkButtonPress(PIN_BUTTON_MODE)){

      //if the correctionFacotr has changed and we're leaving calibration mode, write the new correction factor to the EEPROM
      //Note that we do this check before actually advancing to the next mode
      if (currMode == mode_calibrate && correctionFactor != prevCorrectionFactor){
        writeCF();
        //DEBUG
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("CALIBRATION STORED");
        delay(1000);
        displayNeedsUpdate = true;

        double tempDouble;
        EEPROM.get(0, tempDouble);
        if (tempDouble < 0.00001 && tempDouble > -0.00001){correctionFactor = tempDouble;}
        else correctionFactor = 0.00f;
        lcd.setCursor(0, 2);
        lcd.print("*");
        lcd.print(tempDouble);
        lcd.setCursor(0, 3);
        lcd.print("=");
        lcd.print(correctionFactor);
        delay(10000);
      }

      //actually change the mode, and reset the step index
      currMode = (currMode+1) % NUM_MODES;
      stepIndex = 0;

      //if entering calibration mode, make a note of the current correction factor so we can tell later if it changes
      if (currMode == mode_calibrate){
        prevCorrectionFactor = correctionFactor;
      }
      
      if (currMode == mode_WSPR){ //If entering WSPR mode, set the current freqency to the bottom of the WSPR band slice
        currFreq = findWSPRBand();
      }

      if (currMode == mode_WSPR || currMode == mode_CQ){
        disableOutput(); //In WSPR or CQ mode, the transmitter should be off until manually triggered
      }
      else{
        enableOutput(); //In all other modes, the output of the VFO is on by default
      }
      setFrequency_5351(currFreq);
      displayNeedsUpdate = true;
    }

  /*The band button: if currFreq is inside an amateur band, save that frequency as the one to return to when
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
    if (currMode == mode_WSPR){ //WSPR mode behaves differntly from other modes
      currFreq = WSPRbandEdges[getCurrentBand()][0];
      setFrequency_5351(currFreq);
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

  //If we're in calibration mode, print current calibration factor on line 2:
  if (currMode == mode_calibrate){
    lcd.setCursor(0, 1);
    lcd.print("CORRECTION");

    int correctionPPM = int(correctionFactor * pow(10, 6));
  
    int correctionPad = 0;
    if (correctionPPM < 100) correctionPad++;
    if (correctionPPM < 10) correctionPad++;
    if (correctionPPM > 0) correctionPad++;

    lcd.setCursor(9, 1);  
    for (int i = 0; i < correctionPad; i++) lcd.print(" ");
    lcd.print(correctionPPM);
  }
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
      si5351.set_freq((newFreq * (1 + correctionFactor)) * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_basic:
      si5351.set_freq((newFreq * (1 + correctionFactor)) * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_polyakov:
      si5351.set_freq(((newFreq * (1 + correctionFactor))/ 2) * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_bfo:
      si5351.set_freq((newFreq * (1 + correctionFactor)) * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_WSPR:
      si5351.set_freq((newFreq * (1 + correctionFactor)) * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_CQ:
      si5351.set_freq((newFreq * (1 + correctionFactor)) * 100ULL, 0ULL, SI5351_CLK0);
      break;
    case mode_calibrate:
      si5351.set_freq((newFreq * (1 + correctionFactor)) * 100ULL, 0ULL, SI5351_CLK0);
      break;
  }
}

void enableOutput(){
  si5351.output_enable(SI5351_CLK0, 1);
  digitalWrite(PIN_LED, HIGH);
}

void disableOutput(){
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(PIN_LED, LOW);
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

void transmitWSPR(){
  long startTime = millis();
  enableOutput();
  for (int dataFrame = 0; dataFrame < 162; dataFrame++){
    si5351.set_freq((currFreq + correctionFactor) * 100ULL + (146*WSPR_TRANSMISSION_DATA[dataFrame]*1ULL), SI5351_PLL_FIXED, SI5351_CLK0);
    displayWSPR(dataFrame);
    while (millis() < startTime + 683*(dataFrame+1)){
      if (checkButtonPress(PIN_BUTTON_ENCODER) || dataFrame > 162){
        goto escape;
      }
    }
  }
  escape:
  disableOutput();
  displayNeedsUpdate = true;
}

void displayWSPR(int frame){
  lcd.clear();

  lcd.setCursor(0,0);
  long printFreq = currFreq * 100ULL + (146*WSPR_TRANSMISSION_DATA[frame]*1ULL);
  lcd.print(printFreq);
  
  //current frame and data are printed on the 3rd line
  lcd.setCursor(0, 2);
  lcd.print("FRAME:");
  lcd.setCursor(6, 2);
  lcd.print(frame);
  lcd.setCursor(10, 2);
  lcd.print("DATA:");
  lcd.setCursor(15, 2);
  lcd.print(WSPR_TRANSMISSION_DATA[frame]);

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

  //Callsign is printed at the beginning of the 4th line
  lcd.setCursor(0, 3);
  lcd.print("KK9JEF");

  //The mode is printed on the 4th line with no label
  //lcd.setCursor(6, 3);
  lcd.setCursor(16, 3);
  lcd.print("WSPR");
}

//When switching into WSPR mode, the VFO jumps to the WSPR portion of the appropriate band; if not inside a band when switching to WSPR mode, 
//This fucntion determines where to jump to.
//Currently always resets to the bottom of the lowest band
long findWSPRBand(){
  /*switch (getCurrentBand()){
  case -2: //Below the lowest defined band
    currFreq = WSPRbandEdges[0][0]; //set frequency to the bottom edge of lowest band
    break;
  case -3: //Above the highest defined band
    currFreq = WSPRbandEdges[NUM_BANDS-1][0]; //Set frequency to bottom edge of highest band       
    break;
  case -1: //in between bands
    break;
  default:
    currFreq = currFreq = (currFreq % 5); //round to the nearest multiple of 5 hz, for readability in WSPR mode
    break;
  }
  */
  return WSPRbandEdges[0][0];
}

void transmitMorseWord(char singleWord[]){
  for (int i = 0; i < strlen(singleWord); i++){
    if (singleWord[i] == '-') transmitDash();
    else if (singleWord[i] == '.') transmitDot();
    else transmitIntracharacter();
  }
}

void transmitDash(){
 enableOutput();
 delay(morseElementLength * 3);
 disableOutput();
 delay(morseElementLength);
}

void transmitDot(){
 enableOutput();
 delay(morseElementLength);
 disableOutput();
 delay(morseElementLength);
}

void transmitIntracharacter(){
 disableOutput();
 delay(morseElementLength*2); //each element naturally has a one-dot space built in
}

void transmitSpace(){
  disableOutput();
  delay(morseElementLength*6); //each element naturally has a one-dot space built in that follows it.
}

void writeCF(){
  EEPROM.put(0, correctionFactor);
}

