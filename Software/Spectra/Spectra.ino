//------------------------------INITIAL SETUP----------------------------------------------------
#include <Keypad.h>
#include <Stepper.h>
#include <SoftwareSerial.h>
#include <Encoder.h>

//-----------------LED------------------------
#define LED 13

//--------------Light to Freq------------------
// Pin definitions
# define TSL235R 2                      // Out of TSL235R connected to Digital pin 2

// Constants
int period = 1000;                     // Miliseconds of each light frecuency measurement
int ScalingFactor = 1;                 // Scaling factor of this sensor
float area = 0.0092;                   // Sensing area of TSL235R device (cm2)

// Variables
unsigned long counter = 0;             // Counter of measurements during the test
unsigned long currentTime = millis();  
unsigned long startTime = currentTime; 
volatile long pulses = 0;              // Counter of measurements of the TSL235R
unsigned long frequency;               // Read the frequency from the digital pin (pulses/second)
float irradiance;                      // Calculated irradiance (uW/cm2)

//------------------LCD Display-------------------
#define txPin 1
SoftwareSerial LCD = SoftwareSerial(0, txPin);
// since the LCD does not send data back to the Arduino, we should only define the txPin
const int LCDdelay=10;  // conservative, 2 actually works

//-------------------Keypad-------------------------
const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
}; //connect K1-Ar10, K2-Ar9, K3-Ar8, K4-Ar7, K5-Ar6, K6-Ar5, K7-Ar4
byte rowPins[ROWS] = {9, 4, 5, 7}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {8, 10, 6}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//------------------Stepper-----------------------
const int stepsPerRevolution = 400;
Stepper myStepper(stepsPerRevolution, A0, A1, A2, A3);

//------------------Encoder-----------------------
Encoder enc(3,11);  //First pin has an interrupt
long enc_pos;

//-------------------------------MAIN CODE-----------------------------------------------
void setup(){
  pinMode(txPin, OUTPUT);
  pinMode(LED, OUTPUT);
  myStepper.setSpeed(20);
  LCD.begin(9600);
  clearLCD();
  LCD.print("  Device Name   ");
  LCD.print("     Here       ");
  backlightOn();
  attachInterrupt(0, PulseCount, RISING);
  pinMode(TSL235R, INPUT);                    // Declare the pin such as an input of data
  delay(5000);
}
  
void loop(){
  backlightOn();
  clearLCD();
  delay(1000);
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Press (#) when");
  lcdPosition(1,0);
  LCD.print("ready to image.");

  while(true){
    //Loop until #
    char key = keypad.getKey();
      if (key != NO_KEY){
        if (key == '#'){
          break;
        }
      }
  } 
  
  //To get the wavelength
  int waveln;
  while(true){
    //Loop until the user presses # to enter their wavelength.
    bool cont = true;
    char wavelength[3] = "";
    int num = 0;
    clearLCD();
    lcdPosition(0,0);
    LCD.print("Wavelength: (#)");
    while(cont) {
      char key = keypad.getKey();
      if (key == '#'){
        cont = false;
        break;      
      } 
      if (key == '*'){
        continue; //Ignore this
      }
      else if (key != NO_KEY){
         wavelength[num] = key;
         lcdPosition(1,num);
         LCD.print(key);
         num++;
      }
    }

    wavelength[3] = 0x00;
    waveln = atoi(wavelength);

    if (waveln < 250 || waveln > 750) {
      clearLCD();
      lcdPosition(0,0);
      LCD.print("Out of range.");
      lcdPosition(1,0);
      LCD.print("Please reenter.");
      delay(3000);
      continue;
    }

    clearLCD();
    lcdPosition(0,0);
    LCD.print("Is this correct?");
    lcdPosition(1,0);
    LCD.print(waveln);
    lcdPosition(1, 4);
    LCD.print("nm.  Y:# N:*");

    boolean loopexit = false;
    
    while(true) {
      char key = keypad.getKey();
      if (key != NO_KEY){
        if (key == '#'){
          loopexit = true;
          break;
        }
        else {
          clearLCD();
          lcdPosition(0,0);
          LCD.print("Please reenter.");
          delay(3000);
          break;
        }
      }
    }

    if (loopexit == true){
      break;
    }
  }


  //Now ready to do something with this wavelength  
  
  int samp; //determining the number of samples
  while(true){ //Loop until number of samples entered.
    //Ask for number of samples for imaging
    clearLCD();
    lcdPosition(0,0);
    LCD.print("Enter number of");
    lcdPosition(1,0);
    LCD.print("samples(#)");
    
    bool cont = true;
    char samples[3] = {000};
    int num = 0;
    while(cont) {
      char key = keypad.getKey();
      if (key == '#'){
        cont = false;
        break;      
      } 
      if (key == '*'){
        continue; //Ignore this
      }
      else if (key != NO_KEY){
         samples[num] = key;
         lcdPosition(1,11+num);
         LCD.print(key);
         num++;
      }
    }

    samples[num] = 0x00;
    samp = atoi(samples);
    
    if (samp < 1 || samp > 999) {
      clearLCD();
      lcdPosition(0,0);
      LCD.print("Out of range.");
      lcdPosition(1,0);
      LCD.print("Please reenter.");
      delay(3000);
      continue;
    }
    
    clearLCD();
    lcdPosition(0,0);
    LCD.print("Is this correct?");
    lcdPosition(1,0);
    LCD.print(samples);
    lcdPosition(1, 4);
    LCD.print("     Y:# N:*");

    boolean loopexit = false;
    
    while(true) {
      char key = keypad.getKey();
      if (key != NO_KEY){
        if (key == '#'){
          loopexit = true;
          break;
        }
        else {
          clearLCD();
          lcdPosition(0,0);
          LCD.print("Please reenter.");
          delay(3000);
          break;
        }
      }
    }

    if (loopexit == true){
      break;
    }
  }


  clearLCD();
  lcdPosition(0,0);
  LCD.print("Preparing for");
  lcdPosition(1,0);
  LCD.print("imaging...");
  delay(1500);

  //----------------------------MOTOR ROTATION AND ENCODER CHECK-----------------------------------
  //Rotate motor into proper position, continue in loop until it is verified

  float rot = (float)waveln / 1000.0;
  float angle = asin(rot);
  float rotNumber = round(angle * 400 / (2 * PI));
  int rotNum = (int)rotNumber;
  myStepper.step(rotNum);
  delay(100);
  
  while(true){  
    enc_pos = enc.read(); //With this encoder library, we have 4X counting. Pos is 0-4097
  
    //We want +/- 0.9 degree precision to match the motor specs
    long expected = (int)round(angle * 4098 / (2 * PI));
    long expected_min = (int)round((angle - 0.0157) * 4098 / (2 * PI));
    long expected_max = (int)round((angle + 0.0157) * 4098 / (2 * PI));

    if ((enc_pos <= expected_max) && (enc_pos >= expected_min)) {
      break;
    }
    else {
      int step_error = (int)round(((float)expected - (float)enc_pos) * 400.0 / 4098.0);
      myStepper.step(step_error); //If it has gone too far, step_error will be -, it will rotate back
      rotNum = rotNum + step_error; //Update the rotNum for the later return back to 0
      delay(100);
      continue;
    }
  }
  

  //------------------------------SAMPLE IMAGING--------------------------------------------------
  //Loop for imaging each sample
  for(int x = 1; x<= samp; x++){
    clearLCD();
    lcdPosition(0,0);
    LCD.print("Insert sample");
    lcdPosition(1,0);
    LCD.print("number ");
    lcdPosition(1, 7);
    LCD.print(x);
    lcdPosition(1, 13);
    LCD.print("(#)");

    while(true) { //Wait for user to press #
      char key = keypad.getKey();
      if (key != NO_KEY){
        if (key == '#'){
          break;
        }
      }
    }

    //Now that they have inserted the sample and pressed #, image

    clearLCD();
    lcdPosition(0,0);
    LCD.print("Imaging sample");
    delay(1000);

    //Insert code here for imaging and reading the intensity
    //May want to read 3-5 times, take average
    //Basically, turn LED on, take reading of photodiode while on, repeat
    //Then average the numbers, calculate intensity, and output it.

    //Turn LED on---------------------------------------------------
    digitalWrite(LED, HIGH);
    delay(1000);
    

    //Measuring the light intensity-----------------------------------
    float intensities[9];
    for(int x = 0; x < 9; x++) {
      getfrequency();                      // Request to measure the frequency
      getirradiance();                     // Request to calculate the irradiance (uW/cm2)
      pulses = 0;                          // reset the pulses counter
      intensities[x] = irradiance;
      delay(500);
    }
    //There can be weird readings when it starts, so throw out the first 4
    float sum = (intensities[4] + intensities[5] + intensities[6] + intensities[7] + intensities[8]);
    float avg_I = sum / 5;

    //----------------------------------------------------------------

    //Turn LED off-----------------------
    delay(1000);
    digitalWrite(LED, LOW);

    clearLCD();
    lcdPosition(0,0);
    LCD.print("Sample ");
    lcdPosition(0, 7);
    LCD.print(x);
    lcdPosition(1,0);
    LCD.print("I = ");
    lcdPosition(1, 4);
    LCD.print(avg_I, 3);
    lcdPosition(1, 11);
    LCD.print("W/cm2");

    while(true) { //Wait for user to press # to confirm
      char key = keypad.getKey();
      if (key != NO_KEY){
        if (key == '#'){
          break;
        }
      }
    }

    clearLCD();
    lcdPosition(0,0);
    LCD.print("Please remove");
    lcdPosition(1,0);
    LCD.print("sample ");
    lcdPosition(1, 7);
    LCD.print(x);
    delay(4000);
  }


  myStepper.step(-rotNum);
  while(true) { //Ensure it's truly back to 0
    enc_pos = enc.read();
    if ((enc_pos <= (0.0157 * 4098 / (2 * PI))) && (enc_pos >= (-0.0157 * 4098 / (2 * PI)))) { //Within 1 motor step of 0
      break;
    }
    else {
      int step_error = (int)round((0.0 - (float)enc_pos) * 400.0 / 4098.0);
      myStepper.step(step_error); //If it has gone past 0, step_error will be +, it will rotate back
      delay(100);
      continue;
    }
  }
  clearLCD();
  lcdPosition(0,0);
  LCD.print("Imaging complete");
  delay(1000);
  
}
//-----------------------------------END OF LOOP--------------------------------------------





//---------------------------------USER DEFINED FUNCTIONS-----------------------------------
// wbp: goto with row & column
void lcdPosition(int row, int col) {
  LCD.write(0xFE);   //command flag
  LCD.write((col + row*64 + 128));    //position 
  delay(LCDdelay);
}
void clearLCD(){
  LCD.write(0xFE);   //command flag
  LCD.write(0x01);   //clear command.
  delay(LCDdelay);
}
void backlightOn() {  //turns on the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(143);    //light level.
  delay(LCDdelay);
}
void backlightOff(){  //turns off the backlight
  LCD.write(0x7C);   //command flag for backlight stuff
  LCD.write(128);     //light level for off.
   delay(LCDdelay);
}
// set LCD contrast
void setContrast(int contrast){
  Serial.write(254); 
  Serial.write(80);   
  Serial.write(contrast);   
}

void PulseCount()
{
pulses++;
}

unsigned long getfrequency () {
 noInterrupts();
 frequency = pulses /(period/1000);    // Calculate the frequency (pulses/second)
 interrupts();
 return (frequency);
}

float getirradiance () {
 irradiance = frequency / area / 1000000;      // Calculate Irradiance (W/cm2)
 return (irradiance);
}

