//LCD library
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

//Prototype1.5
//Manaul select profiles, returns to Idle after done

//PID performance untested
//no zero-crossing detection yet

//-----24/05/2015-------
//Changing timer interrupt frequency to 10Hz to allow 5-cycle skips without zero-xrossing detection
//This requires to change the SSR(pump) to zerocrossing model to smooth the motion of the solenoid

//Adding Adafruit LCD shield library and testing of basic functions

#include "pid.c"

volatile long time = 0;
volatile long timeloop = 0;

int mode = 0;  //mode

//Define preset profile structure
typedef struct PRESET {
  float temperature;
  float duration;
  int duty;
}
preset_t;

//Define index for preset structure
typedef struct PRESETINDEX {
  preset_t* preset;
  int length; //number of 'steps' during brew cycle
}
presetIndex_t;


//-----------------------------PROFILES-------------------------------------------------------
//The format of the profile is shown in struct PRESET
//Preset profiles can be modified to users liking
//preset 0 is random?

preset_t preset0[] = {90.0, 0, 0};	//undefined with dummy temperature
preset_t preset1[] = {90.0, 2, 3, 90.0, 2, 1, 92.5, 25, 10};	//3 step brew
preset_t preset2[] = {90.5, 3, 4, 90.0, 3, 1, 90.0, 15, 10, 90.0, 10, 8, 90.0, 6, 6};	// 5 step brew
preset_t preset3[] = {50, 2, 1,50, 2, 2,50, 2, 3,50, 2, 4,50, 2, 5,50, 2, 6,50, 2, 7,50, 2, 8,50, 2, 9,50, 2, 10};	//Test
preset_t preset4[] = {55.5, 60, 5};	//off the shelf - 1 step brew

//random profiles, not selectable in menu
preset_t preset5[] = {88.2, 3, 5, 91.0, 1, 2, 92.0, 15, 10, 92.0, 11, 8, 92.5, 10, 7};
preset_t preset6[] = {89.3, 2, 3, 92.0, 2, 1, 91.0, 15, 10, 88.0, 9, 9, 93.7, 8, 8};
preset_t preset7[] = {87.6, 1, 1, 90.0, 3, 0, 92.0, 15, 10, 94.0, 8, 7, 91.3, 7, 9};
preset_t preset8[] = {92.3, 3, 5, 90.0, 3, 2, 93.0, 15, 10, 89.0, 9, 9, 89.8, 9, 6};
preset_t preset9[] = {90.0, 5, 2, 90.0, 2, 2, 93.0, 15, 10, 88.0, 12, 6, 91.5, 9, 8};
preset_t preset10[] = {91.2, 4, 2, 90.0, 3, 3, 94.0, 15, 10, 95.0, 10, 7, 88.1, 8, 9};
preset_t preset11[] = {93.0, 2, 4, 90.0, 3, 1, 89.0, 15, 10, 90.0, 9, 8, 93.2, 7, 7};
preset_t preset12[] = {88.0, 4, 1, 90.0, 0, 0, 91.0, 15, 10, 91.0, 7, 9, 95.7, 10, 6};
preset_t preset13[] = {91.5, 3, 3, 90.0, 1, 3, 95.0, 15, 10, 93.0, 8, 8, 94.8, 8, 9};
preset_t preset14[] = {90.8, 1, 2, 90.0, 2, 3, 92.0, 15, 10, 94.0, 10, 9, 88.9, 8, 7};
preset_t preset15[] = {92.7, 0, 2, 90.0, 0, 0, 93.0, 15, 10, 92.0, 10, 7, 90.5, 7, 8};



//The length of the index is changed with respect to the number of steps for the specific preset profile

presetIndex_t presetIndex[] =
{
  (preset_t*)&preset0,
  0,

  (preset_t*)&preset1,
  (sizeof(preset1) / sizeof(PRESET)),

  (preset_t*)&preset2,
  (sizeof(preset2) / sizeof(PRESET)),

  (preset_t*)&preset3,
  (sizeof(preset3) / sizeof(PRESET)),

  (preset_t*)&preset4,
  (sizeof(preset4) / sizeof(PRESET)),

  (preset_t*)&preset5,
  (sizeof(preset5) / sizeof(PRESET)),

  (preset_t*)&preset6,
  (sizeof(preset6) / sizeof(PRESET)),

  (preset_t*)&preset7,
  (sizeof(preset7) / sizeof(PRESET)),

  (preset_t*)&preset8,
  (sizeof(preset8) / sizeof(PRESET)),

  (preset_t*)&preset9,
  (sizeof(preset9) / sizeof(PRESET)),

  (preset_t*)&preset10,
  (sizeof(preset10) / sizeof(PRESET)),

  (preset_t*)&preset11,
  (sizeof(preset11) / sizeof(PRESET)),

  (preset_t*)&preset12,
  (sizeof(preset12) / sizeof(PRESET)),

  (preset_t*)&preset13,
  (sizeof(preset13) / sizeof(PRESET)),

  (preset_t*)&preset14,
  (sizeof(preset14) / sizeof(PRESET)),

  (preset_t*)&preset15,
  (sizeof(preset15) / sizeof(PRESET)),
};

//Point to current preset step
byte activePresetStep = 0;
byte activePresetIndex = 0; //Select preset profile
preset_t* activePreset = (preset_t*)presetIndex[0].preset;

//-------------------------Variables---------------------

int tempreadpin = A5; //Temperature reading pin
int pressurereadpin = A1; //Pressure reading pin

long pidprevtime = 0;
long pidreadprevtime = 0;
double pidinterval = 100; //Interval for PID calculation
double pidreadinterval = 50;

long pidprintprevtime = 0;
double pidprintinterval = 1000;
int skipcount = 0;

double setT = 0; //Set temperature

const int numread = 20; //Number of readings before averaging

double tempstring[numread]; //String of readings
int index = 0;
double total = 0;

double avrvalue = 0;
double R2 = 100500; //Series resistance value
double R0 = 100000; //100k NTC thermistor
double Vout = 0;
double temp = 0;
double beta = 3950; //Beat coefficient of the thermistor
double T0 = 298.15; //25 degree in K
double R1; //Current resistance of the thermistor


long pressurereadtime = 0;
long pressurereadprevtime = 0;
long pressurereadinterval = 10;
double pressurestring[numread];
int pindex = 0;
double ptotal = 0;
double pavrvalue = 0;
double pressure = 0;



//-----------------------Define PUMP and PWM------------------------------------------
/*
To set the frequency of the PWM signal to 10Hz, the TOP value is calculated to be 30D(HEX)
and the compare value is set at 0-100% of 30D for respective duty cycles
*/
typedef struct PUMP {
  void duty(byte value) {
    switch (value) {

      case 1: //10% duty
        TCCR4C &= ~(1 << COM4D0);
        cli();
        TC4H = 0x3;		//set top value 30D(hex) = 781(Dec)
        OCR4C = 0x0D;	//base frequency 16MHz/(2048*781) = 10Hz
        TC4H = 0x0;		//set compare value at 4E(hex) = 78(Dec)
        OCR4D = 0x4E;	//	78/781 ~= 10%
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 2: //20% duty
        TCCR4C &= ~(1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x0;
        OCR4D = 0x9C;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 3: //30% duty
        TCCR4C &= ~(1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x0;
        OCR4D = 0xEA;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 4: //40% duty
        TCCR4C &= ~(1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x1;
        OCR4D = 0x38;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 5: //50% duty
        TCCR4C &= ~(1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x1;
        OCR4D = 0x87;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 6: //60% duty
        TCCR4C |= (1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x1;
        OCR4D = 0x38;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 7:
        TCCR4C |= (1 << COM4D0); //70% duty
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x0;
        OCR4D = 0xEA;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 8: //80% duty
        TCCR4C |= (1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x0;
        OCR4D = 0x9C;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 9: //90% duty
        TCCR4C |= (1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x0;
        OCR4D = 0x4E;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      case 10: //100% duty
        TCCR4C &= ~(1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x3;
        OCR4D = 0x0D;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;

      default: //0% duty - OFF
        TCCR4C &= ~(1 << COM4D0);
        cli();
        TC4H = 0x3;
        OCR4C = 0x0D;
        TC4H = 0x0;
        OCR4D = 0x0;
        sei();
        TCCR4B = (1 << CS43) | (1 << CS42) ; //Set pre-scaler to 2048 and start timer
        break;
    }
  }
}
pump_t;

//Define Pump
pump_t pump;
//Point to Pump
pump_t* activepump = &pump;


//--------------------------LCD and related--------------------------------------------
//define lcd
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();


// These #defines make it easy to set the backlight color
#define OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7


int offcount = 0;
long sleeptime = 90; //time of inactivity before back to sleep mode (2mins)
long lasttime = 0;
long transitiontime = 15; //auto return to standby mode

int brewcyclecount = 0;
int checkwater = 12;

//---------------------------Setup-----------------------------------------

void setup() {

  Serial.begin(9600);
  //lcd setup
  lcd.begin(16, 2);

  //heater set-up
  pinMode(9, OUTPUT);
  ICR1 = 0x1E85; //TOP value = 1E(hex) = 7813(Dec), 16MHz/(1024*7813) = 2Hz
  OCR1A = 0x0; //Match value to vary PWM duty cycle default at 0 = off
  TCCR1A = (1 << COM1A1); //Clear OC1A on compare match (set output to low level)
  TCCR1B = (1 << WGM13) | (1 << CS12) | (1 << CS10); //Set PWM, Phase and Frequency Correct and pre-scaler to 1024

  //pump set-up
  pinMode(6, OUTPUT); //Sets Pin 6 As Output
  TCCR4C |= (1 << COM4D1); //Clear on up count
  TCCR4D = (1 << WGM40); //Phase & Frequency correct PWM

  //more on http://www.ermicro.com/blog/?p=1971

  //initialise value strings
  for (int current = 0; current < numread; current++) {
    tempstring[current] = 0;
    pressurestring[current] = 0;
  }


  PIDsetup(90, 0.05, 1, 100); //Set P,I,D gains and sample time
  PIDlimit(0, 0x1E85); //Set limit of PID output = ICR1

  randomSeed(analogRead(A3));
}


//-------------------------------Loop------------------------------------------


void loop() {

  timeloop = millis();

  //define buttons
  uint8_t buttons = lcd.readButtons();

  //The system will be in different mode for different operation
  switch (mode) {

    case 0: //Sleep Mode
      lcd.setBacklight(OFF);
      lcd.noDisplay();
      PIDreset();
      skipcount = 0;

      if (buttons) {
        lcd.clear();
        lcd.setBacklight(WHITE);
        lcd.setCursor(0, 0);
        lcd.print("Xpresso System");
        offcount = 0;
        mode = 1;
        lasttime = timeloop;
      }

      break;



    case 1: //Idle Mode
      //Point back to beginning of Preset
      activePresetStep = 0;
      activePreset = presetIndex[activePresetIndex].preset;

      lcd.display();
      lcd.setCursor(0, 0);
      lcd.print("Xpresso System");
      lcd.setCursor(0, 1);
      lcd.print("Profile:");
      lcd.print(activePresetIndex);
      lcd.print(" ");

      lcd.setCursor(12, 1);
      lcd.print(temp, 0);;
      lcd.print("C ");

      //profile selection + wrap
      if (buttons & BUTTON_LEFT ) {
        activePresetIndex--;
      }
      if (buttons & BUTTON_RIGHT) {
        activePresetIndex++;
      }
      //wrap
      if (activePresetIndex > 5) {
        activePresetIndex = 0;
      }

      //Select profile and start brew mode
      if (buttons & BUTTON_SELECT) {

        lcd.clear();
        if (activePresetIndex > 0 && activePresetIndex < 5) {

          if (brewcyclecount > checkwater) {
            mode = 4;
            lcd.clear();
            lcd.setBacklight(TEAL);
          }
          else {
            mode = 3;
            brewcyclecount++;
          }
        }

        else if (activePresetIndex == 0) {
          if (brewcyclecount > checkwater) {
            mode = 4;
            lcd.clear();
            lcd.setBacklight(TEAL);
          }
          else {
            activePresetIndex = random(0, 16);
            mode = 3;
            brewcyclecount++;
          }
        }

        else mode = 2;
      }

      //Manual Sleep
      if (buttons & BUTTON_UP) {

        offcount++;
        lcd.setCursor(15, 0);
        lcd.print(offcount);

        //Add reset function

      }
      if (offcount >= 3) {
        mode = 0;
      }

      //auto sleep
      if (buttons) {
        lasttime = timeloop;
        lcd.setBacklight(WHITE);
      }
      else if ((timeloop - lasttime) / 1000 >= sleeptime) {
        lcd.setBacklight(VIOLET);
        if (buttons) {
          lasttime = timeloop;
        }
        else if ((timeloop - lasttime) / 1000 >= sleeptime + 15) {
          mode = 0;
        }
      }


      break;



    case 2:  //Manaul select parameters



      if (buttons & BUTTON_UP) {

        offcount++;
        lcd.setCursor(15, 0);
        lcd.print(offcount);

        //Add reset function

      }
      if (offcount >= 3) {
        mode = 0;
      }


      break;



    case 3: //Brew Mode
      {
        lcd.setBacklight(RED);
        lcd.setCursor(0, 0);
        lcd.print("Brewing...");
        lcd.setCursor(10, 0);
        lcd.print("P");
        lcd.print(activePresetIndex);
        lcd.setCursor(14, 0);
        lcd.print("S");
        lcd.print(activePresetStep);

        unsigned long elapsedtime = millis() - time;
        lcd.setCursor(0, 1);
        lcd.print((elapsedtime / 1000));
        lcd.print("s  ");

        lcd.setCursor(4, 1);
        lcd.print(abs(pressure), 1);
        lcd.print("Bar ");

        lcd.setCursor(12, 1);
        lcd.print(temp, 0);;
        lcd.print("C ");

        //manual stop
        if (buttons & BUTTON_SELECT) {
          lcd.clear();
          lcd.setBacklight(YELLOW);
          lcd.setCursor(5, 0);
          lcd.print("Stopped!");
          mode = 5;
          lasttime = timeloop;
        }

        if (activePresetStep == 0) {

          activePreset = presetIndex[activePresetIndex].preset;
          activePresetStep = 1;
          time = millis();
          elapsedtime = 0;

        }

        else {

          if ((float)(millis() - time) / 1000 >= activePreset->duration) {

            if (presetIndex[activePresetIndex].length > activePresetStep) {

              activePreset++;
              activePresetStep++;
              time = millis();
              elapsedtime = 0;
            }

            else {
              lcd.clear();
              lcd.setBacklight(GREEN);
              lcd.setCursor(5, 0);
              lcd.print("Enjoy!");
              /*lcd.setCursor(0,1);
              lcd.print("Time taken: ");
              lcd.print(totaltime);
              lcd.print("s");*/
              mode = 5;
              lasttime = timeloop;
            }
          }
          if(buttons & BUTTON_RIGHT){
            
            if (presetIndex[activePresetIndex].length > activePresetStep) {
              activePreset++;
              activePresetStep++;
              time = millis();
              elapsedtime = 0;
            }
            else {
              lcd.clear();
              lcd.setBacklight(GREEN);
              lcd.setCursor(5, 0);
              lcd.print("Enjoy!");
              /*lcd.setCursor(0,1);
              lcd.print("Time taken: ");
              lcd.print(totaltime);
              lcd.print("s");*/
              mode = 5;
              lasttime = timeloop;
            }
           }//manaul skip end
          
        }//else end

      }
      break;

    case 4: //Check Water transition

      lcd.setCursor(0,0);
      lcd.print("Check Water");
      lcd.setCursor(0,1);
      lcd.print("Press Sel to CNL");

      if (buttons & BUTTON_SELECT) {
        lcd.clear();
        lcd.setBacklight(WHITE);
        mode = 1;
        brewcyclecount = 0;
      }
      else if(buttons){ 
        lcd.clear();
        lcd.setBacklight(RED);
        brewcyclecount = 0;
        mode = 3;
      }
      
      break;

    case 5: //Post brew Transition

      if (buttons & BUTTON_UP) {

        mode = 0;
      }

      else if (buttons || (timeloop - lasttime) / 1000 >= transitiontime) {
        lcd.clear();
        lcd.setBacklight(WHITE);
        mode = 1;
      }

      //auto sleep
      if ((timeloop - lasttime) / 1000 >= sleeptime) {
        lcd.setBacklight(VIOLET);
        if (buttons) {
          lasttime = timeloop;
        }
        else if ((timeloop - lasttime) / 1000 >= sleeptime + 20) {
          mode = 0;
        }
      }

      break;

  }


  //-------------------------Hardware read & control---------------------------------------

  //Pump activates in brew mode
  if (mode == 3) {
    activepump->duty(activePreset->duty);
  }
  
  //Turn pump off when not in use
  else {
    activepump->duty(0);
  }

  //------------------------------------------------------
  //pressure reading and calculation
   if (mode >= 1) {


    if ((float)millis() - pressurereadprevtime > pressurereadinterval) {

      ptotal = ptotal - pressurestring[pindex];

      pressurestring[pindex] = analogRead(pressurereadpin);

      ptotal = ptotal + pressurestring[index];

      pindex = pindex + 1;

      if (pindex >= numread) {
        pindex = 0;
      }

      pavrvalue = ptotal / numread;

      pressure = (300 * (pavrvalue - 105)) / (918 * 14.50377); //calculate psi and convert to Bar

      pressurereadprevtime = millis();
    }

  }
  
  




  //--------------------------------------------------------------
  //Temperature reading and PID calculation
  if (mode >= 1) {

    setT = activePreset->temperature;
    unsigned long pidtime = millis();

    if (pidtime - pidreadprevtime > pidreadinterval) {

      total = total - tempstring[index];

      tempstring[index] = analogRead(tempreadpin);

      total = total + tempstring[index];

      index++;

      if (index >= numread) {
        index = 0;
        skipcount++;
      }

      avrvalue = total / numread;

      Vout = (5 * avrvalue) / 1023;

      R1 = (5 * R2 / Vout) - R2;

      double T = (1 / T0) + (1 / beta) * log(R1 / R0);

      temp = 1 / T - 273.15;

      pidreadprevtime = pidtime;

    }

    if (skipcount > 2) { //skip first 2 readings before start to control heater to avoid overshoot

      if (setT - temp > 45) {  //force 100% while temperature difference is larger than 45, 
                                                     //reducing this value would cause overshoot 
        OCR1A = 0x1E85;  
      }

      else if ((setT - temp > 8 && activePresetStep > 1)) {
        OCR1A = 0xF43;  //force to 50% if temperature drops 8 degrees while brewing
      }

      else if (pidtime - pidprevtime > pidinterval) {
        
        PIDcompute(temp, setT); //pid calculation on current temp and set temp

        OCR1A = output;

        pidprevtime = pidtime;
      }
      
    }
  }

  else {
    setT = 0;
    OCR1A = 0;
  }
  
    //serial debug
  if (timeloop - pidprintprevtime > pidprintinterval) {

    Serial.print("Temp: ");
    Serial.println(temp, 1);
    Serial.print("Heater Duty: ");
    Serial.println(OCR1A/0x1E85);
    Serial.print("Pump Duty: ");
    Serial.println(activePreset->duty);
    Serial.print("Pressure read: ");
    Serial.println(pavrvalue);
    Serial.print("Pressure: ");
    Serial.println(pressure,2);
    Serial.println("-------------------------");
    pidprintprevtime = timeloop;
  }


}  //loop end





