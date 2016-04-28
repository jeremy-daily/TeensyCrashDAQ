/* Example for analogContinuousRead
*  It measures continuously the voltage on pin A9,
*  Write v and press enter on the serial console to get the value
*  Write c and press enter on the serial console to check that the conversion is taking place,
*  Write t to check if the voltage agrees with the comparison in the setup()
*  Write s to stop the conversion, you can restart it writing r.
*/

#include <Wire.h>
#include <ADC.h>
#include "RingBuffer.h"
#include <IntervalTimer.h>
#include <SPI.h>
#include "SdFat.h"
#include "FreeStack.h"
#include <TimeLib.h>        // Imports Time Libraries
#include <TinyGPS.h>       // http://arduiniana.org/libraries/TinyGPS/

const int XPin = A0; // ADC0
const int YPin = A1; // ADC0
const int ZPin = A2; // ADC0 or ADC1
const int STPin = A3; //SelfTest
const int NFET1Pin = 21; //NFET1 Driver
const int NFET2Pin = 5; //NFET1 Driver
const int nResetPin = 22; //NFET1 Driver
const int nBootPin = 23; //NFET1 Driver
const int IMUIntPin = 6; //NFET1 Driver
const int SDchipSelect = 10; //NFET1 Driver
const int StandbyPin = 7; //NFET1 Driver
const int PS0Pin = 2; //NFET1 Driver


ADC *adc = new ADC(); // adc object
SdFat sd;

char AccelFilename[37];
File accelDataFile;

char XaccelString[10];
char YaccelString[10];
char ZaccelString[10];

time_t startTime;

boolean SDOK = false;
boolean LEDState = true;

RingBuffer *bufferX = new RingBuffer; // buffers to store the values
RingBuffer *bufferY = new RingBuffer;
RingBuffer *bufferZ = new RingBuffer;
RingBuffer *bufferXtime = new RingBuffer; // buffers to store the values
RingBuffer *bufferYtime = new RingBuffer;
RingBuffer *bufferZtime = new RingBuffer;

uint32_t currentMicros,previousMicros;



void setup() {

    pinMode(XPin, INPUT);
    pinMode(YPin, INPUT);
    pinMode(ZPin, INPUT);
    pinMode(STPin, OUTPUT);
    pinMode(NFET1Pin, OUTPUT);
    pinMode(NFET2Pin, OUTPUT);
    pinMode(nResetPin, OUTPUT);
    pinMode(nBootPin, OUTPUT);
    pinMode(IMUIntPin, INPUT);
    pinMode(SDchipSelect, OUTPUT);
    pinMode(StandbyPin, OUTPUT);
    pinMode(PS0Pin, OUTPUT);
    
    digitalWrite(SDchipSelect,HIGH);
    digitalWrite(STPin,LOW);
    digitalWrite(NFET1Pin,LOW);
    digitalWrite(NFET2Pin,LOW);
    digitalWrite(nResetPin,HIGH);
    digitalWrite(nBootPin,HIGH);
    digitalWrite(nResetPin,HIGH);
    
    Serial.begin(9600);
    while(!Serial);
    
    ///// ADC0 ////
    // reference can be ADC_REF_3V3, ADC_REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
    adc->setReference(ADC_REF_3V3, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2
    adc->setReference(ADC_REF_3V3, ADC_1); // change all 3.3 to 1.2 if you change the reference to 1V2

    adc->setAveraging(16); // set number of averages
    adc->setResolution(16); // set bits of resolution

    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    // see the documentation for more information
    // additionally the conversion speed can also be ADC_ADACK_2_4, ADC_ADACK_4_0, ADC_ADACK_5_2 and ADC_ADACK_6_2,
    // where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
    adc->setConversionSpeed(ADC_HIGH_SPEED_16BITS); // change the conversion speed
    // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
    adc->setSamplingSpeed(ADC_HIGH_SPEED); // change the sampling speed

    // always call the compare functions after changing the resolution!
    //adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
    //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_0)/3.3, 2.0*adc->getMaxValue(ADC_0)/3.3, 0, 1, ADC_0); // ready if value lies out of [1.0,2.0] V

    // If you enable interrupts, notice that the isr will read the result, so that isComplete() will return false (most of the time)
    adc->enableInterrupts(ADC_0);

    
     //Create new data files.
     Serial.println("Initializing SD card...");
     while(!SDOK){ 
     // see if the card is present and can be initialized:
     if (sd.begin(SDchipSelect)) {
      SDOK = true;
      Serial.println("SD card initialized.");
      
      sprintf(AccelFilename, "%04d-%02d-%02d_%02d%02d%02d_HighGAccellog.csv", year(),month(),day(),hour(),minute(),second()); 
      SdFile::dateTimeCallback(dateTime);
      accelDataFile = sd.open(AccelFilename, O_RDWR | O_CREAT | O_APPEND);
      accelDataFile.println("Accelerometer Data File from Teensy IMU Caper");
      accelDataFile.println("Accelerometer Data File from Teensy IMU Caper");
      accelDataFile.println("Time,X Accel, ,Time,Y Accel, ,Time,Z Accel");
      accelDataFile.println("Microseconds,g, ,Microseconds,g, ,Microseconds,g");
      SdFile::dateTimeCallbackCancel(); //keeps the time of creation of the file from being updated
    }
    else 
    {
      SDOK = false;
      Serial.println("SD Card failed, or not present.");
      LEDState= !LEDState;
      digitalWrite(13,LEDState);
      delay(50);

    }
     }
   startTime=now();
 
   adc->startContinuous(XPin);
  
   
    
}

uint16_t Xvalue = 0;
uint16_t Yvalue = 0;
uint16_t Zvalue = 0;
char c=0;

void loop() {
  currentMicros = micros();
  
    
//  if (currentMicros-previousMicros >= 200){ //200 microsecond peroid or 5,000 Hz
//    previousMicros = currentMicros;
//     
//    Xvalue = adc->analogReadContinuous(ADC_0); 
//    Yvalue = adc->analogReadContinuous(ADC_0);   
//    Zvalue = adc->analogRead(ZPin,ADC_0);
//  
    if(!bufferX->isEmpty() && !bufferY->isEmpty() && !bufferZ->isEmpty())
    { 
    accelDataFile = sd.open(AccelFilename, O_RDWR | O_CREAT | O_APPEND);
    accelDataFile.print(currentMicros);
    accelDataFile.print(",");
    sprintf(XaccelString,"% 9d",bufferX->read());
    accelDataFile.print(XaccelString);
    accelDataFile.print(",");
    sprintf(YaccelString,"%9d",bufferY->read());
    accelDataFile.print(YaccelString);
    accelDataFile.print(",");
    sprintf(ZaccelString,"%9d",bufferZ->read());
    accelDataFile.print(ZaccelString);
    accelDataFile.println();
    accelDataFile.close();

    Serial.print(currentMicros);
    Serial.print(XaccelString);
    Serial.print(YaccelString);
    Serial.println(ZaccelString);
  }
     

    /* fail_flag contains all possible errors,
        They are defined in  ADC_Module.h as

        ADC_ERROR_OTHER
        ADC_ERROR_CALIB
        ADC_ERROR_WRONG_PIN
        ADC_ERROR_ANALOG_READ
        ADC_ERROR_COMPARISON
        ADC_ERROR_ANALOG_DIFF_READ
        ADC_ERROR_CONT
        ADC_ERROR_CONT_DIFF
        ADC_ERROR_WRONG_ADC
        ADC_ERROR_SYNCH

        You can compare the value of the flag with those masks to know what's the error.
    */
    if(adc->adc0->fail_flag) {
        Serial.print("ADC0 error flags: 0x");
        Serial.println(adc->adc0->fail_flag, HEX);
    }
    #if defined(ADC_TEENSY_3_1)
    if(adc->adc1->fail_flag) {
        Serial.print("ADC1 error flags: 0x");
        Serial.println(adc->adc1->fail_flag, HEX);
    }
    #endif

 

}

void adc0_isr(void) {
    
    uint8_t pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; 
    Serial.println(pin);
    if (pin==XPin) {
        bufferX->write(adc->analogReadContinuous());
         adc->startContinuous(YPin);
    } 
    else if (pin==YPin)
    {
        bufferY->write(adc->analogReadContinuous());
        adc->startContinuous(ZPin);
        
    } 
    else if (pin==ZPin)
    {
        bufferZ->write(adc->analogReadContinuous());
        adc->startContinuous(XPin);
        
    } 
    else 
    { // clear interrupt anyway
        adc->readSingle();
    }
}



void dateTime(uint16_t* date, uint16_t* time) {
  // User gets date and time from GPS or real-time
  // clock in real callback function

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}


// RESULTS OF THE TEST Teesny 3.x
// Measure continuously a voltage divider.
// Measurement pin A9 (23). Clock speed 96 Mhz, bus speed 48 MHz.

//
//  Using ADC_LOW_SPEED (same as ADC_VERY_LOW_SPEED) for sampling and conversion speeds
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits            81 kHz                               1
//     12  bits            94 kHz                               1
//     10  bits            94 kHz                               1
//      8  bits           103 kHz                               1

//     16  bits             2.5 kHz                              32
//     12  bits             2.9 kHz                              32
//     10  bits             2.9 kHz                              32
//      8  bits             3.2 kHz                              32

//
//  Using ADC_MED_SPEED for sampling and conversion speeds
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits           193 kHz                               1
//     12  bits           231 kHz                               1
//     10  bits           231 kHz                               1
//      8  bits           261 kHz                               1

//     10  bits            58 kHz                               4 (default settings) corresponds to about 17.24 us

//
//  Using ADC_HIGH_SPEED (same as ADC_HIGH_SPEED_16BITS) for sampling and conversion speeds
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits           414 kHz                               1
//     12  bits           500 kHz                               1
//     10  bits           500 kHz                               1
//      8  bits           571 kHz                               1
//
//      8  bits           308 kHz                               1 ADC_VERY_LOW_SPEED sampling
//      8  bits           387 kHz                               1 ADC_LOW_SPEED sampling
//      8  bits           480 kHz                               1 ADC_MED_SPEED sampling
//      8  bits           632 kHz                               1 ADC_VERY_HIGH_SPEED sampling

//
//  Using ADC_VERY_HIGH_SPEED for sampling and conversion speeds
//  This conversion speed is over the limit of the specs! (speed=24MHz, limit = 18 MHz for res<16 and 12 for res=16)
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits           888 kHz                               1
//     12  bits          1090 kHz                               1
//     10  bits          1090 kHz                               1
//      8  bits          1262 kHz                               1


// At 96 Mhz (bus at 48 MHz), 632 KHz is the fastest we can do within the specs, and only if the sample's impedance is low enough.


// RESULTS OF THE TEST Teensy LC
// Measure continuously a voltage divider.
// Measurement pin A9 (23). Clock speed 48 Mhz, bus speed 24 MHz.

//
//  Using ADC_VERY_LOW_SPEED for sampling and conversion speeds
// ADC resolution     Measurement frequency                 Num. averages
//     16  bits            33.3 kHz                               1
//     12  bits            37.5 kHz                               1
//     10  bits            37.5 kHz                               1
//      8  bits            40.5 kHz                               1

//     16  bits             1.04 kHz                             32
//     12  bits             1.2 kHz                              32
//     10  bits             1.2 kHz                              32
//      8  bits             1.3 kHz                              32

//
//  ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED and ADC_VERY_HIGH_SPEED are the same for Teensy 3.x and LC,
//  except for a very small ammount that depends on the bus speed and not on the ADC clock (which is the same for those speeds).
//  This difference corresponds to 5 bus clock cycles, which is about 0.1 us.
//
//  For 8 bits resolution, 1 average, ADC_MED_SPEED sampling speed the measurement frequencies for the different ADACK are:
//  ADC_ADACK_2_4       106.8 kHz
//  ADC_ADACK_4_0       162.6 kHz
//  ADC_ADACK_5_2       235.1 kHz
//  ADC_ADACK_6_2       263.3 kHz
//  For Teensy 3.x the results are similar but not identical for two reasons: the bus clock plays a small role in the total time and
//  the frequency of this ADACK clock is acually quite variable, the values are the typical ones, but in the electrical datasheet
//  it says that they can range from +-50% their values aproximately, so every Teensy can have different frequencies.
