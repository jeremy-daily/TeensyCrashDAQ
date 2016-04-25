/*
 * TeensyTimeGPS
 Borrowed heavily from TinyGPS Examples
 * example code illustrating time synced from a GPS
 * Modified by Jeremy Daily for the Teensy CrashDAQ
 */

#include <TimeLib.h>        // Imports Time Libraries
#include <TinyGPS.h>       // http://arduiniana.org/libraries/TinyGPS/


#include <SPI.h>
#include "SdFat.h"
#include "FreeStack.h"

SdFat SD;

File GPSdataFile;
char GPSfilename[29];

    
#define SDchipSelect 15 //chip select for SD Card

#define SerialGPS Serial1 //Sets up the GPS serial port for the Teensy
#define timeZoneOffset -5 //Central Daylight Time


TinyGPS gps; //create a gps instance

//Time variables for GPS
float flat, flon;
int32_t lat, lon;
time_t t;  
time_t startTime;
time_t previousSetTime = 0;

uint32_t currentMicros,previousMicros;
uint32_t microDifference;
uint8_t numClockSetAttempts=0;

boolean SDOK = false;
boolean LEDState = true;

unsigned long age, date, time, chars = 0;
unsigned short sentences = 0, failed = 0;

double LONDON_LAT = 51.508131, LONDON_LON = -0.128002; //These are the start points
 
time_t getTeensy3Time(){
  return Teensy3Clock.get();
}


time_t setRTCwithGPS(int timeout){
  while (millis() < timeout && numClockSetAttempts < 5){
  // Do this while loop to look for a GPS signal until timeout.
    while (SerialGPS.available()) {
      if (gps.encode(SerialGPS.read())) { // process gps messages
        // when TinyGPS reports new data...
        int year;
        byte month, day, hour, minute, second, hundredths;
        unsigned long age;
        gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ", month, day, year, hour, minute, second);
        Serial.print(sz);  
        setTime(hour, minute, second, day, month, year);
        adjustTime(timeZoneOffset * SECS_PER_HOUR);
        if (age == TinyGPS::GPS_INVALID_AGE){
          Serial.println("No fix detected");
        }
        else if (age > 5000){
          Serial.println("Warning: possible stale data!");
        }
        else if (age < 500){
          Serial.println("Data is current. Checking Validity...");
          t = now();
          
          if (t - previousSetTime < 2) {
            
            Teensy3Clock.set(t); // set the RTC
            Serial.print("Two readings are constistent. RTC and system time is set from GPS: ");
            Serial.println(digitalClockDisplay());
            
            return(t);
          }
          else 
          {
            previousSetTime = t;
            Serial.println("Previous time is different than current time by more than 2.");
            numClockSetAttempts +=1;
          }
        }
        else
        {
          Serial.println("Something went wrong with setting GPS Code...");
        }
      }
    }
  }
  // Do these things if the while loop timed out. 
  //get the time from the RTC and return it
  Serial.print("GPS timed out, setting data from RTC: ");
  t = getTeensy3Time();
  setTime(t);
  Serial.println(digitalClockDisplay());
  return t;
}


void setup(){
  pinMode(SDchipSelect,OUTPUT);
  digitalWrite(SDchipSelect,HIGH);
  
  pinMode(A3,OUTPUT); //led
  digitalWrite(A3,HIGH);
  
  Serial.begin(115200);
  delay(500);
  //while (!Serial) ; // Needed to see the beginning serial comms
  
  //Connect to GPS
  SerialGPS.begin(9600); //Default
  while(!SerialGPS);
  SerialGPS.println(F("$PMTK251,38400*27"));  // set to 38400 baud
  SerialGPS.end();
  
  SerialGPS.begin(38400); //Connect to GPS at higher rate.
  while(!SerialGPS);
  SerialGPS.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));     // RMC and GGA
  SerialGPS.println(F("$PMTK220,100*2F"));     // set to 10 Hz
  
  //Connect to GPS and try to set the system clock.
  startTime = setRTCwithGPS(10000);

  gps.f_get_position(&flat, &flon, &age);
  LONDON_LAT = flat;
  LONDON_LON = flon;
  
  //Create new data files.
   Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (SD.begin(SDchipSelect)) {
    SDOK = true;
    Serial.println("SD card initialized.");
    Serial.println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum");
    Serial.println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail");
    
    sprintf(GPSfilename, "%04d-%02d-%02d_%02d%02d%02d_GPSlog.csv", year(),month(),day(),hour(),minute(),second()); 
    File GPSdataFile = SD.open(GPSfilename, O_RDWR | O_CREAT | O_APPEND);
    GPSdataFile.println("Time,Speed,Time Diff,Sats,HDOP,Latitude,Longitude,Fix,Date,Time,Date Age,Altitude,Course,Cardinal,Dist. from Start,Course From Start,RX Chars,RX Sentences,Checksum");
    GPSdataFile.println("(sec),(mph),(microsec),(count),(-),(microdegees),(microdegrees),(milliseconds),(MM/DD/YYYY),(HH:MM:SS.ss),(milliseconds),(meters),(degres),(-),(meters),(degrees),(count),(count),(0=Pass/1=Fail)");
    GPSdataFile.close();
  }
  else 
  {
    SDOK = false;
    Serial.println("SD Card failed, or not present.");
  }
  
 startTime=now();
 if (SDOK)  digitalWrite(A3,LOW);
}



String digitalClockDisplay(){
  // digital clock display of the time
  
  char clockDisplay[18];
  sprintf(clockDisplay, "%04d-%02d-%02d_%02d%02d%02d", year(),month(),day(),hour(),minute(),second());
  return clockDisplay;
}


void loop()
{
  while (Serial1.available())
  {
     gps.encode(Serial1.read());
  }
  
  currentMicros = micros() ;
  microDifference = currentMicros - previousMicros;
  if (microDifference >= 100000){
    previousMicros = currentMicros;
    gps.get_position(&lat, &lon, &age);
    gps.stats(&chars, &sentences, &failed);
    
    if (!SDOK) //Blink the LED fast if the SD Card ISNT working.
    { 
       LEDState = !LEDState;
      digitalWrite(A3,LEDState);
    }
    
    if (gps.f_speed_mph() > 1.0) 
    {
      writeGPStoSD();
      SerialPrintGPS();
    }
  }
  
 
  
  
}

void writeGPStoSD(){
  
  File GPSdataFile = SD.open(GPSfilename,  O_RDWR | O_CREAT | O_APPEND);
  
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  char TimeDateDisp[32];
  sprintf(TimeDateDisp, "\"\"%02d/%02d/%02d,\"\"%02d:%02d:%02d.%02d,",
        month, day, year, hour, minute, second, hundredths);
        
  setTime(hour, minute, second, day, month, year);
  adjustTime(timeZoneOffset * SECS_PER_HOUR);
  
  GPSdataFile.print(now()-startTime);
  GPSdataFile.print('.');
  GPSdataFile.print(hundredths);
  GPSdataFile.print(',');
  GPSdataFile.print(gps.f_speed_mph());
  GPSdataFile.print(',');
  GPSdataFile.print(microDifference);
  GPSdataFile.print(',');
  GPSdataFile.print(gps.satellites());
  GPSdataFile.print(',');
  GPSdataFile.print(gps.hdop());
  GPSdataFile.print(',');
  GPSdataFile.print(lat);
  GPSdataFile.print(',');
  GPSdataFile.print(lon);
  GPSdataFile.print(',');
  GPSdataFile.print(age);
  GPSdataFile.print(',');
  GPSdataFile.print(TimeDateDisp);
  GPSdataFile.print(age);
  GPSdataFile.print(',');
  GPSdataFile.print(gps.f_altitude());
  GPSdataFile.print(',');
  GPSdataFile.print(gps.f_course());
  GPSdataFile.print(',');
  GPSdataFile.print(TinyGPS::cardinal(gps.f_course()));
  GPSdataFile.print(',');
  GPSdataFile.print((unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) );
  GPSdataFile.print(',');
  GPSdataFile.print(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON));
  GPSdataFile.print(',');
  GPSdataFile.print(chars);
  GPSdataFile.print(',');
  GPSdataFile.print(sentences);
  GPSdataFile.print(',');
  GPSdataFile.println(failed);
  GPSdataFile.close();
    
}

void SerialPrintGPS(){
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "*** " : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);

  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  Serial.println();
}



static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartdelay(0);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d.%02d ",
        month, day, year, hour, minute, second, hundredths);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartdelay(0);
}
