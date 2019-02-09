// A simple sketch to read GPS data and parse the $GPRMC string 
// see http://www.ladyada.net/make/gpsshield for more info

//20180720: GPS shield 1.0 funktioniert
#include "SD.h"
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial =  SoftwareSerial(2, 3);
//SoftwareSerial mySerial =  SoftwareSerial(2, 6);
#define ledFixPin 6
#define ledWritePin 5
#define powerpin 4
#define modepin 7
#define chipSelect 10

#define GPSRATE 4800
//#define GPSRATE 38400

// power saving modes
#define SLEEPDELAY 10    /* power-down time in seconds. Max 65535. Ignored if TURNOFFGPS == 0 */

const char string_0[] PROGMEM =   "$PSRF100,01,4800,08,01,00*0E\r\n";
const char string_1[] PROGMEM =   "$PSRF103,00,00,01,01*25\r\n";
const char string_2[] PROGMEM =   "$PSRF103,00,00,00,01*24\r\n";

const char* const string_table[] PROGMEM = {string_0, string_1, string_2};

//#define SERIAL_SET   "$PSRF100,01,4800,08,01,00*0E\r\n"

// GGA-Global Positioning System Fixed Data, message 103,00
#define LOG_GGA 1
//#define GGA_ON   "$PSRF103,00,00,01,01*25\r\n"
//#define GGA_OFF  "$PSRF103,00,00,00,01*24\r\n"

// GLL-Geographic Position-Latitude/Longitude, message 103,01
#define LOG_GLL 1
#define GLL_ON   "$PSRF103,01,00,01,01*26\r\n"
#define GLL_OFF  "$PSRF103,01,00,00,01*27\r\n"

// GSA-GNSS DOP and Active Satellites, message 103,02
#define LOG_GSA 1
#define GSA_ON   "$PSRF103,02,00,01,01*27\r\n"
#define GSA_OFF  "$PSRF103,02,00,00,01*26\r\n"

// GSV-GNSS Satellites in View, message 103,03
#define LOG_GSV 1
#define GSV_ON   "$PSRF103,03,00,01,01*26\r\n"
#define GSV_OFF  "$PSRF103,03,00,00,01*27\r\n"

// RMC-Recommended Minimum Specific GNSS Data, message 103,04
#define LOG_RMC 1
#define RMC_ON   "$PSRF103,04,00,01,01*21\r\n"
#define RMC_OFF  "$PSRF103,04,00,00,01*20\r\n"

// VTG-Course Over Ground and Ground Speed, message 103,05
#define LOG_VTG 1
#define VTG_ON   "$PSRF103,05,00,01,01*20\r\n"
#define VTG_OFF  "$PSRF103,05,00,00,01*21\r\n"

// Switch Development Data Messages On/Off, message 105
#define LOG_DDM 1
#define DDM_ON   "$PSRF105,01*3E\r\n"
#define DDM_OFF  "$PSRF105,00*3F\r\n"

#define USE_WAAS   0     // useful in US, but slower fix
#define WAAS_ON    "$PSRF151,01*3F\r\n"       // the command for turning on WAAS
#define WAAS_OFF   "$PSRF151,00*3E\r\n"       // the command for turning off WAAS


// GPS parser for 406a
#define BUFFSIZ 90 // plenty big
char buffer[BUFFSIZ];
char fnamebuf[15];
char sleepdelaybuf[10];
char *parseptr;
char buffidx;
bool fix = false; // current fix data
bool lineread=false;
uint8_t hour, minute, second, year, month, date;
uint32_t latitude, longitude,lonl,lonr,latr,latl;;
uint8_t groundspeed, trackangle;
char latdir, longdir;
char status;
int sleepdelay=SLEEPDELAY;
bool ijustwokeup=true;

File f;

uint8_t i;

byte modeState=0;
unsigned long previousMillis = 0; 
byte ledState = LOW;             // ledState used to set the LED
const long interval = 500;           // interval at which to blink (milliseconds)
char answer[26];  
int pos=0;  
bool commandvalid=false;   

// blink out an error code
void error(uint8_t errno) {
  while(1) {
    for (i=0; i<errno; i++) {
      digitalWrite(ledFixPin, HIGH);
      digitalWrite(ledWritePin, HIGH);
      delay(100);
      digitalWrite(ledFixPin, LOW);
      digitalWrite(ledWritePin, LOW);
      delay(100);
    }
    for (; i<10; i++) {
      delay(200);
    }
  }
}

void setup() 
{ 
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = 0;

  Serial.begin(9600);

  pinMode(modepin, INPUT);
  pinMode(ledFixPin, OUTPUT);
  pinMode(ledWritePin, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card init. failed!");
    error(1);
  }
  modeState = HIGH;//digitalRead(modepin);
  if(modeState==LOW)
  {
   Serial.println("logger mode");
   if (powerpin) {
      pinMode(powerpin, OUTPUT);
    }
  
    mySerial.begin(GPSRATE);
  
    // prints title with ending line break 
    Serial.println("GPS parser"); 
   
     digitalWrite(powerpin, LOW);         // pull low to turn on!
  
    delay(250);
    
    if (SD.exists("POWERSAV.TXT")) {
      Serial.println("found config!");
      f = SD.open("POWERSAV.TXT", FILE_READ);
      if (!f) {
        Serial.println("error opening config");
        error(6);
      }
      else
      {
        Serial.print("reading ... ");
        int i=0;
        int rtn;
        while((i<9)&&( (rtn = f.read((uint8_t*) sleepdelaybuf, 9-i)) > 0))
        {
          Serial.print(sleepdelaybuf);
          i+=rtn;
        }    
        sleepdelaybuf[i]=0;
        Serial.println();
        Serial.print("value: ");
        Serial.print(sleepdelaybuf);
        Serial.println(" read!");
        sscanf(sleepdelaybuf,"%d",&sleepdelay);
        Serial.print("sleepdelay: ");
        Serial.println(sleepdelay,DEC);
        f.close();          
      }
    }
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[0])));
    mySerial.println(buffer);
    delay(250);
  
    if ((LOG_DDM)&&(!sleepdelay))
      mySerial.println(DDM_ON);
    else
      mySerial.println(DDM_OFF);
    delay(250);
  
    if ((LOG_GGA)&&(!sleepdelay))
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[1])));
      mySerial.println(buffer);
    }
    else
    {
      strcpy_P(buffer, (char*)pgm_read_word(&(string_table[2])));
      mySerial.println(buffer);
    }
    delay(250);
  
    if ((LOG_GLL)&&(!sleepdelay))
      mySerial.println(GLL_ON);
    else
      mySerial.println(GLL_OFF);
    delay(250);
  
    if ((LOG_GSA)&&(!sleepdelay))
      mySerial.println(GSA_ON);
    else
      mySerial.println(GSA_OFF);
    delay(250);
  
    if ((LOG_GSV)&&(!sleepdelay))
      mySerial.println(GSV_ON);
    else
      mySerial.println(GSV_OFF);
    delay(250);
  
    if ((LOG_RMC)||(sleepdelay))
      mySerial.println(RMC_ON);
    else
      mySerial.println(RMC_OFF);
    delay(250);
  
    if ((LOG_VTG)&&(!sleepdelay))
      mySerial.println(VTG_ON);
    else
      mySerial.println(VTG_OFF);
    delay(250);
  
    if ((USE_WAAS)&&(!sleepdelay))
      mySerial.println(WAAS_ON);
    else
      mySerial.println(WAAS_OFF);
  
    Serial.println("ready!");
  }
  else
  {
    Serial.println("interactive mode");
    answer[0]=0;
  }
} 
 
 
void loop() 
{ 
  uint32_t tmp;
  if(modeState==LOW)
  {
    //logger mode
  //  Serial.print("\r\nread: ");
  //  Serial.print("\r\n");
    readline();
    lineread=!lineread;
    
    if((fix)||(lineread))
    {
      digitalWrite(ledFixPin, HIGH);
    }
    else
    {
      digitalWrite(ledFixPin, LOW);
    }
    // check if $GPRMC (global positioning fixed data)
    if (strncmp(buffer, "$GPRMC",6) == 0) {
      // find out if we got a fix
      char *p = buffer;
      p = strchr(p, ',')+1;
      p = strchr(p, ',')+1;       // skip to 3rd item
  
      if (p[0] == 'V') {
        fix = 0;
      } else {
        fix = 1;
      }
      if (sleepdelay) {
        if (!fix) {
          Serial.print('_');
          buffidx = 0;
          return;
        }
      }
  
      // hhmmss time data
      parseptr = buffer+7;
      tmp = parsedecimal(parseptr); 
      hour = tmp / 10000;
      minute = (tmp / 100) % 100;
      second = tmp % 100;
      
      parseptr = strchr(parseptr, ',') + 1;
      status = parseptr[0];
      parseptr += 2;
      
      // grab latitude & long data
      // latitude
      latitude = parsedecimal(parseptr);
      latl=latitude;
      if (latitude != 0) {
        latitude *= 10000;
        parseptr = strchr(parseptr, '.')+1;
        latr=parsedecimal(parseptr);
        latitude += latr;
      }
      parseptr = strchr(parseptr, ',') + 1;
      // read latitude N/S data
      if (parseptr[0] != ',') {
        latdir = parseptr[0];
      }
  
      //Serial.println(latdir);
  
      // longitude
      parseptr = strchr(parseptr, ',')+1;
      longitude = parsedecimal(parseptr);
      lonl=longitude;
      if (longitude != 0) {
        longitude *= 10000;
        parseptr = strchr(parseptr, '.')+1;
        lonr = parsedecimal(parseptr);
        longitude+=lonr;
      }
      parseptr = strchr(parseptr, ',')+1;
      // read longitude E/W data
      if (parseptr[0] != ',') {
        longdir = parseptr[0];
      }
  
      // groundspeed
      parseptr = strchr(parseptr, ',')+1;
      groundspeed = parsedecimal(parseptr);
  
      // track angle
      parseptr = strchr(parseptr, ',')+1;
      trackangle = parsedecimal(parseptr);
  
  
      // date
      parseptr = strchr(parseptr, ',')+1;
      tmp = parsedecimal(parseptr); 
      date = tmp / 10000;
      month = (tmp / 100) % 100;
      year = tmp % 100;
      
  /*    Serial.print("\nTime: ");
      Serial.print(hour, DEC); Serial.print(':');
      Serial.print(minute, DEC); Serial.print(':');
      Serial.println(second, DEC);
      Serial.print("Date: ");
      Serial.print(month, DEC); Serial.print('/');
      Serial.print(date, DEC); Serial.print('/');
      Serial.println(year, DEC);
  */    
      if(!f)
      {
        if(((year!=0)&&(month!=0))&&(date!=0))
        {
        Serial.println();
        sprintf(fnamebuf, "%02d%02d%02d00.TXT",year,month,date);
        for (i = 0; i < 100; i++) {
            fnamebuf[6] = '0' + i/10;
            fnamebuf[7] = '0' + i%10;
            Serial.print("trying ");
            Serial.println(fnamebuf);
            // create if does not exist, do not open existing, write, sync after write
            if (! SD.exists(fnamebuf)) {
              break;
            }
        }
  
        f = SD.open(fnamebuf, FILE_WRITE);
        if (!f) {
          Serial.println("error opening ");
          Serial.println(fnamebuf);
          error(6);
        }
        Serial.println("writing to ");
        Serial.println(fnamebuf);
        }
      }
      
  //    Serial.print("Lat  (raw): "); 
  //    Serial.println(latitude, DEC);
  //    Serial.println(String(latl/100,DEC)+' '+String(latl%100,DEC)+','+String(latr,DEC)+' '+latdir);
  //    Serial.print("Long (raw): ");
  //    Serial.println(longitude, DEC);
  
  /*    Serial.print("Lat: "); 
      if (latdir == 'N')
         Serial.print('+');
      else if (latdir == 'S')
         Serial.print('-');
  
      Serial.print((latitude/1000000), DEC); 
      //Serial.print('°', DEC);
      Serial.print(' ');
      Serial.print((latitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
      Serial.print((latitude%10000)*6/1000, DEC); Serial.print('.');
      Serial.print(((latitude%10000)*6/10)%100, DEC); Serial.print('" ');Serial.println(latdir);
     
      Serial.print("Long: ");
      if (longdir == 'E')
         Serial.print('+');
      else if (longdir == 'W')
         Serial.print('-');
      Serial.print((longitude/1000000), DEC); 
      //Serial.print('°', DEC);
      Serial.print(' ');
      Serial.print((longitude/10000)%100, DEC); Serial.print('\''); Serial.print(' ');
      Serial.print((longitude%10000)*6/1000, DEC); Serial.print('.');
      Serial.print(((longitude%10000)*6/10)%100, DEC); Serial.print('" ');Serial.println(longdir);
  */   
    }
    //Serial.println(buffer);
    if(f)
    {
      if(((sleepdelay==0)||(fix))&&(!ijustwokeup))
      {
        if(buffidx>1)
        {
          digitalWrite(ledWritePin, HIGH);      // sets the digital pin as output
          if(f.write((uint8_t *) buffer, buffidx) != buffidx) {
            Serial.println("can't write!");
            error(7);
          }
          f.flush();
          digitalWrite(ledWritePin, LOW);
        }
      }
      if(ijustwokeup)
        ijustwokeup=false;
    }
    if ((fix)&&(!ijustwokeup))
    {  //(don't sleep if there's no fix)
    
      if (sleepdelay)
      {      // turn off GPS module? 
        Serial.print("\r\ngoing to sleep for ");
        Serial.print(sleepdelay, DEC);
        Serial.println(" seconds!");
        digitalWrite(ledFixPin, LOW);
        digitalWrite(ledWritePin, LOW);
    
        digitalWrite(powerpin, HIGH);  //turn off GPS
  
        delay(100);  //wait for serial monitor write to finish
        delay(sleepdelay*1000);  //turn off CPU
  
        digitalWrite(powerpin, LOW);  //turn on GPS
        ijustwokeup=true;
  //      Serial.println("woke up after sleep!");
      } //if (sleepdelay) 
      //fix=false;
    } //if (fix)
  }
  else
  {
    //interactive mode
    unsigned long currentMillis = millis();
  
    if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
  
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
      digitalWrite(ledFixPin, ledState==HIGH?HIGH:LOW);
      digitalWrite(ledWritePin, ledState==HIGH?LOW:HIGH);
    }
    while(Serial.available())
    {
      char inChar=Serial.read();
      answer[pos]=inChar;
      pos++;
      answer[pos]=0;
      if(pos==25)
      {
        pos=0;
        break;
      }
      if(answer[pos-1]=='\n')
      {
        commandvalid=true;
        answer[pos-1]=0;
        break;
      }
    }
    if(commandvalid!=false)
    {
      commandvalid=false;
      for(int x=0;x<26;x++)
      {
        if(answer[x]==0)
        {
          if(strncmp(answer,"ls",2)==0)
          {
            File root = SD.open("/");
            printDirectory(root, 0,false,false);
            break;
          }
          else if(strncmp(answer,"lsr",3)==0)
          {
            File root = SD.open("/");
            printDirectory(root, 0,false,true);
            break;
          }
          else if(strncmp(answer,"dir",3)==0)
          {
            File root = SD.open("/");
            printDirectory(root, 0,true,false);
            break;
          }
          else if(strncmp(answer,"fmt",3)==0)
          {
            File root = SD.open("/");
            while (true) 
            {
              File entry =  root.openNextFile();
              if (! entry) 
              {
                // no more files
                break;
              }
              if((entry.name()[0]>='0')&&(entry.name()[0]<='9'))
              {
                Serial.print("deleting: ");
                Serial.print(entry.name());
              }
              entry.close();
              break;
            }
          }
          else if(strncmp(answer,"del ",3)==0)
          {
            if(SD.exists(answer+4)) 
            {
              SD.remove(answer+4);
            }
            else
            {
              Serial.print("not found: ");
              Serial.println(answer+4);
            }
            break;
          }
          else if(strncmp(answer,"cat ",3)==0)
          {
            if(SD.exists(answer+4)) 
            {
              File dataFile = SD.open(answer+4);
              // if the file is available, write to it:
              if (dataFile) {
                while (dataFile.available()) {
                  Serial.write(dataFile.read());
                }
                dataFile.close();
              }
            }
            else
            {
              Serial.print("not found: ");
              Serial.println(answer+4);
            }
            break;
          }
          else
          {
            Serial.print("unknown: ");
            Serial.println(answer);
            break;
          }
        }
      }
      answer[pos]=0;
      pos=0;
      Serial.println("$P");
    }
  }
}
void printDirectory(File dir, int numTabs,boolean full,boolean recursive) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
/*    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
*/    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      if(recursive==true)
      {
        printDirectory(entry, numTabs + 1,full,recursive);
      }
    } else {
      if(full==true)
      {
        // files have sizes, directories do not
        Serial.print("\t\t");
        Serial.print(entry.size(), DEC);
      }
      Serial.println();
    }
    entry.close();
  }
}

uint32_t parsedecimal(char *str) {
  uint32_t d = 0;
  
  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     return d;
   d *= 10;
   d += str[0] - '0';
   str++;
  }
  return d;
}

void readline(void) {
  char c;
  
  buffidx = 0; // start at begninning
  while (1) {
      c=mySerial.read();
      if (c == -1)
        continue;
      Serial.print(c);
      if (c == '\n')
        continue;
      if ((buffidx == BUFFSIZ-1) || (c == '\r')) {
        buffer[buffidx++]= '\n';
        buffer[buffidx] = 0;
        return;
      }
      buffer[buffidx++]= c;
  }
}
