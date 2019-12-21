#include<stdio.h>

#include <Adafruit_FONA.h>

#include <OBD9141.h>
//#include <EEPROM.h>
#include <string.h>
#include <PString.h>
#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>


/////////////////////////////////////////////////////////////////////////////////////////////////////
//                  ECU - GPS controller
//                  Ricardo Vargas 
/////////////////////////////////////////////////////////////////////////////////////////////////////

/* 
  Notes:
  Ported to ATSAMD21 (Feather M0 test board)
*/


// OBD type (CAN / 9141 /KWD) will need to be programmed via software since no resource was allocated in hardware

//Use Serial 1 on Custom Board
#define ISO_TX 1
#define ISO_RX 2

#define LPGSTATUSPIN A0 // A0 analog input
#define LIN_SLP 3 // sleep pin for LIN IC

// pin for FONA
#define FONA_RST 4
#define FONA_TX 5
#define FONA_RX 6
#define FONA_PWRKEY 7

// Set reset for 12V relay

#define PIN_OFF 25 
#define PIN_ON 26

#define SD_CS 13
#define SD_DT 14

/// MACROS

#define  IMAP   0x0B
#define  RPM    0x0C
#define  SPEED  0x0D
#define  IAT    0x0F

#define _8BITS  1
#define _16BITS 2
#define _32BITS 4

#define UTC -6

//////////

// TO DO: implement a method to program these variables through SMS
char deviceID[25] = "5d421ed6c86164002300c000";
char ownerID[25] = "5d3b6bc8865a59002353aa8c";
char serverurl[80] = "https://aire-nuevo.herokuapp.com/api/v1/stats";


struct t  {
  unsigned long tStart;
  unsigned long tTimeout;
};

//Tasks and their Schedules.
t t_func1 = {0, 1000}; //Run every 1000ms - OBD2 query
t t_func2 = {0, 300000}; //Run every 10 minutes - HTTP POSTing
t t_func3 = {0, 10000}; //Run every 10 seconds - SMS Process

// variables
char replybuffer[255]; // this is a large buffer for replies
char message[255]; 
char sendbuffer[141]; // max limit per SMS
char command[5];
char newpass[9];
char readpass[9];
char delim[2] = " "; // delimiter for parsing
// obdflag
// 0: Default (no system selected)
// 1: ISO 9141 (slow)
// 2: KWP (fast 9141)
// 3: CAN Bus
uint8_t obdflag = 2;

unsigned long elapsedmillis = 0;
unsigned long startmillis = 0;
float elapsedtime = 0;

// Hardware serial port to FONA
HardwareSerial *fonaSerial = &Serial1;

// Hardware port for ISO 9141
HardwareSerial *isoSerial = &Serial2;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

int a; // iterator

//create the CANport acqisition schedulers
//cAcquireCAN CANport0(CAN_PORT_0);

// ISO 9141
OBD9141 obd9141;

bool  bFuelType = 0;// 0 for gasoline, 1 for LPG
int   LPGStatusRead = 0; // Analog read from valve
bool init_success = false;
bool postflag = false;
bool lastpost = false; // use to post a final message
bool engineflag = false;

float fSpeed = 0; // Km per hour
float fRPM = 0;   // Revs per minute
float fIMAP = 0;  //Internal Manifold absolute pressure
float fIAT = 0;   //Internal Air Temperature
float fAir = 0;   // Grams of Air (calculated)
float fLPH = 0;   // Liters per Hour
float fKPL = 0;   // Kilometers per Liter

float EngineVol = 1.6; // Needs to be programmed remotely
float VE = 0.85;
float const MAFConst = 0.029025;
float const LPHGasConst = 0.333648;

float fGasLambda = 14.7;
float fLPGLambda = 15.67;
float fGasDens = 734;
float fLPGDens = 536.3;
const float SecPerHour = 3600;
float fGasTime = 0;
float fLPGTime = 0;
float fGasPerc = 0;
float fLPGPerc = 0;
float fAvgGasLPH = 0;
float fAvgGasKPL = 0;
float fAvgLPGLPH = 0;
float fAvgLPGKPL = 0;
float fSumGasLPH = 0;
float fSumGasKPL = 0;
float fSumLPGLPH = 0;
float fSumLPGKPL = 0;
float nSamples = 0;
float fGasLiters = 0;
float fLPGLiters = 0;
float fLPS = 0;

// GPS variables
float lat, lon, speedkph,heading, altitude;
float lastlat,lastlon;  // to track last available coordinates
char gpsdate[20];

//RTC variables
RTCZero rtc;
char timestamp[25];
byte secs,mins,hrs,day,month,year;
        
struct xSave { // saving to EEPROM
  uint8_t init;
  char password[9];
  char adminpass[9];
  bool  relay;

};

xSave EEPROMSave; // instance of the struct

File datafile;
char filename[13] = "log.txt";
bool logopen = false;

// auxilliary functions for task execution
bool tCheck (struct t *t ) {
  if (millis() > (t->tStart + t->tTimeout)) { 
    return true;
  } else {
    return false;
  }
}

void tRun (struct t *t) {
  t->tStart = millis();
}

void printlog (const __FlashStringHelper *line) {
  SerialUSB.print(line);
  if (logopen) {
    datafile.print(line);
  }
}

void printlog (const char *line) {
  SerialUSB.print(line);
  if (logopen) {
    datafile.print(line);
  }
}

void printlogln (const __FlashStringHelper *line) {
  SerialUSB.println(line);
  if (logopen) {
    datafile.println(line);
  }
}

void printlogln (const char *line) {
  SerialUSB.println(line);
  if (logopen) {
    datafile.println(line);
  }
}

void printlog (bool line) {
  SerialUSB.print(line);
  if (logopen) {
    datafile.print(line);
  }
}

void printlogln (bool line) {
  SerialUSB.println(line);
  if (logopen) {
    datafile.println(line);
  }
}

void printlog (int line) {
  SerialUSB.print(line);
  if (logopen) {
    datafile.print(line);
  }
}

void printlogln (int line) {
  SerialUSB.println(line);
  if (logopen) {
    datafile.println(line);
  }
}

void printlog (float line) {
  SerialUSB.print(line);
  if (logopen) {
    datafile.print(line);
  }
}

void printlogln (float line) {
  SerialUSB.println(line);
  if (logopen) {
    datafile.println(line);
  }
}

void printlogln (long unsigned int line) {
  SerialUSB.println(line);
  if (logopen) {
    datafile.println(line);
  }
}
///////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:

  pinMode(PIN_ON, OUTPUT);
  pinMode(PIN_OFF, OUTPUT);
  pinMode(FONA_RST, OUTPUT);

  obdflag = 1;	// HARDCODE FOR NOW
  SerialUSB.begin(115200);
  rtc.begin();
  
    // SD Card setup
   SerialUSB.println(F("Initializing SD card..."));
 

    // see if the card is present and can be initialized:
    if (SD.begin(SD_CS)) {     
      SerialUSB.println(F("SD card initialized"));
      datafile = SD.open(filename, FILE_WRITE);
      logopen = true;
    }
    else {
      SerialUSB.println(F("SD card failed, or not present"));
    }
  printlogln(F("\r\n\r\n**********************"));
  printlogln(F("*****  ECU GPS  ******"));
  printlogln(F("Initializing..."));

  
// Init variables the first run

   //uint8_t EEPROMInit = EEPROM.read(0); // flash bytes will be 255 at first run
   //printlog("EEPROMInit: ");printlogln(EEPROMInit);
uint8_t EEPROMInit = 0;

   if (EEPROMInit == 0) {
     printlogln(F("Writing default values in EEPROM for the first time..."));
     EEPROMInit = 255;
//     EEPROM.write(0,EEPROMInit);
     strcpy(EEPROMSave.password,"87654321\0");
     strcpy(EEPROMSave.adminpass,"qwertyui\0");
     EEPROMSave.relay = true;
     //WriteEEPROM();
   }

/*
   // read average data from EEPROM at startup
   printlogln("Reading data from EEPROM ..");
   uint8_t b[sizeof(xSave)]; // create byte array to store the struct
   for (a = 1; a < sizeof(EEPROMSave)+1; a++) {
     b[a] = EEPROM.read(a);
   }
   memcpy(&EEPROMSave, b, sizeof(EEPROMSave)); // copy byte array to temporary struct
*/
   
   printlog("Password: ");
   printlogln(EEPROMSave.password);
   printlog("Admin Password: ");
   printlogln(EEPROMSave.adminpass);
   printlog("Relay status: ");
   printlogln(EEPROMSave.relay);
  

   if (EEPROMSave.relay) {
      printlogln(F("Checking power relay is ON"));
      digitalWrite(PIN_ON, HIGH);
      digitalWrite(PIN_OFF, LOW);
   } else {
      printlogln(F("Checking power relay is OFF"));
      digitalWrite(PIN_ON, LOW);
      digitalWrite(PIN_OFF, HIGH);
   }
    
  // TO DO: change fona serial baud rate to 115200 as default
   fonaSerial->begin(4800);
   if (! fona.begin(*fonaSerial)) {
     printlogln(F("Couldn't find FONA"));
     while (1);
   }
   type = fona.type();
   printlogln(F("FONA is found OK"));

   // Print module IMEI number.
   char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
   uint8_t imeiLen = fona.getIMEI(imei);
   if (imeiLen > 0) {
      printlog(F("Module IMEI: ")); printlogln(imei);
   }
   fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
   printlog(F("SIM CCID = ")); printlogln(replybuffer);
   
   // Configure a GPRS APN, username, and password.
   fona.setGPRSNetworkSettings(F("internet.movistar.cr"), F("movistarcr"), F("movistarcr"));

   //configure HTTP gets to follow redirects over SSL
   fona.setHTTPSRedirect(true);

   // Turn on GPS
   if (!fona.enableGPS(true))
      printlogln(F("Failed to turn on GPS!"));
   else
      printlogln(F("GPS Enabled"));

   printlogln(F("Wait 2 seconds to start main loop..."));
   delay (2000);
   // TO DO: change startup method based on jumper configuration

     if (obdflag == 3) {
         //start CAN ports,  enable interrupts and RX masks, set the baud rate here
         printlogln(F("OBD Port Type: CAN"));
         //CANport0.initialize(_500K)
      }
      else if (obdflag == 2) {
        printlogln(F("OBD Port Type: ISO 9141 fast (KWP)"));
        obd9141.begin(Serial1, ISO_RX, ISO_TX);
        init_success =  obd9141.initKWP(); // Aveo uses KWP
        digitalWrite(LIN_SLP, HIGH);
        printlog("OBD2 init success:"); printlogln(init_success);
      } else if (obdflag == 1) {
        printlogln(F("OBD Port Type: ISO 9141 slow"));
        obd9141.begin(Serial1, ISO_RX, ISO_TX);
        digitalWrite(LIN_SLP, HIGH);
        init_success =  obd9141.init(); // crossfox uses 50 baud init
        printlog("OBD2 init success:"); printlogln(init_success);
      }
   // Time from RTC
   rtctimestamp(timestamp);
   printlog(F("Current Time: ")); printlogln(timestamp);
   
   startmillis = millis(); // discount setup time from delta.
   printlogln(F("*****  Init End  ******\r\n"));
   datafile.close();
   logopen = false;
}

////////////////////////////////////////////////////////////////////////////////
void loop() {
  // put your main code here, to run repeatedly:
  // poll the CAN port
  //if (obdflag == 0) {
  //  CANport0.run(POLLING);
  //}
  // start the main function at the interval
  if (tCheck(&t_func1)) {
    ReadOBD();
    tRun(&t_func1);
  }

  if (tCheck(&t_func2)) {
   // WriteEEPROM();
    HTTPPost();
    tRun(&t_func2);
  } 

   if (tCheck(&t_func3)) {
    ProcessSMS();
    tRun(&t_func3);
  } 
}

void ReadOBD() {
  datafile = SD.open(filename, FILE_WRITE);
  if (datafile) {
    logopen = true;
  }
  else {
    SerialUSB.println(F("SD file open failed, try to init SD"));
    // see if the card is present and can be initialized:
    if (SD.begin(SD_CS)) {     
      SerialUSB.println(F("SD card initialized"));
      datafile = SD.open(filename, FILE_WRITE);
      logopen = true;
    }
    else {
      SerialUSB.println(F("SD card failed, or not present"));
    }
  } //if (datafile)
    
  elapsedmillis = millis() - startmillis; // uses startmillis from previous cycle
  startmillis = millis();
  printlog(F("** Elapsed Milliseconds: **")); printlogln(elapsedmillis);
  
  // Read analog pin 
  LPGStatusRead = analogRead(LPGSTATUSPIN);
  printlog(F("LPGStatusRead = ")); printlogln(LPGStatusRead);
  if (LPGStatusRead > 512) {
    // LPG is on
    bFuelType = true;
  } else {
    //LPG is off
    bFuelType = false;
  }

  // Read OBD data based on type
  if (obdflag == 3) {
    // CAN BUS TO BE IMPLEMENTED WITH NEW CONTROLLER
    /*
    // Get the OBD2 data on CAN
    fSpeed = CAN_Speed.getData();
    printlog(CAN_Speed.getName());
    printlog(fSpeed);
    printlogln(CAN_Speed.getUnits());
  
    fRPM = CAN_EngineSpeed.getData();
    printlog(CAN_EngineSpeed.getName());
    printlog(fRPM);
    printlogln(CAN_EngineSpeed.getUnits());
  
    fIMAP = CAN_IMAP.getData();
    printlog(CAN_IMAP.getName());
    printlog(fIMAP);
    printlogln(CAN_IMAP.getUnits());
  
    fMAF = CAN_MAF.getData();
    printlog(CAN_MAF.getName());
    printlog(fMAF);
    printlogln(CAN_MAF.getUnits());
    // END Get the OBD2 CAN data
    */
  }
  else {
    // ISO 9141
    
    if (!init_success) {
      if (obdflag == 2) {
         printlog(F("KWP Init - "));
         init_success =  obd9141.initKWP(); // Aveo uses KWP baud init
      }
      else if (obdflag == 1) {
         printlog(F("50 baud Init - "));
         init_success =  obd9141.init(); // crossfox uses 50 baud init
      }
      printlog(F("OBD2 init success: ")); printlogln(init_success);
    }
    
     bool res;
     res = obd9141.getCurrentPID(RPM,_16BITS);
     if (!res) {
        printlogln(F("** RPM reading invalid - redoing init"));
        if (obdflag == 2) {
           printlog(F("KWP Init - "));
           init_success =  obd9141.initKWP(); // Aveo uses KWP baud init
        }
        else if (obdflag == 1) {
           printlog(F("50 baud Init - "));
           init_success =  obd9141.init(); // crossfox uses 50 baud init
        }
        printlog(F("OBD2 init success: ")); printlogln(init_success);
        if (!init_success) {
          printlogln(F("Vehicle ECU must be off"));
          lastpost = true;
          engineflag = false;
          return;
        } else {
          res = obd9141.getCurrentPID(RPM,_16BITS);
          if (!res) {
            printlogln(F("RPM reading still not valid"));
            return;
          }
        }
     } 
      
     
        fRPM = obd9141.readUint16()/4;
        if (fRPM == 0) {
          engineflag = false; // Engine off
        } else {
          engineflag = true;
          postflag = true; // only do a fuel calculation and HTTP post if the engine is on
        }
        printlog("RPM: ");
        printlog(fRPM);
        
       delay(100);
       
       res = obd9141.getCurrentPID(SPEED,_8BITS);
       if (res) {
          fSpeed = obd9141.readUint8();
          printlog(", Speed: ");
          printlog(fSpeed);
       }
       delay(100);
       
       res = obd9141.getCurrentPID(IMAP,_8BITS);
       if (res) {
          fIMAP = obd9141.readUint8();
          printlog(", IMAP: ");
          printlog(fIMAP);
       }
       delay(100);
       res = obd9141.getCurrentPID(IAT,_8BITS);
       if (res) {
          fIAT = obd9141.readUint8() - 40;
          printlog(", IAT: ");
          printlog(fIAT);
       }


  } // if (obdflag == 3)
  if (! engineflag) {
    printlogln("Engine is turned off - skipping calculation");
  }
  else {
    fAir = fRPM * fIMAP / (fIAT+273) * EngineVol * VE * MAFConst;
    printlog(", Air Gr:" ); printlogln(fAir);
  
    // Calculate consumption and KPL
    nSamples ++;
    if (bFuelType) {
      // LPG
      fLPH = (fAir * SecPerHour) / (fLPGLambda * fLPGDens);
      fKPL = fSpeed / fLPH;
      if (fLPH != 0 ) {
        fKPL = fSpeed / fLPH;
      }
      else {
        fKPL = 0;
      }
      fLPS = fAir / (fLPGLambda * fLPGDens);
      fSumLPGLPH = fSumLPGLPH + fLPH;
      fSumLPGKPL = fSumLPGKPL + fKPL;
      fAvgLPGLPH = fSumLPGLPH / nSamples;
      fAvgLPGKPL = fSumLPGKPL / nSamples;
      fLPGTime += (float)elapsedmillis/1000; // elapsed time in seconds
      fLPGLiters += fLPS * fLPGTime;
    } 
    else {
      // Gasoline
      fLPS = fAir / (fGasLambda * fGasDens);
      fLPH = (fAir * SecPerHour) / (fGasLambda * fGasDens);
      if (fLPH != 0 ) {
        fKPL = fSpeed / fLPH;
      }
      else {
        fKPL = 0;
      }
      fSumGasLPH = fSumGasLPH + fLPH;
      fSumGasKPL = fSumGasKPL + fKPL;
      fAvgGasLPH = fSumGasLPH / nSamples;
      fAvgGasKPL = fSumGasKPL / nSamples;
      fGasTime += (float)elapsedmillis/1000; // elapsed time in seconds
      fGasLiters += fLPS * fGasTime;
    }
    fGasPerc = 100* fGasTime / (fGasTime + fLPGTime);
    fLPGPerc = 100* fLPGTime / (fGasTime + fLPGTime);
    
    printlog("Fuel Type (TRUE = LPG, FALSE = Gasoline): ");
    printlogln(bFuelType);
    printlog("Liters/Hour: ");
    printlog(fLPH);
    printlog(", KM/Liter: ");
    printlog(fKPL);
    printlog(", Fuel LPS: ");
    printlogln(fLPS); 
    printlog("Gasoline time: ");
    printlogln(fGasTime);
    printlog("LPG Time: ");
    printlogln(fLPGTime);
    printlog("Gasoline %: ");
    printlogln(fGasPerc);
    printlog("LPG %: ");
    printlogln(fLPGPerc);
    printlog("Cumulative Gasoline liters: ");
    printlogln(fGasLiters);
    printlog("Cumulative LPG liters: ");
    printlogln(fLPGLiters);
    printlog("Average Gas LPH: ");
    printlogln(fAvgGasLPH);
    printlog("Average Gas KPL: ");
    printlogln(fAvgGasKPL);
    printlog("Average LPG LPH: ");
    printlogln(fAvgLPGLPH);
    printlog("Average LPG KPL: ");
    printlogln(fAvgLPGKPL);
  }
  datafile.close();
  logopen = false;
} // ReadOBD
  

void ProcessSMS () {
  // Read # of SMS, process them all
  uint16_t smslen;
  int8_t smsnum = fona.getNumSMS();

  if (smsnum == 0) {
    printlogln(F("No SMS received"));
  }
  else if (smsnum < 0) {
    printlogln(F("ERROR: Could not read # SMS"));
  }
  else {
    if (smsnum > 0) {
      uint8_t smsid = 1;
      bool smsfound = false;
      while (!smsfound) {
        if (! fona.readSMS(smsid, replybuffer, 250, &smslen)) { // Read message #1 pass in buffer and max len!
          printlog("ERROR Failed readSMS #");printlogln(smsid);
          smsid++;
        }
        else {
          strcpy(message,replybuffer);
          printlog("Reading SMS #");printlogln(smsid);
          smsfound = true;
        }
      }
      if (smsfound) {
        
        if (! fona.getSMSSender(smsid, replybuffer, 250)) {
          printlogln("ERROR failed getSMSSender!");
        }
        else {
          char sender[255];
          strcpy(sender, replybuffer);
          printlog(F("FROM: ")); printlogln(sender);
          
          if (fona.deleteSMS(smsid)) {
            printlogln(F("Message is deleted OK!"));
          } else {
            printlogln(F("Couldn't delete SMS"));
          }
          printlog(F("***Message received: ")); printlogln(message);
          char *ptr = strtok(message, delim);
          sprintf(command, "%s", ptr);
          printlog(F("***Command parsed: ")); printlogln(command);
          ptr = strtok(NULL, delim);
          sprintf(readpass, "%s", ptr);
          printlog(F("***Password sent: ")); printlogln(readpass);

          if (strcmp(command,"PASS") == 0) {
            int i = strcmp(readpass, EEPROMSave.password);
            if (i != 0) {
              printlogln(F("WARNING Password sent is NOT correct.. bypassing action"));
            }
            else {
              printlogln(F("***Password sent is correct"));
              if (ptr != NULL) {  // parse new password
                ptr = strtok(NULL, delim);
                sprintf(newpass, "%s", ptr);
                printlog(F("***New password sent: ")); printlogln(newpass);
                strcpy(EEPROMSave.password, newpass);
                //WriteEEPROM();
                if (!fona.sendSMS(sender, "New Password OK")) {
                  printlogln(F("ERROR could not send SMS"));
                } else {
                  printlogln(F("SMS sent with password confirmation"));
                }
              }
            }
          }
          if (strcmp(command,"ADMN") == 0) {
            int i = strcmp(readpass, EEPROMSave.adminpass);
            if (i != 0) {
              printlogln(F("WARNING Admin Password sent is NOT correct.. bypassing action"));
            }
            else {
              printlogln(F("***Admin Password sent is correct"));
              if (ptr != NULL) {  // parse new password
                ptr = strtok(NULL, delim);
                sprintf(newpass, "%s", ptr);
                printlog(F("***New password sent: ")); printlogln(newpass);
                strcpy(EEPROMSave.adminpass, newpass);
                //WriteEEPROM();
                if (!fona.sendSMS(sender, "New Admin Password OK")) {
                  printlogln(F("ERROR could not send SMS"));
                } else {
                  printlogln(F("SMS sent with password confirmation"));
                }
              }
            }
          }
          if (strcmp(command,"GPS?") == 0) {
            int i = strcmp(readpass, EEPROMSave.password);
            if (i != 0) {
              printlogln(F("WARNING Password sent is NOT correct.. bypassing action"));
            }
            else {
              char gpsdata[120];
              fona.getGPS(0, gpsdata, 120);
              printlogln(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
              printlogln(gpsdata);

              if (!fona.sendSMS(sender, gpsdata)) {
                printlogln(F("ERROR could not send SMS"));
              } else {
                printlogln(F("SMS sent with GPS coordinates"));
              }
            }
          }
          if (strcmp(command,"SHUT") == 0) {
            int i = strcmp(readpass, EEPROMSave.adminpass);
            if (i != 0) {
              printlogln(F("WARNING Admin Password sent is NOT correct.. bypassing action"));
            }
            else {
              printlogln(F("***Admin Password sent is correct. Shutting down ECU"));
              // disconnect ECU through relay
              digitalWrite(PIN_ON, LOW);
              digitalWrite(PIN_OFF, HIGH);
              //WriteEEPROM();
              if (!fona.sendSMS(sender, "LPG ECU Shut down OK")) {
                printlogln(F("ERROR could not send SMS"));
              } else {
                printlogln(F("SMS sent for Shutdown"));
              }
            }
          }
          if (strcmp(command,"CONN") == 0) {
            int i = strcmp(readpass, EEPROMSave.adminpass);
            if (i != 0) {
              printlogln(F("WARNING Admin Password sent is NOT correct.. bypassing action"));
            }
            else {
              printlogln(F("***Admin Password sent is correct. Turning on ECU"));
              // Connect ECU through relay
              digitalWrite(PIN_OFF, LOW);
              digitalWrite(PIN_ON, HIGH);
              //WriteEEPROM();
              if (!fona.sendSMS(sender, "LPG ECU Reconnect OK")) {
                printlogln(F("ERROR could not send SMS"));
              } else {
                printlogln(F("SMS sent for Reconnect"));
              }
            }
          }

        }



      }

    } // if (smsnum > 0)

  }
  printlogln(F("\r\n"));
  datafile.close();
  logopen = false;
} // ProcessSMS

/*
void WriteEEPROM() {
    
    // write EEPROMSave struct to flash at address 1
    uint8_t b2[sizeof(xSave)]; // create byte array to store the struct
    memcpy(b2, &EEPROMSave, sizeof(xSave)); // copy the struct to the byte array
    for ( a=1; a < sizeof(xSave)+1; a++){
      EEPROM.update(a,b2[a]);
    }
    printlogln(F("***Wrote Data to EPROM memory!"));
    
}
*/




void HTTPPost() {
      datafile = SD.open(filename, FILE_WRITE);
      if (datafile) {
        logopen = true;
      }
      else {
        SerialUSB.println(F("SD file open failed, or not present"));
      }
      printlogln(F("\r\n*****  HTTP POST  ******"));

      // get GPS location
      
      if (fona.getGPS(&lat,&lon,&speedkph,&heading,&altitude) ) {
        
        printlog(F("Latitude: "));printlogln(lat);
        printlog(F("Longitude: "));printlogln(lon);
        printlog(F("Altitude: "));printlogln(altitude);
        lastlat = lat;
        lastlon = lon;
        // Check if RTC has a proper date else use GPS date to synch
        
      } else {
        printlog(F("GPS not locked, using last coordinate: "));
        printlog(lastlat);
        printlog(F(","));
        printlogln(lastlon);
      }
      //if (rtc.getYear() == 0) {
        printlogln(F("Synch RTC with GPS date"));
        if (fona.getGPStime(gpsdate)) {
          printlog(F("GPS Date: "));printlogln(gpsdate);
          rtc.setYear(valueFromString(gpsdate,2,2));
          rtc.setMonth(valueFromString(gpsdate,4,2));
          rtc.setDay(valueFromString(gpsdate,6,2));
          rtc.setHours(valueFromString(gpsdate,8,2) + UTC);
          rtc.setMinutes(valueFromString(gpsdate,10,2));
          rtc.setSeconds(valueFromString(gpsdate,12,2));
        }
      //}

      rtctimestamp(timestamp);
      printlog(F("Current Time: ")); printlogln(timestamp);

      if (!postflag) {
        printlogln(F("****POSTing is disabled!"));
      }
      else{
        
        if (lastpost) {
          printlogln(F("LAST POST after engine is turned off!"));
          lastpost = false;
          postflag = false;
        }

        if (CheckNetwork()) {
            // build JSON string for HTTP POST
            printlogln(F("HTTP POST data.."));
            char data[350];
            char url[80];
            uint16_t statuscode;
            int16_t lengthp;
            strcpy (url,serverurl);
            PString pdata(data,350);
            
        
            pdata.print("{\"datetime\":\"");
            pdata.print(timestamp);
            pdata.print("\",\"device\":\"");
            pdata.print(deviceID);
            pdata.print("\",\"lgpTime\":"); 
            pdata.print(fLPGTime);
            pdata.print(",\"lgpPercentage\":");
            pdata.print(fLPGPerc);
            pdata.print(",\"lgpLh\":");
            pdata.print(fAvgLPGLPH);
            pdata.print(",\"lgpKml\":");
            pdata.print(fAvgLPGKPL);
            pdata.print(",\"petrolTime\":");
            pdata.print(fGasTime);
            pdata.print(",\"petrolPercentage\":");
            pdata.print(fGasPerc);
            pdata.print(",\"petrolLh\":");
            pdata.print(fAvgGasLPH);
            pdata.print(",\"petrolKml\":");
            pdata.print(fAvgGasKPL);
            pdata.print(",\"owner\":\"");
            pdata.print(ownerID);
            pdata.print("\",\"latitude\":");
            pdata.print(lastlat,4);
            pdata.print(",\"longitude\":");
            pdata.print(lastlon,4);
            pdata.print("}");
         
            printlogln(pdata);
            
            if (!fona.HTTP_POST_start(url, F("application/json"), (uint8_t *) data, strlen(pdata), &statuscode, (uint16_t *)&lengthp)) {
                printlogln("Failed HTTP POST! Wait 5 secondsto get error and retry");
                delay(5000);
                fona.HTTP_POST_end();
              }
              else {
                while (lengthp > 0) {
                  while (fona.available()) {
                    char c = fona.read();
                    SerialUSB.write(c);
          
                    lengthp--;
                    if (! lengthp) break;
                  }
                }
                printlogln(F("\n\n****"));
                fona.HTTP_POST_end();
            
                // reset variables to start next measurement cycle
                fGasTime = 0;
                fLPGTime = 0;
                fGasPerc = 0;
                fLPGPerc = 0;
                fAvgGasLPH = 0;
                fAvgGasKPL = 0;
                fAvgLPGLPH = 0;
                fAvgLPGKPL = 0;
                fSumGasLPH = 0;
                fSumGasKPL = 0;
                fSumLPGLPH = 0;
                fSumLPGKPL = 0;
                nSamples = 0;
                elapsedmillis = 0;
                fGasLiters = 0;
                fLPGLiters = 0;
                fLPS = 0;
      
              
            } // HTTP POST 
           if (!fona.enableGPRS(false))
              printlogln(F("Failed to turn off GPRS."));
           else
              printlogln(F("GPRS disconnected"));
        
        }
      } // if (postflag == false)
     
      datafile.close();
      logopen = false;
}


bool CheckNetwork() {
  // Always get RSSI
  uint8_t i = fona.getRSSI();
  int8_t dbm;
  printlog(F("RSSI = ")); printlog(i); printlog(": ");
  if (i == 0) dbm = -115;
  if (i == 1) dbm = -111;
  if (i == 31) dbm = -52;
  if ((i >= 2) && (i <= 30)) {
    dbm = map(i, 2, 30, -110, -54);
  }
  printlog(dbm); printlogln(F(" dBm"));
  
  uint8_t n = fona.getNetworkStatus();
  if (n != 1) {
    printlog(F("Network not registered yet. Status: ")); printlogln(n);
    return false;
  }
  else {
    printlogln(F("Network Registered"));
    if (!fona.enableGPRS(true)) {
        printlogln(F("Failed to turn GPRS!"));
        return false;
    } else {
      printlogln(F("GPRS Enabled!"));
      return true;
    }
    
  }
}

void rtctimestamp( char *timestamp) {
  int fixyear;
  fixyear = 2000 + rtc.getYear();
  sprintf(timestamp,"%.4d-%.2d-%.2d %.2d:%.2d:%.2d",fixyear,rtc.getMonth(),rtc.getDay(),rtc.getHours(),rtc.getMinutes(),rtc.getSeconds());
}

byte valueFromString(char *string,byte start, byte width)
{
  byte value=0;
  char convert;
  for( byte n=0; n < width; n++ ){
    value = value * 10 + string[start +n] - '0'; 
  }
  
  return value;  
}  
// TO DO:
// figure how to keep battery supply . no easy way with current circuit.
