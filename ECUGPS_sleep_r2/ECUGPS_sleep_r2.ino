 #include<stdio.h>

#include "Adafruit_FONA.h"

#include <OBD9141.h>
//#include <EEPROM.h>
#include <string.h>
#include <PString.h>
#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>


/////////////////////////////////////////////////////////////////////////////////////////////////////
//                  ECU - GPS controller
//                  Ricardo Vargas 
/////////////////////////////////////////////////////////////////////////////////////////////////////

/* 
  Notes:
  Ported to ATSAMD21 (Feather M0 test board)
  Sleep features included 6 June 2020
*/


// OBD type (CAN / 9141 /KWD) will need to be programmed via software since no resource was allocated in hardware

//Use Serial 2 on Custom Board
#define ISO_TX 1
#define ISO_RX 2

#define LPGSTATUSPIN A0 // A0 analog input
#define LIN_SLP 3 // sleep pin for LIN IC


// pin for FONA
#define FONA_RST 4
#define FONA_TX 5
#define FONA_RX 6
#define FONA_PWRKEY 7
#define PWR_GATE 19 

// Set reset for 12V relay
#define PIN_OFF 26
#define PIN_ON 25

// SD pins
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
//char deviceID[37] = "0d8a50b8-c3d2-5c4e-a671-cf5809ac4f12"; //Daniel 2. de daescastros
char deviceID[37] = "4b035011-a59f-5637-8b0d-8b5efada1b2b"; //Daniel Castro. de daescastros //Carro de Carlos
char ownerID[37] = "1651e756-93f8-52be-a7b0-7332c2c7c66d"; //daescastros
char serverurl[80] = "https://airenuevoapp.japuware.com/api/v1/stats";

//char ownerID[37] = "2818baa6-9263-5133-a102-5279845967fc"; // Owner ID Ricardo
//char deviceID[37] = "fd5c8844-c060-56f9-9772-4c6467572a10"; //  Device ID Ricardo

//Timer reset values for tasks
uint16_t TC4_time = 0x0078; // 120 seconds (3 minutes)
uint16_t TC5_time = 0x0257; // 600 second (10 minutes)
uint16_t Tcount;

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
// 4: SIMULATION
uint8_t obdflag = 1;
char serialcommand = '0';
float simbatvolt = 4.2; // variable to simulate battery voltage
bool pwrgatestatus = false;
bool chargersentstatus = false;

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

uint16_t vbat;

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
bool GPSflag = false; // Ricardo - 28 Mar 2020 add GPS status flag
bool SIMflag = false; // Ricardo 21 Mar 2020 

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
// Ricardo - eliminated the Liters per Second metric

// GPS variables
float lat, lon, speedkph,heading, altitude;
float lastlat = 0;
float lastlon = 0;  // to track last available coordinates
char gpsdate[20];
bool gpslock = false;  // Wait until first GPS lock to disable and sleep
bool FonaSleep = false; // To track if FONA is asleep or not
uint8_t batreads = 0; // allow 2 measures of charging before turning charger on or off

char data[400];
PString pdata(data,400);
bool success = false;

//Variables for DayClosingPost
bool alreadyPosted = false;

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
  // put your setup code here, to run once:s
//  while (!SerialUSB);
  printlogln("Wait 8 seconds to begin\n");
  delay(8000);
  
  pinMode(PIN_ON, OUTPUT);
  pinMode(PIN_OFF, OUTPUT);
  pinMode(FONA_RST, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(SD_DT,INPUT);
  
  pinMode(PWR_GATE,OUTPUT);
  
  SerialUSB.begin(115200);
  rtc.begin();
  
    // SD Card setup

    // see if the card is present and can be initialized:
    if (digitalRead(SD_DT) == HIGH) {
      SerialUSB.println(F("SD card not present"));
    }
    else {
      if (SD.begin(SD_CS)) {
        SerialUSB.println(F("SD card initialized"));
        datafile = SD.open(filename, FILE_WRITE);
        logopen = true;
      } 
      else {
        SerialUSB.println(F("SD card init Error"));
      }
    }

  printlogln(F("\r\n\r\n**********************"));
  printlogln(F("*****  ECU GPS  ******"));
  
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

  printlogln(F("Clearing relay status"));
      digitalWrite(PIN_OFF, LOW);
      digitalWrite(PIN_ON, LOW);
      delay(500);
      
   if (EEPROMSave.relay) {
      printlogln(F("Checking power relay is ON"));
      digitalWrite(PIN_OFF, LOW);
      digitalWrite(PIN_ON, HIGH);
   } else {
      printlogln(F("Checking power relay is OFF"));
      digitalWrite(PIN_ON, LOW);
      digitalWrite(PIN_OFF, HIGH);
   }
  
  pinMode(FONA_RST, OUTPUT);
  pinMode(FONA_PWRKEY, OUTPUT);
  
  digitalWrite(FONA_RST, HIGH);
  delay(100);


   fonaSerial->begin(9600);
   fona.initPort(*fonaSerial);
   
   SimTogglePWRKEY();

   if (! fona.begin()) {
     printlogln(F("Couldn't find FONA"));
   }
   else {
     SIMflag = true;
	 printlogln(F("***SIMflag TRUE"));
     type = fona.type();
     printlogln(F("FONA is found OK"));
	 
	 //Turn on battery recharge option
     printlogln(F("Turning on battery charging"));
     if(!fona.enableBattCharging(1))
		  printlogln(F("Failed turning battery charging on"));
     else
	 {
       printlogln(F("Battery charging turned on"));

       // Ricardo 23 March 2020 - put back again the restart in case of a car battery failure
       SimTogglePWRKEY();
       if (! fona.begin()) {
          printlogln(F("Couldn't find FONA"));
          delay(5000);
       }
			 // Print module IMEI number.
			 char imei[16] = {0};
			 uint8_t imeiLen = fona.getIMEI(imei);
			 if (imeiLen > 0) {
				printlog(F("Module IMEI: ")); printlogln(imei);
			 }
			 fona.getSIMCCID(replybuffer);
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
				GPSflag = true;
		
		 }
	 }

   printlogln(F("Wait 2 seconds to start main loop..."));
   delay (2000);
   // TO DO: change startup method based on jumper configuration
     if (obdflag == 4) {  // SIMULATION
        printlogln(F("OBD in simulation mode!"));
     }
     if (obdflag == 3) {
         //start CAN ports,  enable interrupts and RX masks, set the baud rate here
         printlogln(F("OBD Port Type: CAN"));
         //CANport0.initialize(_500K)
      }
      else if (obdflag == 2) {
        printlogln(F("OBD Port Type: ISO 9141 fast (KWP)"));
        //digitalWrite(LIN_SLP, HIGH);
        obd9141.begin(Serial2, ISO_RX, ISO_TX);
        init_success =  obd9141.initKWP(); // Aveo uses KWP
        printlog("OBD2 init success:"); printlogln(init_success);
      } else if (obdflag == 1) {
        printlogln(F("OBD Port Type: ISO 9141 slow"));
        //digitalWrite(LIN_SLP, HIGH);
        obd9141.begin(Serial2, ISO_RX, ISO_TX);
        init_success =  obd9141.init(); // crossfox uses 50 baud init
        printlog("OBD2 init success:"); printlogln(init_success);
      }
	  
   printlogln(F("Clearing relay status"));//Daniel 1/3/2020. Relay set to low save energy
   digitalWrite(PIN_OFF, LOW);
   digitalWrite(PIN_ON, LOW);
	
   // Time from RTC
   rtctimestamp(timestamp);
   printlog(F("Current Time: ")); printlogln(timestamp);
   // Timer setup
   TCsetup();
   
   startmillis = millis(); // discount setup time from delta.
   printlogln(F("*****  Init End  ******\r\n"));
   datafile.flush();
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
  // Ricardo 6/6/20 - added CPU sleep time in case engine is off
  //				- Added code to use counters to trigger SMS and HTTP functions
  if (! engineflag) {
	  SerialUSB.end();
	  LowPower.sleep(5000);
	  // Resume from Wake-up
	  SerialUSB.begin(115200);
  }
  ReadOBD();
  
  Tcount = REG_TC4_COUNT16_COUNT;
  SerialUSB.print("TC4 Count: ");SerialUSB.println(Tcount);
  if (Tcount == 65535) {
   // WriteEEPROM();
    ProcessSMS();
	REG_TC4_COUNT16_COUNT = TC4_time;                      // Set period register
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	REG_TC4_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  }
  
  Tcount = REG_TC5_COUNT16_COUNT;
  SerialUSB.print("TC5 Count: ");SerialUSB.println(Tcount);
  if (Tcount == 65535) {
   // WriteEEPROM();
    HTTPPost();
	REG_TC5_COUNT16_COUNT = TC5_time;                      // Set period register
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
	REG_TC5_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
	while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  }
  
	DayClosingPost(); //Last post at the end of the day
}

void ReadOBD() {
  //SDCheckOpen();
    
  // Ricardo 9 March 2020 - moved ellapsed millis to closer to the actual calculation, time is lost during reading so it didnt add up
   
  // Read analog pin 
  LPGStatusRead = analogRead(LPGSTATUSPIN);
  if (obdflag == 4) {
    LPGStatusRead = random(1024);
  }
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
  else if (obdflag == 4) {
    printlogln(F("OBD simulation mode"));
    delay(2000);

    if (Serial.available() > 0) {
      serialcommand = SerialUSB.read();
       printlogln(F("Serial command read!"));
      switch (serialcommand) {
        case '0': { // engine off
            engineflag = false;
            break;
          }
        case '1': { // engine on
          engineflag = true;
          break;
        }
		
      }
    }
    if (engineflag) {
		
        postflag = true;
        fRPM = random (5000);
        fIMAP = random(20,50);
        fIAT = random(25,50);
        fSpeed = random(100);
        printlog("RPM: ");
        printlog(fRPM);
        printlog(", Speed: ");
        printlog(fSpeed);
        printlog(", IMAP: ");
        printlog(fIMAP);
        printlog(", IAT: ");
        printlog(fIAT);
    }
    else {
        printlogln(F("Vehicle ECU turned off in simulation mode"));
        startmillis = millis(); // Ricardo 13 March 2020 - reset start counter to avoid false times adding up
        if (engineflag == true) { // Ricardo 24 March 2020 - include an engine on check since car could have turned off
          printlogln(F("* Engine just turned off - Flag for posting"));
          lastpost = true;
          postflag = true;
        }
    }
    
    
          
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
        delay(500);
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
    		  startmillis = millis(); // Ricardo 13 March 2020 - reset start counter to avoid false times adding up
    		  if (engineflag == true) { // Ricardo 24 March 2020 - include an engine on check since car could have turned off
      			printlogln(F("* Engine just turned off - Flag for posting"));
      			lastpost = true;
      			postflag = true;
      			engineflag = false;
    		  }
        } else {
          res = obd9141.getCurrentPID(RPM,_16BITS);
          if (!res) {
            printlogln(F("RPM reading still not valid"));
            return;
          }
        }
     } //if (!res)
      
     else {
        fRPM = obd9141.readUint16()/4;
        if (fRPM == 0) {
          // Ricardo - 10 March 2020 if engine was on, flag for a last post 
          if (engineflag == true) {
            printlogln(F("Engine just turned off - Flag for posting"));
            postflag = true;
            lastpost = true;
          }
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

       printlogln("");
     }
  } // if (obdflag == 3)
  

  if (! engineflag) {
    printlogln("Engine is turned off - skipping calculation");
	// If battery level is less than 3.6, turn on charging - Ricardo 28 march 2020
	
		if (! fona.getBattVoltage(&vbat) ) {
		  printlogln(F("Failed to read Batt or SIM is turned off"));
		  batreads = 0;
		} else {
			if (obdflag == 4) {
				 /*if (pwrgatestatus) {
					 // charger off
					 simbatvolt -= 0.05;
				 }
				 else {
					 simbatvolt += 0.05;
				 }
				 vbat = simbatvolt*1000;
				 printlog(F("Simulation VBAT = "));printlog(vbat);
				*/
				printlog(F("VBat = ")); printlog(vbat); printlogln(F(" mV"));
			}
			else {
				printlog(F("VBat = ")); printlog(vbat); printlogln(F(" mV"));
			}
     
		
			if (vbat < 3600) {
				// turn on charging
				digitalWrite(PWR_GATE,LOW);
				pwrgatestatus = false;
				if (chargersentstatus == false) {
					sendText("Turned Charger ON");
					chargersentstatus = true; // send only once
				}
				delay(4000);
			}
			else if (vbat > 4050) {
			  batreads ++;
			  if (batreads > 2) {
  				sendText("Turned Charger OFF battery is charged");
  				delay(4000);
  				digitalWrite(PWR_GATE,HIGH);
  				pwrgatestatus = true;
				chargersentstatus = false;
  				printlogln(F("Turn off SIM808 via PWRKEY"));
  				SimTogglePWRKEY();
  				SIMflag = false;
				printlogln(F("***SIMflag FALSE"));
  				GPSflag = false;
  				// Enter sleep mode for CPU
				
			  }

			}
			else{
			  batreads = 0;
			}
		}

  }
  else {
  	digitalWrite(PWR_GATE,LOW); // Keep charger ON if engine is on.. Ricardo 28 march 2020
	pwrgatestatus = false;
    chargersentstatus = false;
  	if (!SIMflag) {
  		SimTogglePWRKEY();
      if (! fona.begin()) {
        printlogln(F("Couldn't find FONA"));
        delay(5000);
      }
  		SIMflag = true;
		printlogln(F("***SIMflag TRUE"));
      sendText("Engine is turned ON");
  	}
  	// Turn on GPS in case it was disabled
  	if (GPSflag == false) {
  		 if (!fona.enableGPS(true))
  			printlogln(F("Failed to turn ON GPS!"));
  		 else {
  			printlogln(F("GPS Enabled!"));
  			GPSflag = true;
  		 }
  	}
    fAir = fRPM * fIMAP / (fIAT+273) * EngineVol * VE * MAFConst;
    printlog(", Air Gr:" ); printlogln(fAir);

    // Ricardo - Moved ellapsed millis here to include time spent during engine readings
    elapsedmillis = millis() - startmillis; // uses startmillis from previous cycle
    startmillis = millis();
    printlog(F("** Elapsed Milliseconds: **")); printlogln(elapsedmillis);
      
    // Calculate consumption and KPL
    nSamples ++;
    if (bFuelType) {
      // LPG
      fLPH = (fAir * SecPerHour) / (fLPGLambda * fLPGDens);
      if (fLPH != 0 ) {
        fKPL = fSpeed / fLPH;
      }
      else {
        fKPL = 0;
      }       
      fSumLPGLPH = fSumLPGLPH + fLPH;
      fSumLPGKPL = fSumLPGKPL + fKPL;
      fAvgLPGLPH = fSumLPGLPH / nSamples;
      fAvgLPGKPL = fSumLPGKPL / nSamples;
      

      fLPGTime += (float)elapsedmillis/3600000; // elapsed time in hours, changed by Ricardo 14/2/20
      fLPGLiters += fLPH * (float)elapsedmillis/3600000;
    } 
    else {
      // Gasoline
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
      fGasTime += (float)elapsedmillis/3600000; // elapsed time in hours, changed by Ricardo 14/2/20
      fGasLiters += fLPH * (float)elapsedmillis/3600000;
    }
    fGasPerc = 100* fGasTime / (fGasTime + fLPGTime);
    fLPGPerc = 100* fLPGTime / (fGasTime + fLPGTime);
    
    printlog("Fuel Type (TRUE = LPG, FALSE = Gasoline): ");
    printlogln(bFuelType);
    printlog("Liters/Hour: ");
    printlog(fLPH);
    printlogln(", KM/Liter: ");
    printlog(fKPL);
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
	printlogln(F("**** SMS Process *****"));
  if (SIMflag == false) {
    SIMflag = true;
	printlogln(F("***SIMflag TRUE"));
    printlogln(F("Turn ON SIM808 via PWRKEY"));
    SimTogglePWRKEY();
     if (! fona.begin()) {
          printlogln(F("Couldn't find FONA"));
          delay(5000);
		      return;
     }
     else{
        printlogln(F("Turn ON SIM808 successfully"));
        printlogln(F("20 seconds Delay before reading SMS"));
        SerialUSB.end();
        LowPower.sleep(20000);
        // Resume from Wake-up
        SerialUSB.begin(115200);
		
		// turn on charger in case battery is low
		if (! fona.getBattVoltage(&vbat) ) {
		  printlogln(F("Failed to read Batt"));

		} else {
			printlog(F("VBat = ")); printlog(vbat); printlogln(F(" mV"));
		
			if (vbat < 3600) {
				// turn on charging
				digitalWrite(PWR_GATE,LOW);
				pwrgatestatus = false;
				sendText("Turned Charger ON when checking SMS");
			}
		}
		  
		
     }
       
  }

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
		  char *sender = 	replybuffer + 4; // remove 4 digits +506 from number
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
				char sms[140];
				PString psms(sms,140);

				  // GPS was turned off due to engine off
				if (GPSflag == false) {
					if (!fona.enableGPS(true)) {
						printlogln(F("Failed to turn ON GPS!"));
						psms.print("Failed to turn ON GPS!");
					}
					else {
						printlogln(F("GPS Enabled! Waiting 60 seconds for GPS lock"));
						delay(60000);
						GPSflag = true;
					}
				}
			  
			  if (GPSflag) {
				  if (fona.getGPS(&lat,&lon,&speedkph,&heading,&altitude) ) {
					  printlogln(F("Latitude: "));printlogln(lat);
					  printlogln(F("Longitude: "));printlogln(lon);
					  lastlat = lat;
					  lastlon = lon;
				  }
				  else {
					  psms.print("GPS not locked. ");

				  }
				  if (lastlat != 0) {
					  psms.print("https://www.google.com/maps/search/?api=1&query=");
					  psms.print(lastlat,4);
					  psms.print(",");
					  psms.print(lastlon,4);
				  }
			  }
			  
			  if (!fona.sendSMS(sender, sms)) {
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
			  delay(500);//Daniel 1/3/2020. Relay set to low save energy
			  printlogln(F("Clearing relay status"));
			  digitalWrite(PIN_OFF, LOW);
			  digitalWrite(PIN_ON, LOW);
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
			  delay(500);//Daniel 1/3/2020. Relay set to low save energy
			  printlogln(F("Clearing relay status"));
			  digitalWrite(PIN_OFF, LOW);
			  digitalWrite(PIN_ON, LOW);
			  //WriteEEPROM();
			  if (!fona.sendSMS(sender, "LPG ECU Reconnect OK")) {
				printlogln(F("ERROR could not send SMS"));
			  } else {
				printlogln(F("SMS sent for Reconnect"));
			  }
			}
		  }
		  // Ricardo 2 FEB 2020 - Include a command to read back battery voltage through SMS
		  if (strcmp(command,"BAT?") == 0) {
			  int i = strcmp(readpass, EEPROMSave.password);
			  if (i != 0) {
				printlogln(F("WARNING Password sent is NOT correct.. bypassing action"));
			  }
			  else {

						char sms[140];
						PString psms(sms,140);
						if (! fona.getBattVoltage(&vbat)) {
						  printlogln(F("Failed to read Batt"));
						} else {
							printlogln(F("VBat = ")); printlogln(vbat); printlogln(F(" mV"));
							psms.print("Battery Voltage (mV):");
							psms.print(vbat);
						}
						if (!fona.sendSMS(sender, sms)) {
							printlogln(F("ERROR could not send SMS"));
						} else {
							printlogln(F("SMS sent with Battery Voltage"));
						}
				
				  } // else
		  } // if (strcmp(command,"BAT?") == 0) 
			// Ricardo 13 MARCH 2020 - Include a command to read back the OBD2 Data
		  if (strcmp(command,"DATA?") == 0) {
			  int i = strcmp(readpass, EEPROMSave.password);
			  if (i != 0) {
				printlogln(F("WARNING Password sent is NOT correct.. bypassing action"));
			  }
			  else {
						
						char sms[140];
						PString psms(sms,140);
						psms.print("Fuel:");
						psms.print(bFuelType);
						psms.print(",LPH:");
						psms.print(fLPH);
						psms.print(",KPL:");
						psms.print(fKPL);
						psms.print(",Gt:");
						psms.print(fGasTime);
						psms.print(",Lt:");
						psms.print(fLPGTime);
						psms.print(",Gl:");
						psms.print(fGasLiters);
						psms.print(",Ll:");
						psms.print(fLPGLiters);
						
						if (!fona.sendSMS(sender, sms)) {
							printlogln(F("ERROR could not send SMS"));
						} else {
							printlogln(F("SMS sent with Parametric data"));
						}
				
				  } // else
		  } // if (strcmp(command,"BAT?") == 0) 



		} // fona.getSMSSender(smsid, replybuffer, 250)) {
	  } // if sms found

	} // if (smsnum > 0)

  }
  printlogln(F("\r\n"));
  datafile.flush();
  logopen = false;


	if (!engineflag && pwrgatestatus == true) {
		    printlogln(F("Charger is off - Turn off SIM808 via PWRKEY"));
        SimTogglePWRKEY();
        SIMflag = false;
		printlogln(F("***SIMflag FALSE"));
	}
	
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


bool HTTPPost() {
      //SDCheckOpen();
	  
	  if (!postflag) {
        printlogln(F("****HTTP POSTing is disabled!"));
      }
      else{

		  if (SIMflag == false) {
			printlogln(F("Turn ON SIM808 via PWRKEY"));
			SimTogglePWRKEY();
			SIMflag = true;
			printlogln(F("***SIMflag TRUE"));
			 if (! fona.begin()) {
				  printlogln(F("Couldn't find FONA"));
				  delay(5000);
			   }
			 else{
				printlogln(F("Turn ON SIM808 successfully"));
				printlogln(F("20 seconds Delay before reading SMS"));
				delay(20000);
			 }
			   
		  }
		  printlogln(F("\r\n*****  HTTP POST  ******"));

		  uint16_t batt; //Battery reports
			if (! fona.getBattVoltage(&batt)) {
			  printlogln(F("Failed to read Batt"));
			} else {
			  printlog("VBat = "); printlog(batt); printlogln(" mV");
			}


			if (! fona.getBattPercent(&batt)) {
			  printlogln("Failed to read Batt");
			} else {
			  printlog("VPct = "); printlog(batt); printlogln("%");
			} //Battery reports

			// GPS was turned off due to engine off
			if (GPSflag == false) {
			  if (!fona.enableGPS(true)) {
				printlogln(F("Failed to turn ON GPS!"));
			  }
			  else {
				printlogln(F("GPS Enabled! Waiting 60 seconds for GPS lock"));
				delay(60000);
				GPSflag = true;
			  }
			}
			
			if (GPSflag) {
			  if (fona.getGPS(&lat,&lon,&speedkph,&heading,&altitude) ) {
				
				printlog(F("Latitude: "));printlogln(lat);
				printlog(F("Longitude: "));printlogln(lon);
				printlog(F("Altitude: "));printlogln(altitude);
				lastlat = lat;
				lastlon = lon;
				
				
			  } else {
				printlog(F("GPS not locked, using last coordinate: "));
				printlog(lastlat);
				printlog(F(","));
				printlogln(lastlon);
			  }
				// Check if RTC has a proper date else use GPS date to synch
				printlogln(F("Synch RTC with GPS date"));
				if (fona.getGPStime(gpsdate)) {
				  printlog(F("GPS Date: "));printlogln(gpsdate);
				  rtc.setYear(valueFromString(gpsdate,2,2));
				  rtc.setMonth(valueFromString(gpsdate,4,2));
				  rtc.setDay(valueFromString(gpsdate,6,2));
				  if ((valueFromString(gpsdate,8,2) + UTC) < 0) {
					rtc.setHours(valueFromString(gpsdate,8,2) + UTC + 24);
				  } else{
					rtc.setHours(valueFromString(gpsdate,8,2) + UTC);
				  }
				  rtc.setMinutes(valueFromString(gpsdate,10,2));
				  rtc.setSeconds(valueFromString(gpsdate,12,2));
				}
			}


		  rtctimestamp(timestamp);
		  printlog(F("Current Time: ")); printlogln(timestamp);

		  
			
			if (lastpost) {
			  printlogln(F("LAST POST after engine is turned off!"));
			  lastpost = false;
			}

			if (CheckNetwork()) {
				// build JSON string for HTTP POST
				delay(500);
				printlogln(F("HTTP POST data.."));

				char data[350];
				PString pdata(data,350);
				
				uint16_t statuscode;
				int16_t lengthp;
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
				pdata.print("\",\"lgpL\":\"");
				pdata.print(fLPGLiters);
				pdata.print("\",\"petrolL\":\"");
				pdata.print(fGasLiters);
				pdata.print("\",\"latitude\":");
				pdata.print(lastlat,4);
				pdata.print(",\"longitude\":");
				pdata.print(lastlon,4);
				pdata.print("}");
			 
				printlogln(data);

				char sms[140];
				PString psms(sms,140);
				psms.print("Gast:");
				psms.print(fGasTime);
				psms.print(",LPGt:");
				psms.print(fLPGTime);
				psms.print(",Gasl:");
				psms.print(fGasLiters);
				psms.print(",LPGl:");
				psms.print(fLPGLiters);    
       			
				sendText(sms);
				delay(5000);
				
				success = false;
				printlogln(F("POSTing data.."));
				
				if (fona.HTTP_POST_start(serverurl, F("application/json"), (uint8_t *) data, strlen(pdata), &statuscode, (uint16_t *)&lengthp)) {
				  success = true;
				  while (lengthp > 0) {
					while (fona.available()) {
					  char c = fona.read();
					  SerialUSB.write(c);
			
					  lengthp--;
					  if (! lengthp) break;
					}
				  }
			  			  
				  printlogln("");
				  printlogln(F("POSTing success"));
				  printlogln(F("\n\n****"));
				  fona.HTTP_POST_end();
				  postflag = false;  // Ricardo 24 Mar 2020 - moved post flag here - flag only clears after a successful posting

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
				  //elapsedmillis = 0; counter should not be reset - Ricardo 13 March 2020
				  fGasLiters = 0;
				  fLPGLiters = 0;
				  
				}
				else {
					printlogln(F("Failed HTTP POST! Wait 5 seconds to get error and continue"));
					sendText("Failed HTTP POST");
					delay(5000);
					fona.HTTP_POST_end();
				}
				 // HTTP POST
			}
			   
			   
			 if (!fona.enableGPRS(false))
				printlogln(F("Failed to turn off GPRS."));
			 else
				printlogln(F("GPRS disconnected"));
			
			if (!engineflag && pwrgatestatus == true) {
				printlogln(F("Charger is off - Turn off SIM808 via PWRKEY"));
				SimTogglePWRKEY();
				SIMflag = false;
				printlogln(F("***SIMflag FALSE"));
			}
		} // if (postflag == false)
		 
		  datafile.close();
		  logopen = false;

	return success;
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
      sendText("Failed Turn on GPRS");
    } else {
      printlogln(F("GPRS Enabled!"));
      
    }
    return true;
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

void sendText(char *msg)
{
	printlogln(msg);
	if (!fona.sendSMS("84050685", msg)) {
		printlogln(F("ERROR could not send SMS"));
  	} else {
  	    printlogln(F("SMS sent with message"));
  	}
}

void SDCheckOpen() { 
  if (digitalRead(SD_DT) == HIGH) {
      SerialUSB.println(F("SD card not present"));
  }
  else {
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
         SerialUSB.println(F("SD card failed!"));
       }
     } //if (datafile)
    
  }
}

void DayClosingPost() //Daniel Castro 29/2/2020. Makes post at the end of the day if engine is off
{
	if ((rtc.getHours() == 23)&&(rtc.getMinutes() >= 58)&& (!alreadyPosted))
	{
		lastpost = true;
		postflag = true;
		if(HTTPPost()) alreadyPosted = true;
	}
	if (rtc.getMinutes() < 10) alreadyPosted = false;
}

void SimTogglePWRKEY()
{
	
	printlogln(F("Toggling PWRKEY..."));
	digitalWrite(FONA_PWRKEY, HIGH);
	delay(1000);
	digitalWrite(FONA_PWRKEY, LOW);
	delay(1800);
	digitalWrite(FONA_PWRKEY, HIGH);
	delay(500);
  
}

// Setup timers for multiple tasks
void TCsetup() {
  // Power Manager turn off unneeded peripherals
  PM->APBBMASK.reg &= ~PM_APBBMASK_DMAC;
  PM->AHBMASK.reg &= ~PM_AHBMASK_DMAC;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM1;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM3;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC0;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC1;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC2;
  PM->APBCMASK.reg &= ~PM_APBCMASK_AC;
  PM->APBCMASK.reg &= ~PM_APBCMASK_DAC;
  PM->APBCMASK.reg &= ~PM_APBCMASK_I2S;
  
  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(32) |          // Divide the 32KHz clock source by divisor 8
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_OSCULP32K |   // Set the 32KHz ULP clock source
                     GCLK_GENCTRL_RUNSTDBY |
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 
  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;    // Feed GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

// TC4
  REG_TC4_COUNT16_COUNT = TC4_time;                      // Set period register
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  //REG_TC4_INTENSET = TC_INTENSET_OVF; // Enable TC3 interrupts
 
  //NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 0 (highest)
  //NVIC_EnableIRQ(TC4_IRQn);         // Connect TC3 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_CTRLBSET |= TC_CTRLBSET_ONESHOT |       // one shot operation
                      TC_CTRLBSET_DIR;            // count down
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |      // Set prescaler to 1024, 1Hz
                   TC_CTRLA_MODE_COUNT16 |         // Set the counter to 16-bit mode
                   TC_CTRLA_RUNSTDBY |
                   TC_CTRLA_ENABLE;               // Enable TC3
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  REG_TC4_READREQ = TC_READREQ_RCONT |            // Enable a continuous read request
                    TC_READREQ_ADDR(0x10);        // Offset of the 16 bit COUNT register
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for (read) synchronization


// TC5
  REG_TC5_COUNT16_COUNT = TC5_time;                      // Set period register
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization

  //REG_TC4_INTENSET = TC_INTENSET_OVF; // Enable TC3 interrupts
 
  //NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC3 to 0 (highest)
  //NVIC_EnableIRQ(TC4_IRQn);         // Connect TC3 to Nested Vector Interrupt Controller (NVIC)

  REG_TC5_CTRLBSET |= TC_CTRLBSET_ONESHOT |       // one shot operation
                      TC_CTRLBSET_DIR;            // count down
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  REG_TC5_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |      // Set prescaler to 1024, 1Hz
                   TC_CTRLA_MODE_COUNT16 |         // Set the counter to 16-bit mode
                   TC_CTRLA_RUNSTDBY |
                   TC_CTRLA_ENABLE;               // Enable TC3
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  
  REG_TC5_READREQ = TC_READREQ_RCONT |            // Enable a continuous read request
                    TC_READREQ_ADDR(0x10);        // Offset of the 16 bit COUNT register
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for (read) synchronization
}
