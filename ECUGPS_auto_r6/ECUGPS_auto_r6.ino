#include<stdio.h>

#include "Adafruit_FONA.h"

#include <OBD9141.h>
#include <string.h>
#include <PString.h>
#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <ArduinoLowPower.h>
#include <WDTZero.h>
#include <CAN.h>
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

#define LPGSTATUSPIN 0 // A0  input now digital
#define ISO_TX 1
#define ISO_RX 2
#define LIN_SLP 3 // sleep pin for LIN IC


// pin for FONA
#define FONA_RST 4
#define FONA_TX 5
#define FONA_RX 6
#define FONA_PWRKEY 7

//CAN Bus
#define CAN_RST 19

// Set reset for 12V relay
#define PIN_OFF 25
#define PIN_ON 26

// Power pins
#define IGN_DET 28
#define DTR 29
#define PWR_GATE 30 
#define PWR_GPS 31

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

#define MIN_BAT_V 3650
#define MAX_BAT_V 4050
#define BAT_READS 3

#define UTC -6
//////////


void runconfig();
void SDWriteFile();
void sendText(char* sender, char *msg);
void GetGPSandTime();
void rtctimestamp( char *timestamp);
void TCsetup();
void SDdateTime(uint16_t* date, uint16_t* time);
void wdt_reset();
void Ignition();
void DayClosingPost();
void BattMeasure();
bool CheckNetwork();
void testOBD();

// TO DO: implement a method to program these variables through SMS
//char deviceID[37] = "0d8a50b8-c3d2-5c4e-a671-cf5809ac4f12"; //Daniel 2. de daescastros
char deviceID[37] = "4b035011-a59f-5637-8b0d-8b5efada1b2b";  //Carro de Carlos
//char ownerID[37] = "1651e756-93f8-52be-a7b0-7332c2c7c66d"; //daescastros
//char serverurl[80] = "https://airenuevoapp.japuware.com/api/v1/stats";
char serverurl[80] = "https://3.19.239.190/api/v1/stats";

char ownerID[37] =  "2818baa6-9263-5133-a102-5279845967fc"; // Owner ID Ricardo
//char deviceID[37] = "e11cdf9f-61cb-5abe-84c7-ed71fff2254b"; // SIMULACION
//char deviceID[37] = "fd5c8844-c060-56f9-9772-4c6467572a10"; //  Device ID Ricardo
//char deviceID[37] =   "d8423351-0bfe-52e6-b93a-7f28618ec9ff"; //  Device ID Carro01
//char deviceID[37] =     "120d24cb-7712-5463-8fd7-3109d0206d70"; //  Device ID Carro02
//char deviceID[37] =     "242ad15c-a2d7-50c3-9fe3-e59daaab0c88";// Carro 04

//Timer reset values for tasks
//uint16_t TC4_time = 0x003C; // debug valuet
//uint16_t TC5_time = 0x0100; // debug value
uint16_t TC4_time = 0x0096; // 150 second
//uint16_t TC4_time = 0x012C; // 300 second
//uint16_t TC5_time = 0x012C; // 300 second (5 minutes)
uint16_t TC5_time = 0x0257; // 600 second (10 minutes)
uint16_t Tcount;

// variables
char replybuffer[255]; // this is a large buffer for replies
char message[255]; 
char command[10];
char newpass[9];
char readpass[9];
char delim[2] = " "; // delimiter for parsing

// obdflag
// 0: Default (no system selected)
// 1: ISO 9141 (slow)
// 2: KWP (fast 9141)
// 3: CAN Bus 11-bit header regular PIDs
// 4: SIMULATION
// 5: CAN Bus 11-bit Suzuki
// 6: CAN Bus 29-bit header, regular PIDs (not implemented yet)
// 7: Suzuki KWP (reads all parameters in a single call)

#define ISO9141SLOW 1
#define ISO9141KWP 2
#define CANBUS 3
#define SIMULATION 4
#define CANSUZ 5
#define CANBUS29 6
#define KWPSUZ 7

uint8_t obdflag = ISO9141SLOW;

char serialcommand = 0;
double simbatvolt = 4.2; // variable to simulate battery voltage
bool pwrgatestatus = false;
bool chargersentstatus = false;

unsigned long elapsedmillis = 0;
unsigned long startmillis = 0; 
double elapsedtime = 0;

// Hardware serial port to FONA
HardwareSerial *fonaSerial = &Serial1;

// Hardware port for ISO 9141
HardwareSerial *isoSerial = &Serial2;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA();

int count; // iterator

uint16_t vbat;

//create the CANport acqisition schedulers
//cAcquireCAN CANport0(CAN_PORT_0);

// ISO 9141
OBD9141 obd9141;

uint8_t FuelTypeCount = 0;
bool  bFuelType = 0;// 0 for gasoline, 1 for LPG
int   LPGStatusRead = 0; // Analog read from valve
bool init_success = false;
bool postflag = false;
bool lastpost = false; // use to post a final message
bool engineflag = false;
bool GPSflag = false; // Ricardo - 28 Mar 2020 add GPS status flag
bool SIMflag = false; // Ricardo 21 Mar 2020

double fSpeed = 0; // Km per hour
double fPreviousSpeed = 0;
double fRPM = 0;   // Revs per minute
double fIMAP = 0;  //Internal Manifold absolute pressure
double fIAT = 0;   //Internal Air Temperature
double fAir = 0;   // Grams of Air (calculated)
double fLPH = 0;   // Liters per Hour
double fKPL = 0;   // Kilometers per Liter
// Ricardo Oct 10 - KPL averaging is not working for pilot - will calculate distance using speed * time and upload to website instead of KPL
double fLPGKm __attribute__ ((section (".noinit")));
double fGasKm __attribute__ ((section (".noinit")));

double const MAFConst = 0.029025;
double const LPHGasConst = 0.333648;

double fGasLambda = 14.7;
double fLPGLambda = 15.67;
double fGasDens = 734;
double fLPGDens = 536.3;
const double SecPerHour = 3600;
double fGasTime __attribute__ ((section (".noinit")));
double fLPGTime __attribute__ ((section (".noinit")));
double fGasPerc __attribute__ ((section (".noinit")));
double fLPGPerc __attribute__ ((section (".noinit")));
double fAvgGasLPH __attribute__ ((section (".noinit")));
double fAvgGasKPL __attribute__ ((section (".noinit")));
double fAvgLPGLPH __attribute__ ((section (".noinit")));
double fAvgLPGKPL __attribute__ ((section (".noinit")));
double fSumGasLPH __attribute__ ((section (".noinit")));
double fSumGasKPL __attribute__ ((section (".noinit")));
double fSumLPGLPH __attribute__ ((section (".noinit")));
double fSumLPGKPL __attribute__ ((section (".noinit")));
double nSamples __attribute__ ((section (".noinit")));
double fGasLiters __attribute__ ((section (".noinit")));
double fLPGLiters __attribute__ ((section (".noinit")));
double            ftempGasTime ;
double            ftempLPGTime ;
double            ftempGasKm ;
double            ftempLPGKm ;
double            ftempSumGasLPH;
double            ftempSumGasKPL ;
double            ftempSumLPGLPH ;
double            ftempSumLPGKPL;
double            ntempSamples ;
double            ftempGasLiters ;
double            ftempLPGLiters ;
// Ricardo - eliminated the Liters per Second metric

// GPS variables
float lat, lon, speedkph,heading, altitude;
float lastlat __attribute__ ((section (".noinit")));
float lastlon __attribute__ ((section (".noinit")));  // to track last available coordinates
char gpsdate[20];
bool gpslock = false;  // Wait until first GPS lock to disable and sleep
bool FonaSleep = false; // To track if FONA is asleep or not
uint8_t batreads = 0; // allow 2 measures of charging before turning charger on or off

char data[400];
PString pdata(data,400);

//Variables for DayClosingPost
bool alreadyPosted = false;

// data to be saved in non volatile memory
char password[9] = "87654321";
char adminpass[9] = "qwertyui";
bool relay = true;
double EngineVol = 1.6; 
double VE = 0.85;


//RTC variables
RTCZero rtc;
char timestamp[25];
byte secs,mins,hrs,day,month,year;

// File Variables
File datafile;
char filename[13] = "00000.txt";
word filecount = 0;
char filebuffer[16240];
PString pfilebuffer(filebuffer,16240);
char stringbuffer[255];

// Ricardo 7 July 2020 - variables for interrupts
volatile bool IGN_Det = false;
volatile bool TC4trig = false;
volatile bool TC5trig = false;

// Variables for phonebook
char PHnumber[20];
char PHtext[15];

// Watchdog
bool wdtwake = false; // used to skip cycle and go to sleep without further action
WDTZero wdt;

// OBD2 autodetect
uint8_t obdtries = 0;
bool obdfound = true;  //disable auto recognition for now

// Ricardo 22 June 2020 - remove all print log functions other than chars
// any numeric data passed to logfile or USB should be sprintf'd first

// Ricardo 5 August 2020 - CAN bus variables
bool useStandardAddressing = true;
uint16_t CANtimeout = 0;
uint16_t dev = 0x7e0; // ECU0 device ID 

// 11 OCT 2020 added flags to check SMS and HTTP message sent asynchronous
bool SMSsent = false;
bool HTTPsending = false;
//25 Nov 2020 SMS send queue
uint8_t SmsToSend = 0;

char sender1[15];
char sender2[15];
char sender3[15];

char sms1[140];
char sms2[140];
char sms3[140];
#define SMSphone "84050685"
int SMStimeout;

/* 	Ricardo 7 July 2020 - added functions to R/W to SIM memory for non volatile vars
	As following:
	VARNAME		INDEX		NUMBER/TEXT
	password	1			TEXT
	adminpass	2			TEXT	
	ownerID.1	3			TEXT	Owner and device ID are stored in 3 parts based on size
	ownerID.2	4			TEXT
	ownerID.3	5			TEXT
	deviceID.1	6			TEXT
	deviceID.2	7			TEXT
	deviceID.3	8			TEXT
	relay		9			NUMBER (0 off/ 1 on)
	OBDType		10			NUMBER
	EngineVol	11			NUMBER	Divide by 10 to obtain liters
	VE			12			NUMBER	Divide by 100 to obtain variable
*/

void printlog (const char *line) {
  SerialUSB.print(line);
  pfilebuffer.print(line);
  
  if (pfilebuffer.length() > 16150) {
	  pfilebuffer.begin();
	  SDWriteFile();
  }
}

void printlogln (const char *line) {
  SerialUSB.println(line);
  pfilebuffer.print(line);
  pfilebuffer.print("\n");
  if (pfilebuffer.length() > 16150) {
	  SDWriteFile();
    memset(filebuffer,0,strlen(filebuffer));
    pfilebuffer.begin();
  }

}

void setup() {
  // put your setup code here, to run once:s
//  while (!SerialUSB);
  SerialUSB.begin(115200);
  rtc.begin();

  pinMode(LPGSTATUSPIN,INPUT);
  pinMode(PIN_ON, OUTPUT);
  pinMode(PIN_OFF, OUTPUT);
  pinMode(FONA_RST, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(LIN_SLP,OUTPUT);
  pinMode(SD_DT,INPUT);
  pinMode(IGN_DET,INPUT);
  pinMode(DTR,OUTPUT);
  pinMode(PWR_GATE,OUTPUT);
  pinMode(PWR_GPS,OUTPUT); 
  pinMode(CAN_RST,OUTPUT);

  digitalWrite(CAN_RST,HIGH);  
  digitalWrite(LIN_SLP, HIGH);
  digitalWrite(DTR, LOW);
  
	// FONA init
	pinMode(FONA_RST, OUTPUT);
	pinMode(FONA_PWRKEY, OUTPUT);

	digitalWrite(FONA_RST, HIGH);
  digitalWrite(PWR_GATE, LOW);

  
  if (SerialUSB.available()) {
    serialcommand = SerialUSB.read();
    if (serialcommand == '1') {
      runconfig();
    }
  }

  
  

  if (PM->RCAUSE.reg == PM_RCAUSE_WDT) {
    
     printlogln("WDT Reset - keeping variables in place");
  }
  else {
    SerialUSB.println("Wait 8 seconds to begin\n");
    delay(8000);
    printlogln("Standard Reset - init all vars to 0");
    fGasTime = 0;
    fLPGTime = 0;
    fGasPerc = 0;
    fLPGPerc = 0;
    fAvgGasLPH = 0;
    fAvgLPGLPH = 0;
    fSumGasLPH = 0;
    fSumLPGLPH = 0;
    nSamples = 0;
    fGasLiters = 0;
    fLPGLiters = 0;
    lastlat = 0;
    lastlon = 0;
    fLPGKm = 0;
    fGasKm = 0;
    fAvgGasKPL = 0;
    fAvgLPGKPL = 0;
    fSumGasKPL = 0;
    fSumLPGKPL = 0;
    pfilebuffer.begin();
  }
   printlogln("\n\n*****  ECU GPS INIT ******");
  
   fonaSerial->begin(38400);
   fona.initPort(*fonaSerial);
   
   //SimTogglePWRKEY();

    printlogln("Resetting SIM808..");
    pinMode(FONA_RST, OUTPUT);
    digitalWrite(FONA_RST, HIGH);
    delay(10);
    digitalWrite(FONA_RST, LOW);
    delay(100);
    digitalWrite(FONA_RST, HIGH);


   while (! fona.begin()) {
     printlogln("Couldn't find FONA - retry");
   }

	SIMflag = true;
	printlogln("FONA is found OK");


	// Ricardo 7 July 2020 - removed second reset, was added to fona.begin code with check to the ECHARGE status
	
	// Print module IMEI number.
	fona.getIMEI(replybuffer);
	printlog("Module IMEI: "); printlogln(replybuffer);
	fona.getSIMCCID(replybuffer);
	printlog("SIM CCID = "); printlogln(replybuffer);
  // Turn on GPS
  if (!fona.enableGPS(true))
    printlogln("Failed to turn on GPS!");
  else {
    digitalWrite(PWR_GPS,LOW);
    printlogln("GPS Enabled");
    GPSflag = true;
  }
  
	delay(3000);
	// Check the non volatile vars located in SIM card
	// If value is blank, default value is left there
	if (! fona.ReadPhonebook(1,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 1");
	} else {
		if (PHtext == "") {
			printlogln("SIM Card Index is blank, using default password");
		} else {
			strncpy(password,PHtext,8);
			printlog("Read password from SIM card: ");
			printlogln(password);
		}
	}
	if (! fona.ReadPhonebook(2,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 2");
	} else {
		if (PHtext == "") {
			printlogln("SIM Card Index is blank, using default adminpass");
		} else {
			strncpy(adminpass,PHtext,8);
			printlog("Read adminpass from SIM card: ");
			printlogln(adminpass);
		}
	}
	// Read owner ID and device ID
	if (! fona.ReadPhonebook(3,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 3");
	} else {
		if (PHtext == "") {
			printlogln("SIM Card Index is blank");
		} else {
			strncpy(message,PHtext,14);
			while (! fona.ReadPhonebook(4,PHnumber,PHtext)) {
				printlogln("Failed to read PhoneBook 4");
        delay(500);
			} 
			strncpy(message+14,PHtext,14);
			while (! fona.ReadPhonebook(5,PHnumber,PHtext)) {
				printlogln("Failed to read PhoneBook 5");
        delay(500);
			}
			strncpy(message+28,PHtext,8);
			strncpy(ownerID,message,36);
			printlog("Read owner ID from SIM card: ");
			printlogln(ownerID);
		}
	}
	if (! fona.ReadPhonebook(6,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 6");
	} else {
		if (PHtext == "") {
			printlogln("SIM Card Index is blank");
		} else {
			strncpy(message,PHtext,14);
			while (!fona.ReadPhonebook(7,PHnumber,PHtext)) {
				printlogln("Failed to read PhoneBook 7");
        delay(500);
			}
		  strncpy(message+14,PHtext,14);
		  while (! fona.ReadPhonebook(8,PHnumber,PHtext)) {
				printlogln("Failed to read PhoneBook 8");
        delay(500);
			} 
			strncpy(message+28,PHtext,8);
			strncpy(deviceID,message,36);
			printlog("Read deviceID from SIM card: ");
			printlogln(deviceID);
		}
	}
	// Relay read and process
	if (! fona.ReadPhonebook(9,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 9");
	} else {
		if (PHnumber == "") {
			printlogln("SIM Card Index is blank, using default relay");
		} else {
			relay = atoi(PHnumber);
			sprintf(stringbuffer,"Read relay from SIM card: %d",relay);
			printlogln(stringbuffer);
		}
	}
	if (relay) {
      printlogln("Checking power relay is ON");
      digitalWrite(PIN_OFF, LOW);
      digitalWrite(PIN_ON, HIGH);
	} else {
      printlogln("Checking power relay is OFF");
      digitalWrite(PIN_ON, LOW);
      digitalWrite(PIN_OFF, HIGH);
	}
	delay(500);
	printlogln("turn off relay transistors");//Daniel 1/3/2020. Relay set to low save energy
	digitalWrite(PIN_OFF, LOW);
	digitalWrite(PIN_ON, LOW);

	// OBD type read and process
	if (! fona.ReadPhonebook(10,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 10");
	} else {
		if (PHnumber == "") {
			printlogln("SIM Card Index is blank, using default obdflag");
		} else {
			obdflag = atoi(PHnumber);
			sprintf(stringbuffer,"Read obdflag from SIM card: %d",obdflag);
			printlogln(stringbuffer);
		}
	}
	
	// Engine volume read and process
	if (! fona.ReadPhonebook(11,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 11");
	} else {
		if (PHnumber == "") {
			printlogln("SIM Card Index is blank, using default engine volume");
		} else {
			EngineVol = atof(PHnumber);
			EngineVol = EngineVol / 10;
			sprintf(stringbuffer,"Read EngineVol from SIM card: %.2f",EngineVol);
			printlogln(stringbuffer);
		}
	}
	
	// Engine volume read and process
	if (! fona.ReadPhonebook(12,PHnumber,PHtext)) {
         printlogln("Failed to read PhoneBook 12");
	} else {
		if (PHnumber == "") {
			printlogln("SIM Card Index is blank, using default volumetric efficiency");
		} else {
			VE = atof(PHnumber);
			VE = VE / 100;
			sprintf(stringbuffer,"Read VE from SIM card: %.2f",VE);
			printlogln(stringbuffer);
		}
	}

	// Configure a GPRS APN, username, and password.
	fona.setGPRSNetworkSettings(F("internet.movistar.cr"), F("movistarcr"), F("movistarcr"));

	//configure HTTP gets to follow redirects over SSL
	fona.setHTTPSRedirect(true);


	 

   // TO DO: change startup method based on jumper configuration
     if (obdflag == SIMULATION) {  // SIMULATION
        printlogln("OBD in simulation mode");
     }
     else if ((obdflag == CANBUS) || (obdflag == CANSUZ)){
         //start CAN ports,  enable interrupts and RX masks, set the baud rate here
         printlogln("OBD Port Type: CAN");
          // start the CAN bus at 500 kbps
        if (!CAN.begin(500E3)) {
          sendText(SMSphone,"Starting CAN failed!");
        }
        // Investigate use CAN MCP2515 filter later
        // add filter to only receive the CAN bus ID's we care about
        //if (useStandardAddressing) {
        //  CAN.filter(0x7e8);
        //} else {
        //  CAN.filterExtended(0x18daf110);
        //}
        
      }
      else if (obdflag == ISO9141KWP) {
        printlogln("OBD Port Type: ISO 9141 fast (KWP)");
        obd9141.begin(Serial2, ISO_RX, ISO_TX);
        
      } else if (obdflag == ISO9141SLOW) {
        printlogln("OBD Port Type: ISO 9141 slow");
        obd9141.begin(Serial2, ISO_RX, ISO_TX);
        
      }
      else if (obdflag == KWPSUZ) {
        printlogln("OBD Port Type: Suzuki KWP");
        obd9141.begin(Serial2, ISO_RX, ISO_TX);
        
      }


   // Timer setup
   TCsetup();
   SdFile::dateTimeCallback(SDdateTime);
   // WDT setup
   wdt.attachShutdown(wdt_reset);
   wdt.setup(WDT_SOFTCYCLE8M);
   // Interrupt setup  
   LowPower.attachInterruptWakeup(digitalPinToInterrupt(IGN_DET), Ignition, CHANGE);
   startmillis = millis(); // discount setup time from delta.
   printlogln("*****  Init End  ******\r\n");
   sendText(SMSphone,"ECU-GPS booted OK");
   while (SMStimeout > 0) {
      checkSMSsend();
      if (SmsToSend == 0) break;
      delay(1000);

   }
   ProcessSMS();
   
   // Time from RTC
   rtctimestamp(timestamp);
   printlog("Current Time: "); printlogln(timestamp);
   
   IGN_Det = digitalRead(IGN_DET);
   ReadOBD();
}

////////////////////////////////////////////////////////////////////////////////
void loop() {
  // start the main function at the interval
  // Ricardo 6/6/20 - added CPU sleep time in case engine is off
  //				- Added code to use counters to trigger SMS and HTTP functions
  //SerialUSB.println("main loop start");
  if (!engineflag) {
    while (SmsToSend > 0) {
      checkSMSsend();
      SerialUSB.println("Wait in main loop for SMS to send..");
      if (IGN_Det) break;
      delay(500); 
      wdtwake = false;
    }
    if (!IGN_Det) {
      //SerialUSB.println("DEBUG: Before sleep");
      // Only go to sleep if not waiting for SMS to be sent
      if (!wdtwake) {
  	    printlogln("** Engine OFF - sleep on main loop");
  	    SerialUSB.end();
      }
      wdtwake = false;
  	  LowPower.sleep();
      if (wdtwake) return;
    
      // Resume from Wake-up
      SerialUSB.begin(115200);
      delay(1000); // extra delay to allow to print to USB
      printlogln("\r\n** INT trigger and wake");
    }
  
  
  }
 
  //SerialUSB.println("DEBUG: Before ReadOBD");
	ReadOBD();
	if (TC4trig) {
		SerialUSB.println("TC4 triggered!\n");
		TC4trig = false;
	  ProcessSMS();
	}

	if (TC5trig) {
		SerialUSB.println("TC5 triggered!\n");
		TC5trig = false;
	  HTTPPost();
	}
  //SerialUSB.println("DEBUG: Before DayClosingPost");

	DayClosingPost(); //Last post at the end of the day
	//SerialUSB.println("DEBUG: After DayClosingPost");

  if (SmsToSend > 0) {
     checkSMSsend();
  }
   
}

void ReadOBD() {
  bool res;
  uint8_t numtries = 0;
    
  // Ricardo 9 March 2020 - moved ellapsed millis to closer to the actual calculation, time is lost during reading so it didnt add up
  // Ricardo 21 June 2020 - use Ignition detect signal to get the engine status
  //
  if (!IGN_Det) {

    if (!SIMflag) {
        
        printlogln("Turn DTR OFF - SIM out of power saving");
        digitalWrite(DTR,LOW);
        digitalWrite(LIN_SLP, HIGH);
        delay(200);
        SIMflag = true;
         
    }
     
    if (engineflag) { // Ricardo 24 March 2020 - include an engine on check since car could have turned off
      printlogln("* Engine off - Flag for posting");
      sendText(SMSphone,"Engine is turned OFF");
      lastpost = true;
      postflag = true;
      engineflag = false;
    }

    
  
  } // END IGN_Det == false
  else {  // IGN_Det == true

    if (!engineflag) {
  	  rtctimestamp(timestamp);
  	  printlog("Current Time: "); printlogln(timestamp);
      startmillis = millis(); // Start counter only when engine is on (first time)
      engineflag = true;
      digitalWrite(PWR_GATE,HIGH); // DISABLE charger to allow car to start up properly! - 25 june 2020
      printlogln("DISABLE charger to allow car to start up properly!");
      pwrgatestatus = true;
      chargersentstatus = false;
      init_success = false;
  	  FuelTypeCount = 0; // track a few cycles (30) of OBD to account for gasoline
      fPreviousSpeed = 0;  // to help in calculating actual distance
  	  
  	  printlogln("Turn DTR OFF - SIM out of power saving");
  	  digitalWrite(DTR,LOW);
      digitalWrite(LIN_SLP, HIGH);
  	  delay(50);
  	  SIMflag = true;
	    sendText(SMSphone,"Engine is turned ON");  
	    printlogln("Fuel\t\tRPM\tSpeed\tIMAP\tIAT\tAirGr\tms\tL/Hr\tKmGas\tKmLPG\tGasTime\tLPGTime\tGas%\tLPG%\tGas_L\tLPG_L\tAvgGas_LPH\tAvgLPG_LPH");
    }
    if ((SmsToSend == 0) && !HTTPsending) {
      // Turn on GPS in case it was disabled
      if (GPSflag == false) {
         if (!fona.enableGPS(true))
          printlogln("Failed to turn ON GPS!");
         else {
          digitalWrite(PWR_GPS,LOW);
          printlogln("GPS Enabled!");
          GPSflag = true;
         }
      }
    }
    
	// Ricardo 26 June 2020 - Moved OBD types inside the reading logic, including simulation
  // Moved Fuel type read inside with engine data  
    
  if ((obdflag == ISO9141SLOW) || (obdflag == ISO9141KWP) || (obdflag == KWPSUZ)) {
  	
  	while (!init_success && (numtries < 4)) {
  		if (obdflag == ISO9141KWP) {
  		   printlog("KWP Init - ");
  		   init_success =  obd9141.initKWP(); // Aveo uses KWP baud init
  		   
  		}
  		else if (obdflag == ISO9141SLOW) {
  		   printlog("50 baud Init - ");
  		   init_success =  obd9141.init(); // crossfox uses 50 baud init        
  		}
     else if (obdflag ==7) {
        printlog("Suzuki KWP Init - ");
        init_success =  obd9141.initSZ(); 
     }
  		
  		sprintf(stringbuffer,"  OBD2 init success: %d",init_success);
  			printlogln(stringbuffer);
  		if (!init_success) {
  		  numtries++;
  		}
  		delay(200); // Added delay on rev2 - Ricardo 21 June 2020
  	}
  	if (!init_success && (numtries == 4)) {
  		printlogln("OBD2 init not OK!! ");
      if (obdfound == false) { // automatic finding of OBD type 
        obdtries++;
        if (obdtries == 10) {
          printlog("OBD2 init tried 10 times no response - changing OBD2 type: ");
          obdtries = 0;
          if (obdflag == ISO9141SLOW) {
            obdflag = 2;
            printlogln("KWP");
          }
          else if (obdflag == ISO9141KWP) {
            obdflag = 3;
            printlogln("CAN");
          }
        }
      }
  		return;
  	}
  	numtries = 0;
  }
  else if (obdflag == SIMULATION) {
    //printlogln("Simulation - Init OK");
    init_success = 1;
  }
	res = false;
	while (!res && (numtries < 4)) {
		if (obdflag == SIMULATION) {
			res = 1;
		}
		else if (obdflag == CANBUS) {
      // CAN BUS
      if (pwrgatestatus) {
          delay(2000); // CAN requires charger and 5V line to be ON.          
          digitalWrite(PWR_GATE,LOW); // ENABLE charger once the engine is started - 25 june 2020
          pwrgatestatus = false;
          printlogln("Charger is now Enabled after RPM detected");
        }
         
      if (useStandardAddressing) {
        SerialUSB.println("\t ** CAN begin Packet");
        CAN.beginPacket(0x7df, 8);
      } else {
        SerialUSB.println("\t ** CAN begin extended Packet");
        CAN.beginExtendedPacket(0x18db33f1, 8);
      }
      SerialUSB.println("\t ** CAN write 02 01 0C: RPM");
      CAN.write(0x02); // number of additional bytes
      CAN.write(0x01); // show current data;
      CAN.write(RPM); // engine RPM
      CAN.endPacket();
      SerialUSB.println("Sent CAN package");
      CANtimeout = 500;
      // wait for response
      while (CAN.parsePacket() == 0 ||
             CAN.read() < 3 ||          // correct length
             CAN.read() != 0x41 ||      // correct mode
             CAN.read() != RPM  )        // correct PID
      {
        delay(1);
        CANtimeout--;
        if  (CANtimeout == 0) {
          SerialUSB.println("CAN Bus read timeout!");
          res = false;

          if (obdfound == false) { // automatic finding of OBD type 
            obdtries++;
            if (obdtries == 10) {
              printlog("OBD2 CAN tried 10 times no response - changing OBD2 type: ISO9141 SLOW");
              obdflag = 1;
            }           
          }
          break;
        }
      
      } // end while
      if (CANtimeout != 0) {
        fRPM = ((CAN.read()*256.0) + CAN.read())/4;
        res = true;
      }
    }
    else if (obdflag == CANSUZ) {  // Suzuki rev eng data
        if (pwrgatestatus) {
          delay(2000); // CAN requires charger and 5V line to be ON.          
          digitalWrite(PWR_GATE,LOW); // ENABLE charger once the engine is started - 25 june 2020
          pwrgatestatus = false;
          printlogln("Charger is now Enabled after RPM detected");
        }
      
      
        // Send request for all data
          
          CAN.beginPacket(dev, 8);
          CAN.write(0x02); // number of additional bytes
          CAN.write(0x21);
          CAN.write(0x00);   
          CAN.endPacket();
          
          CAN.beginPacket(dev, 8);
          CAN.write(0x30); // number of additional bytes
          CAN.endPacket();

          bool x25found = false;
          bool x26found = false;
          unsigned long cantime = millis();
          while (!(x25found && x26found)) { // need to receive both messages before continue
            int DLC = 0;
            while( DLC == 0) {
              DLC = CAN.parsePacket();
              if (millis() - cantime > 500) break;
            }
            if (CAN.packetId() == 0x7E8) {
              uint8_t bitid;
              bitid = CAN.read();
              if (bitid == 0x25) {
                CAN.read();
                CAN.read();
                CAN.read();
                fRPM = CAN.read() *256;
                fRPM = (fRPM + CAN.read())/4;
                x25found = true;
              }
              else if (bitid == 0x26) {
                CAN.read();
                fSpeed = CAN.read();
                fIMAP = CAN.read();
                CAN.read();
                fIAT = CAN.read() - 40;
                x26found = true;
              }
                
            }
          
            if (millis() - cantime > 500) break;
               
          }
          if (x25found && x26found) res = true;

    }
    else if (obdflag == KWPSUZ) {
      res = obd9141.requestSZ(&fRPM, &fIAT,&fIMAP,&fSpeed);

      // Data validation to avoid spurious
      if ((fRPM < 500) || (fIAT <= 0) || (fIMAP <= 0)) {
        printlogln("Read invalid, return!");
        return;     
      }
    }
    else {
		  res = obd9141.getCurrentPID(RPM,_16BITS);
    }
		
		numtries++;
		delay(50);
	}
	if (!res) {
		printlogln("** RPM reading invalid");
		delay(500);
    if ((obdflag == ISO9141SLOW) || (obdflag == ISO9141KWP) || (obdflag == KWPSUZ))  {
  		init_success = false;
  		numtries = 0;
  		while (!init_success && (numtries < 2)) {
  			if (obdflag == ISO9141KWP) {
  			   printlog("KWP Init - ");
  			   init_success =  obd9141.initKWP(); // Aveo uses KWP baud init
  			   
  			}
  			else if (obdflag == ISO9141SLOW) {
  			   printlog("50 baud Init - ");
  			   init_success =  obd9141.init(); // crossfox uses 50 baud init        
  			}
       
         else if (obdflag == KWPSUZ) {
           printlog("Suzuki KWP Init - ");
           init_success =  obd9141.initSZ(); 
        }
  			sprintf(stringbuffer,"  OBD2 init success: %d",init_success);
  			printlogln(stringbuffer);
  			if (!init_success) {
  			  numtries++;
  			}
  			delay(200); // Added delay on rev2 - Ricardo 21 June 2020
  		}
  		if (!init_success && (numtries == 2)) {
  			printlogln("OBD2 init not OK!! - return");
  			return;
  		}
  		res = obd9141.getCurrentPID(RPM,_16BITS);
  		if (!res) {
  		  printlogln("RPM reading still not valid - return");
  		  return;
  		}
    } // if (obdflag != 3)
    else {
      printlogln("Read invalid, return!");
      return;            
    }
	} //if (!res)

	// RPM reading is valid!

		if (obdflag == SIMULATION) {
			fRPM = random (5000);
		} else if ((obdflag == ISO9141SLOW) || (obdflag == ISO9141KWP)) {
			fRPM = obd9141.readUint16()/4;
		}
		if (fRPM == 0) {
		  // Ricardo - 21 June 2020 RPM 0 and ignition means engine is off, skip calculation 
		  printlogln("Ignition ON but Engine OFF - skip calculation");
		  engineflag = false; // Engine off
		  return;
		  
		} else {
      obdfound = true; // add to indicate the current OBD comm method is valid
		  engineflag = true;
		  postflag = true; // only do a fuel calculation and HTTP post if the engine is on
				   
		  if (pwrgatestatus) {
			digitalWrite(PWR_GATE,LOW); // ENABLE charger once the engine is started - 25 june 2020
			pwrgatestatus = false;
			printlogln("Charger is now Enabled after RPM detected");
		  }
      
		  // Read LPG  pin 
		  LPGStatusRead = digitalRead(LPGSTATUSPIN);
		  if (obdflag == SIMULATION) {
			  LPGStatusRead = random(1);
		  }
		  if (FuelTypeCount < 30) { // Have 30 cycles of fuel at startup accounted for gasoline
			  LPGStatusRead = LOW;
			  FuelTypeCount++;
		  }
		  if (LPGStatusRead == HIGH) {
			// LPG is on
  			bFuelType = true;
  			printlog("LPG\t\t");
		  } else {
			//LPG is off
  			bFuelType = false;
  			printlog("Gas\t\t");
		  }
      		  
		}

		delay(100);
		if (obdflag == SIMULATION) {
			// Simulation
			fSpeed = random(100);
			fIMAP = random(20,50);
			fIAT = random(25,50);
			delay(500);
		}
    else if (obdflag == CANBUS) {
      // CAN
      
        // SPEED
        if (useStandardAddressing) {
          SerialUSB.println("\t ** CAN begin Packet");
          CAN.beginPacket(0x7df, 8);
        } else {
          SerialUSB.println("\t ** CAN begin extended Packet");
          CAN.beginExtendedPacket(0x18db33f1, 8);
        }
        CAN.write(0x02); // number of additional bytes
        CAN.write(0x01); // show current data;
        CAN.write(SPEED);
        CAN.endPacket();
        CANtimeout = 500;
        // wait for response
        while (CAN.parsePacket() == 0 ||
               CAN.read() < 3 ||          // correct length
               CAN.read() != 0x41 ||      // correct mode
               CAN.read() != SPEED  )        // correct PID
        {
          delay(1);
          CANtimeout--;
          if  (CANtimeout == 0) {
            SerialUSB.println("CAN Bus read timeout!");
            break;
          }
        
        } // end while
        if (CANtimeout != 0) {
          fSpeed = CAN.read();
        }

        //   IMAP 
        
        if (useStandardAddressing) {
          SerialUSB.println("\t ** CAN begin Packet");
          CAN.beginPacket(0x7df, 8);
        } else {
          SerialUSB.println("\t ** CAN begin extended Packet");
          CAN.beginExtendedPacket(0x18db33f1, 8);
        }
        CAN.write(0x02); // number of additional bytes
        CAN.write(0x01); // show current data;
        CAN.write(IMAP);
        CAN.endPacket();
        CANtimeout = 500;
        // wait for response
        while (CAN.parsePacket() == 0 ||
               CAN.read() < 3 ||          // correct length
               CAN.read() != 0x41 ||      // correct mode
               CAN.read() != IMAP  )        // correct PID
        {
          delay(1);
          CANtimeout--;
          if  (CANtimeout == 0) {
            SerialUSB.println("CAN Bus read timeout!");
            break;
          }
        
        } // end while
        if (CANtimeout != 0) {
          fIMAP = CAN.read();
        }

        //   IAT 
        if (useStandardAddressing) {
          SerialUSB.println("\t ** CAN begin Packet");
          CAN.beginPacket(0x7df, 8);
        } else {
          SerialUSB.println("\t ** CAN begin extended Packet");
          CAN.beginExtendedPacket(0x18db33f1, 8);
        }
        CAN.write(0x02); // number of additional bytes
        CAN.write(0x01); // show current data;
        CAN.write(IAT);
        CAN.endPacket();
        CANtimeout = 500;
        // wait for response
        while (CAN.parsePacket() == 0 ||
               CAN.read() < 3 ||          // correct length
               CAN.read() != 0x41 ||      // correct mode
               CAN.read() != IAT  )        // correct PID
        {
          delay(1);
          CANtimeout--;
          if  (CANtimeout == 0) {
            SerialUSB.println("CAN Bus read timeout!");
            break;
          }
        
        } // end while
        if (CANtimeout != 0) {
          fIAT = CAN.read() - 40;
        }
      
    }
		else if ((obdflag == ISO9141KWP) || (obdflag ==1)) {
			// OBD9141 (fast and slow)
			res = obd9141.getCurrentPID(SPEED,_8BITS);
			if (res) {
				fSpeed = obd9141.readUint8();
			}
			
			delay(100);
			
			res = obd9141.getCurrentPID(IMAP,_8BITS);
			if (res) {
				fIMAP = obd9141.readUint8();
			}
			
			delay(100);
			res = obd9141.getCurrentPID(IAT,_8BITS);
			if (res) {
			  fIAT = obd9141.readUint8() - 40;
			}
		}

	
    
    fAir = fRPM * fIMAP / (fIAT+273) * EngineVol * VE * MAFConst;

    // Ricardo - Moved ellapsed millis here to include time spent during engine readings
    elapsedmillis = millis() - startmillis; // uses startmillis from previous cycle
    startmillis = millis();
	  
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
      fLPGKm += (fSpeed + abs(fPreviousSpeed-fSpeed)/2) * elapsedmillis/3600000;      // added cumulative of Km instead of Km/l 10/12/2020
      fLPGTime += (double)elapsedmillis/3600000; // elapsed time in hours, changed by Ricardo 14/2/20
      fLPGLiters += fLPH * (double)elapsedmillis/3600000;
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
      fGasKm += (fSpeed + abs(fPreviousSpeed-fSpeed)/2) * elapsedmillis/3600000;      // added cumulative of Km instead of Km/l 10/12/2020
      fGasTime += (double)elapsedmillis/3600000; // elapsed time in hours, changed by Ricardo 14/2/20
      fGasLiters += fLPH * (double)elapsedmillis/3600000;
    }
    fPreviousSpeed = fSpeed;
    fGasPerc = 100* fGasTime / (fGasTime + fLPGTime);
    fLPGPerc = 100* fLPGTime / (fGasTime + fLPGTime);

        
    sprintf(stringbuffer,"%4.0f\t%.0f\t%.0f\t%.0f\t%.3f\t%d\t%.2f\t%.3f\t%.3f\t%.3f\t%.3f\t%.2f\t%.2f\t%.3f\t%.3f\t%.2f\t%.2f",fRPM,fSpeed,fIMAP,fIAT,fAir,elapsedmillis,fLPH,fGasKm,fLPGKm,fGasTime,fLPGTime,fGasPerc,fLPGPerc,fGasLiters,fLPGLiters,fAvgGasLPH,fAvgLPGLPH);
    printlogln(stringbuffer);

  } // END // IGN_Det == true
  
  
} // ReadOBD
  

void ProcessSMS () {
	
	char sms[140];
	PString psms(sms,140);
	replybuffer[0] = '\0';
	char sender[15];

  rtctimestamp(timestamp);
  printlogln(timestamp);
  
	printlogln("**** SMS Process *****");
  wdt.clear();
  if (SIMflag == false) {
	
  	printlogln("Turn DTR OFF - SIM out of power saving");
  	digitalWrite(DTR,LOW);
    digitalWrite(LIN_SLP, HIGH);
  	delay(500);
  	SIMflag = true;
  }

  init_success == false; // redo a init for OBD2 9141 types
  // Check if SMS is still under way before proceed
  if (SmsToSend > 0) {
    printlogln("SMS still outbound - retrigger SMS check");
    TC4trig = true;
    return;
  }

      
  GetGPSandTime();
  BattMeasure();

  while (SmsToSend > 0) {
      checkSMSsend();
      printlogln("Wait in SMS Process for SMS to send..");
      if (IGN_Det) {
        printlogln("Engine On! Return");
        return;
      }
      delay(500);
  }
  // Read # of SMS, process them all
  uint16_t smslen;
  count = fona.getNumSMS();
  
  if (count == 0) {
	  printlogln("No SMS received");
  }
  else if (count < 0) {
	  printlogln("ERROR: Could not read # SMS");
  }
  else {
	  if (count > 0) {
  	  sprintf(stringbuffer,"# of SMS received: %d",count);
  	  printlogln(stringbuffer);
  
  	  int smsid = count;
  	  bool smsfound = false;
  	  while (smsid > 0) { // Change
        
        while (SmsToSend > 0) {
            checkSMSsend();
            printlogln("Wait in SMS Process for SMS to send..");
            if (IGN_Det) {
              printlogln("Engine On! Return");
              return;
            }
            delay(500);
        }
		  
        sprintf(stringbuffer,"Read SMS # %d",smsid);
        printlogln(stringbuffer);
  			if (! fona.readSMS(smsid, replybuffer, 250, &smslen)) { // Read message #1 pass in buffer and max len!
  			  printlogln("ERROR Failed readSMS");        
  			}
  			else {
  			  strcpy(message,replybuffer);
  			  sprintf(stringbuffer,"Reading SMS # %d",smsid);
  			  printlogln(stringbuffer);
  			  smsfound = true;

  			}
		    if (smsfound) {
  			  smsfound = false; // Reset smsfound for next message
  			  if (! fona.getSMSSender(smsid, replybuffer, 250)) {
  			    printlogln("ERROR failed getSMSSender!");
  			  }
  			  else {
    			  char *sender = 	replybuffer + 4; // remove 4 digits +506 from number
    			  printlog("FROM: "); printlogln(sender);
    			  
  			    if (fona.deleteSMS(smsid)) {
  				    printlogln("Message is deleted OK!");
  			    } else {
  				    printlogln("Couldn't delete SMS");
  			    }
  				  smsid--; // Ready to read next SMS ID
  			    printlog("***Message received: "); printlogln(message);
  			    char *ptr = strtok(message, delim);
  			    sprintf(command, "%s", ptr);
  			    printlog("***Command parsed: "); printlogln(command);
  			    ptr = strtok(NULL, delim);
  			    sprintf(readpass, "%s", ptr);
  			    printlog("***Password sent: "); printlogln(readpass);

  			  if (strcmp(command,"PASS") == 0) {
    				printlogln("User Password Change Command Received");
    				int i = strcmp(readpass, password);
    				if (i != 0) {
    				  printlogln("WARNING Password sent is NOT correct.. bypassing action");
    				}
    				else {
    				  printlogln("***Password sent is correct");
    				  if (ptr != NULL) {  // parse new password
      					ptr = strtok(NULL, delim);
      					sprintf(newpass, "%s", ptr);
      					printlog("***New password sent: "); printlogln(newpass);
      					strcpy(password, newpass);
      					// Write to SIM NVM
    						if (! fona.WritePhonebook(1,"0",password)) {
    							printlogln("Failed to write to PhoneBook");
    						}
    						else {
    							printlogln("Write to PhoneBook OK!");
    						}
                sendText(sender,"New Password OK");
    				  }
    				}
  			  }
			  else if (strcmp(command,"ADMN") == 0) {
				printlogln("Admin Password Change Command Received");
				int i = strcmp(readpass, adminpass);
				if (i != 0) {
				  printlogln("WARNING Admin Password sent is NOT correct.. bypassing action");
				}
				else {
				  printlogln("***Admin Password sent is correct");
				  if (ptr != NULL) {  // parse new password
  					ptr = strtok(NULL, delim);
  					sprintf(newpass, "%s", ptr);
  					printlog("***New password sent: "); printlogln(newpass);
  					strcpy(adminpass, newpass);
  					// Write to SIM NVM
  					if (! fona.WritePhonebook(2,"0",adminpass)) {
  						printlogln("Failed to write to PhoneBook");
  					}
  					else {
  						printlogln("Write to PhoneBook OK!");
  					}
  					sendText(sender,"New Password OK");
				  }
				}
			  }
			  else if (strcmp(command,"GPS?") == 0) {
  				printlogln("GPS SMS Command Received");
  				int i = strcmp(readpass, password);
  				if (i != 0) {
  				  printlogln("WARNING Password sent is NOT correct.. bypassing action");
  				}
  				else {
  									  
  					  psms.print("https://www.google.com/maps/search/?api=1&query=");
  					  psms.print(lastlat,4);
  					  psms.print(",");
  					  psms.print(lastlon,4);

              sendText(sender,sms);
  				}
			  }
			  else if (strcmp(command,"SHUT") == 0) {
  				printlogln("ECU Shutdown Command Received");
  				int i = strcmp(readpass, adminpass);
  				if (i != 0) {
  				  printlogln("WARNING Admin Password sent is NOT correct.. bypassing action");
  				}
  				else {
  				  printlogln("***Admin Password sent is correct. Shutting down ECU");
  				  // disconnect ECU through relay
  				  digitalWrite(PIN_ON, LOW);
  				  digitalWrite(PIN_OFF, HIGH);
  				  delay(500);//Daniel 1/3/2020. Relay set to low save energy
  				  printlogln("Clearing relay status");
  				  digitalWrite(PIN_OFF, LOW);
  				  digitalWrite(PIN_ON, LOW);
  				  // Write to SIM NVM
  					if (! fona.WritePhonebook(9,"0","0")) {
  						printlogln("Failed to write to PhoneBook");
  					}
  					else {
  						printlogln("Write to PhoneBook OK!");
  					}
  				  sendText(sender,"LPG ECU Shut down OK");
           
  				}
			  }
			  else if (strcmp(command,"CONN") == 0) {
  				printlogln("ECU Reconnect Command Received");
  				int i = strcmp(readpass, adminpass);
  				if (i != 0) {
  				  printlogln("WARNING Admin Password sent is NOT correct.. bypassing action");
  				}
  				else {
  				  printlogln("***Admin Password sent is correct. Turning on ECU");
  				  // Connect ECU through relay
  				  digitalWrite(PIN_OFF, LOW);
  				  digitalWrite(PIN_ON, HIGH);
  				  delay(500);//Daniel 1/3/2020. Relay set to low save energy
  				  printlogln("Clearing relay status");
  				  digitalWrite(PIN_OFF, LOW);
  				  digitalWrite(PIN_ON, LOW);
  				  // Write to SIM NVM				  
  				  if (! fona.WritePhonebook(9,"1","1")) {
  						printlogln("Failed to write to PhoneBook");
  					}
  					else {
  						printlogln("Write to PhoneBook OK!");
  					}
  					 sendText(sender,"LPG ECU Reconnect OK");
  				  
  				}
			  }
			  // Ricardo 2 FEB 2020 - Include a command to read back battery voltage through SMS
			  else if (strcmp(command,"BAT?") == 0) {
				  printlogln("Battery Readout Command Received");
				  int i = strcmp(readpass, password);
				  if (i != 0) {
					  printlogln("WARNING Password sent is NOT correct.. bypassing action");
				  }
				  else {

						if (! fona.getBattVoltage(&vbat)) {
						  printlogln("Failed to read Batt");
						} else {
							sprintf(stringbuffer,"VBat = %d mV",vbat);
							printlogln(stringbuffer);
							psms.print("Battery Voltage (mV):");
							psms.print(vbat);
						}
            sendText(sender,sms);
					
					} // else
			  } // if (strcmp(command,"BAT?") == 0) 
				// Ricardo 13 MARCH 2020 - Include a command to read back the OBD2 Data
			  else if (strcmp(command,"DATA?") == 0) {
				  printlogln("ECU Data Command Received");
				  int i = strcmp(readpass, password);
				  if (i != 0) {
					  printlogln("WARNING Password sent is NOT correct.. bypassing action");
				  }
				  else {
							
						psms.print("Fuel:");
						psms.print(bFuelType);
						psms.print(",LPH:");
						psms.print(fLPH);
						psms.print(",Gt:");
						psms.print(fGasTime);
						psms.print(",Lt:");
						psms.print(fLPGTime);
						psms.print(",Gl:");
						psms.print(fGasLiters);
						psms.print(",Ll:");
            psms.print(",GKm:");
            psms.print(fGasKm);
            psms.print(",LKm:");
            psms.print(fLPGKm);
						psms.print(fLPGLiters);
						
						sendText(sender,sms);
					
					} // else
			  } // if (strcmp(command,"BAT?") == 0) 
			  else  { // Additional commands to change variables 
				  printlogln("Other Command Received");
				  int i = strcmp(readpass, password);
				  if (i != 0) {
					printlogln("WARNING Password sent is NOT correct.. bypassing action");
				  }
				  else {
					  if (ptr != NULL) {  // parse new string
						ptr = strtok(NULL, delim);
						sprintf(message, "%s", ptr);
						printlog("***New parameter sent: "); printlogln(message);
						
						if (strcmp(command,"DEVID") == 0) {
							printlogln("Writing new Device ID ");
							strcpy(deviceID, message);
							strncpy(PHtext,deviceID,14);
							printlogln(PHtext);
							if (! fona.WritePhonebook(6,"0",PHtext)) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHtext);
							}
							strncpy(PHtext,deviceID+14,14);
							printlogln(PHtext);
							if (! fona.WritePhonebook(7,"0",PHtext)) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHtext);
							}
							strncpy(PHtext,deviceID+28,9);
							printlogln(PHtext);
							if (! fona.WritePhonebook(8,"0",PHtext)) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHtext);
							}
						}
						else if (strcmp(command,"OWNID") == 0) {
							printlogln("Writing new Owner ID ");
							strcpy(ownerID, message);

							strncpy(PHtext,ownerID,14);
							if (! fona.WritePhonebook(3,"0",PHtext)) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHtext);
							}
							strncpy(PHtext,ownerID+14,14);
							if (! fona.WritePhonebook(4,"0",PHtext)) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHtext);
							}
							strncpy(PHtext,ownerID+28,8);
							if (! fona.WritePhonebook(5,"0",PHtext)) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHtext);
							}
							
						}

						else if (strcmp(command,"ENGV") == 0) {
							printlogln("Writing new engine volume");
							EngineVol = atof(message) / 10;
							
							strncpy(PHnumber,message,2);
							if (! fona.WritePhonebook(11,PHnumber,"0")) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHnumber);
							}
						}
						else if (strcmp(command,"OBD2") == 0) {
							printlogln("Writing new OBD2 type");
							if (strcmp(message,"SLOW") == 0) {
								obdflag = 1;
							}
							else if (strcmp(message,"FAST") == 0) {
								obdflag = 2;
							}
							else if (strcmp(message,"CAN") == 0) {
								obdflag = 3;
							}
              else if (strcmp(message,"CANSUZ") == 0) {
               obdflag = 5;
              }
							else if (strcmp(message,"SIM") == 0) {
								obdflag = 4;
							}
              else if (strcmp(message,"KWPSUZ") == 0) {
               obdflag = 7;
              }
							sprintf(stringbuffer,"OBD2 Type:  %d",obdflag);
							printlogln(stringbuffer);
							
							
							sprintf(PHnumber,"%d",obdflag);
							if (! fona.WritePhonebook(10,PHnumber,"")) {
								printlogln("Failed to write to PhoneBook");
							}
							else {
								printlogln("Write to PhoneBook OK: ");
								printlogln(PHnumber);
							}
						}
						
           sendText(sender, "New Parameter OK");
					
						
					  }
					
				  } // else
			  } // if (strcmp(command,"BAT?") == 0) 
				  


			} // fona.getSMSSender(smsid, replybuffer, 250)) {
		  } // if sms found
		} // while (smsid > 0) 

	} // if (smsnum > 0)

  }

	if (!engineflag && pwrgatestatus == true) {
		printlogln("Charger is off - Sleep SIM808 via DTR");
		digitalWrite(DTR,HIGH);
   digitalWrite(LIN_SLP, LOW);
        //SimTogglePWRKEY();
        SIMflag = false;
	}
	
} // ProcessSMS


bool HTTPPost() {
    uint16_t statuscode, lengthp;
    uint16_t timeout; // added timeout to avoid locking up the loop ahead
    bool timeoutflag = false;
    unsigned long httptimeout;
    
    wdt.clear();
	  if (!postflag) {
        printlogln("****HTTP POSTing is disabled!");
      }
      else{

		  if (SIMflag == false) {
			  
  			printlogln("Turn DTR OFF - SIM out of power saving");
  			digitalWrite(DTR,LOW);
        digitalWrite(LIN_SLP, HIGH);
  			delay(50);
  			SIMflag = true;
			   
		  }
      if (SMSsent) {
        printlogln("SMS still outbound - retrigger HTTP POST");
        TC5trig = true;
        return false;
      }
		  printlogln("\r\n*****  HTTP POST  ******");
      delay(100);
      init_success == false; // redo a init for OBD2 9141 types
      
			if (! fona.getBattVoltage(&vbat)) {
			  printlogln("Failed to read Batt");
			} else {
			  	sprintf(stringbuffer,"VBat = %d mV",vbat);
				printlogln(stringbuffer);
			}

			// GPS was turned off due to engine off
			if (GPSflag == false) {
			  if (!fona.enableGPS(true)) {
				  printlogln("Failed to turn ON GPS!");
			  }
			  else {
  				printlogln("GPS Enabled!");
  				digitalWrite(PWR_GPS,LOW);
			  }
			}
			
      GetGPSandTime();
      
      HTTPsending = true;
      ReadOBD();
      			
			// Turn off GPS in case it was previously off
			if (GPSflag == false) {
				if (!fona.enableGPS(false))
					printlogln("Failed to turn OFF GPS!");
				else {
					digitalWrite(PWR_GPS,HIGH);
					printlogln("GPS Disabled!");
				}
			}
			


		  rtctimestamp(timestamp);
		  printlog("Current Time: "); printlogln(timestamp);

			if (lastpost) {
			  printlogln("LAST POST after engine is turned off!");
			  lastpost = false;
			}

			if (CheckNetwork()) {
				// build JSON string for HTTP POST
        ReadOBD();

          // save data in temp variables. If POST is OK, the data collected between now and the variable cleanup needs to be considered
            ftempGasTime = fGasTime;
            ftempLPGTime = fLPGTime;
            ftempGasKm = fGasKm;
            ftempLPGKm = fLPGKm;
            ftempSumGasLPH = fSumGasLPH;
            ftempSumGasKPL = fSumGasKPL;
            ftempSumLPGLPH = fSumLPGLPH;
            ftempSumLPGKPL = fSumLPGKPL;
            ntempSamples = nSamples;
            ftempGasLiters = fGasLiters;
            ftempLPGLiters = fLPGLiters;
            sprintf(stringbuffer,"nSamples: %.0f",nSamples);
            printlogln(stringbuffer);
				printlogln("HTTP POST data..");
				pdata.begin();
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
        pdata.print(",\"lgpKm\":");
				pdata.print(fLPGKm);
				pdata.print(",\"petrolTime\":");
				pdata.print(fGasTime);
				pdata.print(",\"petrolPercentage\":");
				pdata.print(fGasPerc);
				pdata.print(",\"petrolLh\":");
				pdata.print(fAvgGasLPH);
				pdata.print(",\"petrolKml\":");
				pdata.print(fAvgGasKPL);
        pdata.print(",\"petrolKm\":");
        pdata.print(fGasKm);
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
        psms.begin();
        psms.print("Gast:");
        psms.print(fGasTime*60);
        psms.print(",LPGt:");
        psms.print(fLPGTime*60);
        psms.print(",Gasl:");
        psms.print(fGasLiters);
        psms.print(",LPGl:");
        psms.print(fLPGLiters);
        psms.print(",GasKm:");
        psms.print(fGasKm);
        psms.print(",LPGKm:");
        psms.print(fLPGKm);
        psms.print(",");
        psms.print(lastlat,4);
        psms.print(",");
        psms.print(lastlon,4);
        
        printlogln("POSTing data..");
        
        if (fona.HTTP_POST_start_asynch(serverurl, F("application/json"), (uint8_t *) data, strlen(pdata))) {
          httptimeout = millis();
          HTTPsending = true;
          while (! fona.HTTP_POST_start_check(&statuscode, &lengthp)) {
              ReadOBD();
              if (millis() > httptimeout + 60000) {
                 printlogln("Timeout while doing HTTP Post!");
                 timeoutflag = true;
                 break;
              }
              
          }
          HTTPsending = false;
          if (!timeoutflag) {          
            timeout = 1000;
            while (lengthp > 0) {
              while (fona.available()) {
                char c = fona.read();
                //SerialUSB.write(c);
                sprintf(stringbuffer,"%c",c);
                printlog(stringbuffer);
                lengthp--;
                if (! lengthp) break;
              }
              delay(1);
              timeout--;
              if (!timeout) break;
            }
            printlogln(" ");
          }
          if (statuscode == 200) {
            psms.print(" - POST OK"); 
            psms.print(statuscode);
            
            printlogln("\nPOSTing success\n\n*****");

            // reset variables to start next measurement cycle

            fAvgGasLPH = 0;
            fAvgGasKPL = 0;
            fAvgLPGLPH = 0;
            fAvgLPGKPL = 0;
          

            fGasTime -= ftempGasTime;
            fLPGTime -= ftempLPGTime;
            fGasKm -= ftempGasKm;
            fLPGKm -= ftempLPGKm;
            fSumGasLPH -= ftempSumGasLPH;
            fSumGasKPL -= ftempSumGasKPL;
            fSumLPGLPH -= ftempSumLPGLPH;
            fSumLPGKPL -= ftempSumLPGKPL;
            nSamples -= ntempSamples;
            fGasLiters -= ftempGasLiters;
            fLPGLiters -= ftempLPGLiters;
            sprintf(stringbuffer,"nSamples: %.0f",nSamples);
            printlogln(stringbuffer);

            HTTPsending = true;
            ReadOBD();
            HTTPsending = false;  
            
            fona.HTTP_POST_end();
            postflag = false;  // Ricardo 24 Mar 2020 - moved post flag here - flag only clears after a successful posting
  
          }
          else {
          
            printlogln("Failed HTTP POST!");
            psms.print(" - Failed ");
            if (timeoutflag) {
               psms.print("Timeout");
            }
            else {
              psms.print(statuscode);
            }
            HTTPsending = true;
            ReadOBD();
            HTTPsending = false;  

            fona.HTTP_POST_end();
          }
          
        }
        
        
          if (!fona.enableGPRS(false))
            printlogln("Failed to turn off GPRS.");
          else
            printlogln("GPRS disconnected");
            
          HTTPsending = true;
          ReadOBD();
          HTTPsending = false;  
           
          sendText(SMSphone,sms);
          // HTTP POST
        
      }
      else {
        if (!fona.enableGPRS(false))
          printlogln("Failed to turn off GPRS.");
       else
          printlogln("GPRS disconnected");

      }

      if (!engineflag && pwrgatestatus == true) {
          printlogln("Turn DTR ON for power saving");
          digitalWrite(DTR,HIGH);
          digitalWrite(LIN_SLP, LOW);
          SIMflag = false;
       }
		} // if (postflag == false)		 
  printlogln("END HTTP POST");
	return true;
}


bool CheckNetwork() {
  // Always get RSSI
  uint8_t i = fona.getRSSI();
  int8_t dbm;
  
  if (i == 0) dbm = -115;
  if (i == 1) dbm = -111;
  if (i == 31) dbm = -52;
  if ((i >= 2) && (i <= 30)) {
    dbm = map(i, 2, 30, -110, -54);
  }
  sprintf(stringbuffer,"RSSI: %d, dBm: %d",i,dbm);
  printlogln(stringbuffer);
  
  uint8_t n = fona.getNetworkStatus();
  if (n != 1) {
	sprintf(stringbuffer,"Network not registered yet. Status:  %d",n);
	printlogln(stringbuffer);
    return false;
  }
  else {
    printlogln("Network Registered");
    fona.enableGPRS(false); // try to disable GPRS anyway
    if (!fona.enableGPRS(true)) {
      sprintf(stringbuffer,"Failed Turn on GPRS, dBm: %d",dbm);
      printlogln(stringbuffer);
      sendText(SMSphone,stringbuffer);
      fona.enableGPRS(false);
	    return false;
    } else {
      printlogln("GPRS Enabled!");
      
    }
    return true;
  }
}

void GetGPSandTime() {
      byte hours,day,month,year;
        
      if (fona.getGPS(gpsdate,&lat,&lon,&speedkph,&heading,&altitude) ) {
        sprintf(stringbuffer,"Latitude: %f, Longitude: %f, Speed: %f, Heading: %f, Alt: %f",lat,lon,speedkph,heading,altitude);
        printlogln(stringbuffer);
        lastlat = lat;
        lastlon = lon;
      } else {
        sprintf(stringbuffer,"GPS not locked, using last coordinate: %f, %f",lastlat,lastlon);
        printlogln(stringbuffer);
      }
      // Check if RTC has a proper date else use GPS date to synch
      //if (rtc.getYear() < 20) {
        
          printlog("GPS Date: ");printlogln(gpsdate);
          hours = valueFromString(gpsdate,8,2);
          day =valueFromString(gpsdate,6,2);
          month = valueFromString(gpsdate,4,2);
          year = valueFromString(gpsdate,2,2);
          if (year >= 20) {
            printlogln("Synch RTC with GPS date");
            if (( hours + UTC) < 0) {
              // adjust time and date 
              rtc.setHours(hours + UTC + 24);
              if ((day-1) > 0) {
                rtc.setDay(day-1);
              }
              else {
                // fix month
                month --;
                if (month == 0) {
                  month = 12;
                  year--;  
                }
                if ((month == 1) || (month == 3) || (month == 5) || (month == 7) || (month == 8) || (month == 10) || (month == 12)) {
                   rtc.setDay(31);
                }
                else if (month == 2) {
                  rtc.setDay(28); // doesnt consider leap years but well.. 
                }
                else {
                  rtc.setDay(30);
                }
              }  
            } else{
              rtc.setHours(hours + UTC);
              rtc.setDay(day);
            }
            rtc.setYear(year);
            rtc.setMonth(month);
            rtc.setMinutes(valueFromString(gpsdate,10,2));
            rtc.setSeconds(valueFromString(gpsdate,12,2));
          }
        //}

}

void rtctimestamp( char *timestamp) {
  int fixyear;
  fixyear = 2000 + rtc.getYear();
  sprintf(timestamp,"%.4d-%.2d-%.2d %.2d:%.2d:%.2d",fixyear,rtc.getMonth(),rtc.getDay(),rtc.getHours(),rtc.getMinutes(),rtc.getSeconds());
}

void SDdateTime(uint16_t* date, uint16_t* time) {
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(2000 + rtc.getYear(), rtc.getMonth(), rtc.getDay());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

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

void sendText(char *dest, char *msg)
{
  char *sms;
  char *sender;
  bool HTTPsending = false;
  
  
  if (!HTTPsending) {
    
    
    switch (SmsToSend) {
      case 0: {
        printlogln("No SMS in Queue, send as SMS1");
        sms = sms1;
        sender = sender1;
        break;
      }
        case 1: {
         printlogln("1 SMS in Queue, send as SMS2");
           sms = sms2;  
           sender = sender2;
          break;
        }
      case 2: {
        printlogln("2 SMS in Queue, send as SMS3");
         sms = sms3;
         sender = sender3;
        break;
      }
      case 3: {
        printlogln("Queue is full - dropping message!");
        return;
      }
    }
    rtctimestamp(timestamp);

    strncpy(sms,timestamp,140);
    strncat(sms,": ",140);
    strncat(sms,msg,140);
    SerialUSB.print("sms1: ");SerialUSB.println(sms1);
    SerialUSB.print("sms2: ");SerialUSB.println(sms2);
    SerialUSB.print("sms3: ");SerialUSB.println(sms3);
    strncpy(sender,dest,15);
    if (SmsToSend == 0) {
      SendSingleText();
    }
    SmsToSend++;
  }
}

bool checkSMSsend() {
  
  if (SmsToSend == 0) {
    printlogln("0 SMS in send queue");
  }
  else {
    int SMSstatus = fona.sendSMScheck();
    if (SMSstatus == -1) {
      printlogln("ERROR sending SMS, should retry");
      SMStimeout = 0;
      return false;
    }
    if (SMSstatus == 0) {
      SMStimeout--;
      sprintf(stringbuffer,"SMS still sending, Timeout %d",SMStimeout);
      printlogln(stringbuffer);
      
    }
    if ((SMSstatus == 1) || (SMStimeout<0)) {
      if (SMSstatus == 1) {
        printlogln("SMS sent ok!");
      }
      else{
        printlogln("SMS Timeout!");
      }
      if (SmsToSend == 1) {
        printlogln("All SMS sent");
      }
      else {
        printlogln("Sending next SMS in queue");
        strcpy(sms1,sms2);
        memset(sms2,0,140);
        strcpy(sender1,sender2);
        SerialUSB.print("sms1: ");SerialUSB.println(sms1);
        SerialUSB.print("sms2: ");SerialUSB.println(sms2);
        SerialUSB.print("sms3: ");SerialUSB.println(sms3);
        SendSingleText();
        if (SmsToSend > 2) {
          printlogln("2 SMS in Queue, pushing up");
          strcpy(sms2,sms3);
          strcpy(sender2,sender3);
          memset(sms3,0,140);
          SerialUSB.print("sms1: ");SerialUSB.println(sms1);
          SerialUSB.print("sms2: ");SerialUSB.println(sms2);
          SerialUSB.print("sms3: ");SerialUSB.println(sms3);
        }
      }      
      SmsToSend--;
    }
  }
  return true;
}

void SendSingleText() { 
    
    if (!fona.sendSMS(sender1, sms1)) {
      SerialUSB.println("ERROR could not send SMS");
    } else {
      SerialUSB.println("SMS sent with message");
      SMStimeout = 20;
    }
}


void SDWriteFile() { 
    bool filenamefound = false;
    // see if the card is present and can be initialized:
    if (digitalRead(SD_DT) == HIGH) {
      SerialUSB.println("SD card not present");
    }
    else {
	  
      if (SD.begin(SD_CS)) {
        SerialUSB.println("SD card initialized");
		
        while (!filenamefound) {
			    sprintf(filename,"%05d.txt",filecount);
			    if (!SD.exists(filename)) {
				    filenamefound = true;
				    datafile = SD.open(filename, FILE_WRITE);
				    SerialUSB.print("File open: "); SerialUSB.println(filename);
			    }
			    else{
				    filecount++;
			    }
		}
		SerialUSB.println("Writing data buffer to file.. ");
		datafile.write(filebuffer,sizeof(filebuffer));
		datafile.close();
			
      }
      else {
        SerialUSB.println("SD card init Error");
      }
    }
}

void DayClosingPost() //Daniel Castro 29/2/2020. Makes post at the end of the day if engine is off
{
	if ((rtc.getHours() == 00) && (!alreadyPosted))
	{
		lastpost = true;
		postflag = true;
    if (!SMSsent) {
		  if(HTTPPost()) alreadyPosted = true;
    }
	}
	if (rtc.getHours() == 1) alreadyPosted = false;
}

void BattMeasure(void) {
  if (!HTTPsending){ // Check if HTTP is being sent in OBD reading, all fona functions halted until so
	// If battery level is less than 3.6, turn on charging - Ricardo 28 march 2020
    printlogln("Battery Measurement:");
    if (! fona.getBattVoltage(&vbat) ) {
      printlogln("Failed to read Batt or SIM is turned off");
    } else {
      if (obdflag == SIMULATION) {  // simulation code
         
      }
	  sprintf(stringbuffer,"VBat = %d mV",vbat);
      printlogln(stringbuffer);
	  
      if (vbat < MIN_BAT_V) {
        // turn on charging
        digitalWrite(PWR_GATE,LOW);
        pwrgatestatus = false;
        if (chargersentstatus == false) {
		      rtctimestamp(timestamp);
		      printlog("Current Time: "); printlogln(timestamp);
          sendText(SMSphone,"Turned Charger ON");
          chargersentstatus = true; // send only once
        }     
      }
      else if (vbat > MAX_BAT_V) {
          digitalWrite(PWR_GATE,HIGH);
          pwrgatestatus = true;
          if (chargersentstatus == true) {
            rtctimestamp(timestamp);
            printlog("Current Time: "); printlogln(timestamp);
            sendText(SMSphone,"Turned Charger OFF battery is charged");
            chargersentstatus = false; // send only once
          }   
      }
      
    } // END Battery read
  }
	
}

void SimTogglePWRKEY()
{
	
	printlogln("Toggling PWRKEY...");
	digitalWrite(FONA_PWRKEY, HIGH);
	delay(1000);
	digitalWrite(FONA_PWRKEY, LOW);
	delay(2000);
	digitalWrite(FONA_PWRKEY, HIGH);
	delay(500);
  
}

void Ignition(void) {
  IGN_Det = digitalRead(IGN_DET);
}


void wdt_reset() {
  digitalWrite(DTR,LOW);
  delay(100);
  sendText(SMSphone,"System reset by watchdog!");
}

void WDT_Handler(void) {  // ISR for watchdog early warning, DO NOT RENAME!, need to clear
  WDTZeroCounter--;                          // EWT down counter, makes multi cycle WDT possible
  if (!IGN_Det)  wdtwake = true;                            // Skip rest of wakeup cycle if its a WDT early warning
  if (WDTZeroCounter<=0) {                   // Software EWT counter run out of time : Reset
         //if (WDT_Shutdown != NULL) WDT_Shutdown(); // run extra Shutdown functions if defined
         WDT->CLEAR.reg = 0xFF;              // value different than WDT_CLEAR_CLEAR_KEY causes reset
         while(true);
         }
 else {
        WDT->INTFLAG.bit.EW = 1;              // Clear INT EW Flag
        WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY; // Clear WTD bit
        while(WDT->STATUS.bit.SYNCBUSY); 
       }
}

// Interrupt Service Routine (ISR) for timer TC4
void TC4_Handler(void) {                        
  REG_TC4_INTFLAG = TC_INTFLAG_OVF;               // clear INT flag
  TC4trig = true;
  REG_TC4_COUNT16_COUNT = TC4_time;                      // Set period register
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_TC4_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
}

// Interrupt Service Routine (ISR) for timer TC5
void TC5_Handler(void) {                        
  REG_TC5_INTFLAG = TC_INTFLAG_OVF;               // clear INT flag
  TC5trig = true;
  REG_TC5_COUNT16_COUNT = TC5_time;                      // Set period register
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
  REG_TC5_CTRLBSET |= TC_CTRLBSET_CMD_RETRIGGER;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);        // Wait for synchronization
}


// Setup timers for multiple tasks
void TCsetup(void) {
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

  REG_TC4_INTENSET = TC_INTENSET_OVF; // Enable TC4 interrupts
 
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

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

  REG_TC5_INTENSET = TC_INTENSET_OVF; // Enable TC5 interrupts
 
  NVIC_SetPriority(TC5_IRQn, 1);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC5 to 1
  NVIC_EnableIRQ(TC5_IRQn);         // Connect TC5 to Nested Vector Interrupt Controller (NVIC)

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



void runconfig() {
  SerialUSB.println(F("Test y Configuracion del ECU-GPS"));
  SerialUSB.println(F("Iniciando....(Puede tomar unos segundos)"));
   digitalWrite(FONA_PWRKEY, HIGH);
  delay(10);
  digitalWrite(FONA_PWRKEY, LOW);
  delay(1000);
  digitalWrite(FONA_PWRKEY, HIGH);
  
  fonaSerial->begin(38400);
  fona.initPort(*fonaSerial);
  while (! fona.begin()) {
    SerialUSB.println(F("Error: No se pudo iniciar comunicacion con el chip SIM808. \r\nDiagnostico: Revise que la bateria este bien conectada. Revise que la bateria tenga al menos 3.4 Voltios.\r\nPresione 1 para reintentar"));
    serialcommand = 0;
    while(serialcommand != '1') {
      serialcommand = SerialUSB.read();
    }
    SerialUSB.println("serialcommand: ");
    SerialUSB.println(serialcommand,HEX); 
  }
  SerialUSB.println(F("Comunicacion con SIM808 establecida"));
  
   // Configure a GPRS APN, username, and password.
   fona.setGPRSNetworkSettings(F("internet.movistar.cr"), F("movistarcr"), F("movistarcr"));

   //configure HTTP gets to follow redirects over SSL
   fona.setHTTPSRedirect(true);
     
  fona.getIMEI(replybuffer);
  SerialUSB.print(F("Codigo IMEI del modulo: ")); SerialUSB.println(replybuffer);
  fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
  SerialUSB.print(F("CCID de tarjeta SIM: ")); SerialUSB.println(replybuffer);
  if (replybuffer == "ERROR") {
    SerialUSB.println(F("Verifique que la tarjeta SIM se encuentre bien instalada y reinicie el sistema con el boton"));
  }
  else {
    SerialUSB.println(F("Espere 10 segundos para establecer conexion GSM"));
    delay(10000);
    networkstatus();
    batstatus();
    printmenu();
    bool exitmenu = false;
    while (!exitmenu) {
      if (SerialUSB.available()) {
        serialcommand = SerialUSB.read();
        switch(serialcommand) {
          case '0': {
            exitmenu = true;
            break;
          }
          case '1': {
            systemconfig();
            break;
          }
          case '2': {
            networkstatus();
            break;
          }
          case '3': {
            batstatus();
            break;
          }
          case '4': {
            gpsstatus();
            break;
          }
          case '5': {
            break;
          }
          case '6': {
            webtest();
            break;
          }
          case '7': {
            testOBD();
            break;
          }
          default: {
            printmenu();
            break;
          }
          
        } // switch serialcommand
      } // if SerialUSB available
    } // while !exitmenu
  } // else
}

void printmenu() {
  SerialUSB.println(F("-------------------------------------"));
  SerialUSB.println(F("Menu de Diagnostico"));
  SerialUSB.println(F("Presione la tecla correspondiente"));
  SerialUSB.println(F("-------------------------------------"));
  SerialUSB.println(F("[1] Configuracion de sistema"));
  SerialUSB.println(F("[2] Potencia y status de GSM"));
  SerialUSB.println(F("[3] Lectura de bateria"));
  SerialUSB.println(F("[4] Lectura de GPS"));
  SerialUSB.println(F("[5] Prueba de SMS (Mensajes de texto"));
  SerialUSB.println(F("[6] Prueba de pagina web"));
  SerialUSB.println(F("[7] Lectura datos OBD2"));
  SerialUSB.println(F("[0] Salir y reiniciar sistema"));
  SerialUSB.println(F(">"));
}

void networkstatus() {
  uint8_t n = fona.getRSSI();
  int8_t r;
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  SerialUSB.print(F("Potencia de la senal GSM (RSSI): "));
  SerialUSB.print(r); SerialUSB.println(" dBm");
  if (r < -100) {
    SerialUSB.println(F("Señal GSM muy debil. Verifique conexion de la antena plana GSM. Verifique estar en un lugar con recepcion adecuada, preferible al aire libre o cerca de ventanas") );  
  }
  
  n = fona.getNetworkStatus();
  SerialUSB.print(F("Status de la red: "));
  if (n == 0) SerialUSB.println(F("No registrado"));
  if (n == 1) SerialUSB.println(F("Registrado (home)"));
  if (n == 2) SerialUSB.println(F("No registrado (buscando)"));
  if (n == 3) SerialUSB.println(F("Denegado"));
  if (n == 4) SerialUSB.println(F("Desconocido"));
  if (n == 5) SerialUSB.println(F("Registrado (roaming)"));
}

void batstatus() {
  
  uint16_t vbat;
  if (! fona.getBattVoltage(&vbat)) {
    SerialUSB.println(F("Fallo de lectura de bateria!"));
  } else {
    SerialUSB.print(F("Voltaje de bateria = ")); SerialUSB.print(vbat); SerialUSB.println(F(" mV"));
  }
  if (! fona.getBattPercent(&vbat)) {
    SerialUSB.println(F("Fallo de lectura de bateria"));
  } else {
    SerialUSB.print(F("Porcentaje de carga = ")); SerialUSB.print(vbat); SerialUSB.println(F("%"));
  }
}

void gpsstatus() {
  if (!fona.enableGPS(true))
     SerialUSB.println(F("No se pudo encender el GPS"));
  else {
    digitalWrite(PWR_GPS,LOW);
    SerialUSB.println(F("Espere 60 segundos para capturar GPS.  \r\nVerifique que la antena GPS (cuadrada) este bien conectada\r\nColoque el sistema de preferencia al aire libre y la antena en posicion vertical"));
    for (int i = 0; i <60; i++) {
      SerialUSB.print(".");
      delay(1000);
    }
    int stat = fona.GPSstatus();
    if (stat < 0)  SerialUSB.println(F("Fallo de status GPS"));
    if (stat == 0) SerialUSB.println(F("GPS apagado"));
    if (stat == 1) SerialUSB.println(F("No hay captura de posicion GPS"));
    if (stat == 2) SerialUSB.println(F("Posicion GPS 2D establecida"));
    if (stat == 3) SerialUSB.println(F("Posicion GPS 3D establecida"));
    if (stat >= 2) {
      if (fona.getGPS(gpsdate, &lat,&lon,&speedkph,&heading,&altitude) ) {
      sprintf(stringbuffer,"Posicion GPS encontrada! Latitud: %f, Longitud: %f, Altitud: %f",lat,lon,altitude);
      SerialUSB.println(stringbuffer);
      if (fona.getGPStime(gpsdate)) {
        SerialUSB.println(F("Fecha y hora GPS: "));
        SerialUSB.println(gpsdate);
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
        SerialUSB.println(F("Fecha y hora fijada para el sistema"));
      }
    } else {
      SerialUSB.println(F("Posicion GPS no encontrada, verifique conexion de antena y posicion"));
    }
              
    } // stat >= 2
  }
}

void enterline(char *input) {
  uint8_t index = 0;
  char endmarker = '\r';
  char inputbyte;
  bool newdata = false;
  SerialUSB.flush();
  while ((newdata == false)) {
    if (SerialUSB.available()){
      inputbyte = SerialUSB.read();
      SerialUSB.print(inputbyte);
      if (inputbyte != endmarker) {
        input[index] = inputbyte;
        index++;
      }
      else {
        input[index] = 0; // terminate string;
        newdata = true;
      }
    }
  }
  SerialUSB.println("");
}

void systemconfig() {
  memset(PHtext,0,15);
  // Password
  if (! fona.ReadPhonebook(1,PHnumber,PHtext)) {
         SerialUSB.println(F("No hay lectura de datos de SIM"));
  } 
  if (!*PHtext) {
    SerialUSB.print(F("Indice de tarjeta SIM esta en blanco. Password por defecto: "));
  } else {
    strncpy(password,PHtext,8);
    SerialUSB.print(F("Password leido de tarjeta SIM: "));
  }
  SerialUSB.println(password);
  SerialUSB.println(F("Introduzca un nuevo password de 8 caracteres o ENTER para usar el actual:"));
  enterline(replybuffer);
  if (replybuffer[0] != 0) {
    strncpy(password,replybuffer,8);
    if (! fona.WritePhonebook(1,"0",password)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    else {
      SerialUSB.println(F("Escritura en tarjeta SIM OK"));
    }
  }
  SerialUSB.print(F("Password establecido: "));
  SerialUSB.println(password);
  
  memset(PHtext,0,15);
  
  // Admin Password
  if (! fona.ReadPhonebook(2,PHnumber,PHtext)) {
         SerialUSB.println(F("No hay lectura de datos de SIM"));
  } 
  if (!*PHtext) {
    SerialUSB.print(F("Indice de tarjeta SIM esta en blanco. Password Administrativo por defecto: "));
  } else {
    strncpy(adminpass,PHtext,8);
    SerialUSB.print(F("Password Administrativo leido de tarjeta SIM: "));
  }
  SerialUSB.println(adminpass);
  SerialUSB.println(F("Introduzca un nuevo Password Administrativo de 8 caracteres o ENTER para usar el actual:"));
  enterline(replybuffer);
  if (replybuffer[0] != 0) {
    strncpy(adminpass,replybuffer,8);
    if (! fona.WritePhonebook(2,"0",adminpass)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    else {
      SerialUSB.println(F("Escritura en tarjeta SIM OK"));
    }
  }
  SerialUSB.print(F("Password Administrativo establecido: "));
  SerialUSB.println(adminpass);
  
  memset(PHtext,0,15);
  
  // Owner ID
  if (! fona.ReadPhonebook(3,PHnumber,PHtext)) {
         SerialUSB.println(F("No hay lectura de datos de SIM"));
  } 
  if (strcmp(PHtext,"") == 0) {
    SerialUSB.print(F("Indice de tarjeta SIM esta en blanco. OwnerID por defecto: "));
  } else {
    strncpy(stringbuffer,PHtext,14); 
    memset(PHtext,0,15);
    if (! fona.ReadPhonebook(4,PHnumber,PHtext)) {
      SerialUSB.println("No hay lectura de datos de SIM");
    } else {  
      strncpy(stringbuffer+14,PHtext,14);
      memset(PHtext,0,15);
      if (! fona.ReadPhonebook(5,PHnumber,PHtext)) {
        SerialUSB.println("No hay lectura de datos de SIM");
      } else {
        strncpy(stringbuffer+28,PHtext,8);
      }
    }
    strncpy(ownerID,stringbuffer,36);
    SerialUSB.print(F("OwnerID leido de tarjeta SIM: "));
  }
  SerialUSB.println(ownerID);
  SerialUSB.println(F("Introduzca un nuevo OwnerID o ENTER para usar el actual:"));
  enterline(replybuffer);
  if (replybuffer[0] != 0) {
    strncpy(ownerID,replybuffer,36);
    memset(PHtext,0,15);
    strncpy(PHtext,ownerID,14);
    if (! fona.WritePhonebook(3,"0",PHtext)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    memset(PHtext,0,15);
    strncpy(PHtext,ownerID+14,14);
    if (! fona.WritePhonebook(4,"0",PHtext)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    memset(PHtext,0,15);
    strncpy(PHtext,ownerID+28,8);
    if (! fona.WritePhonebook(5,"0",PHtext)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    else {
      SerialUSB.println(F("Escritura en tarjeta SIM OK"));
    }
    
  }
  SerialUSB.print(F("OwnerID establecido: "));
  SerialUSB.println(ownerID);
  
  memset(PHtext,0,15);
  // Device ID
  if (! fona.ReadPhonebook(6,PHnumber,PHtext)) {
         SerialUSB.println(F("No hay lectura de datos de SIM"));
  } 
  if (strcmp(PHtext,"") == 0) {
    SerialUSB.print(F("Indice de tarjeta SIM esta en blanco. deviceID por defecto: "));
  } else {    
    strncpy(stringbuffer,PHtext,14);
    memset(PHtext,0,15);
    if (! fona.ReadPhonebook(7,PHnumber,PHtext)) {
      SerialUSB.println("No hay lectura de datos de SIM");
    } else {
      strncpy(stringbuffer+14,PHtext,14);
      memset(PHtext,0,15);
      if (! fona.ReadPhonebook(8,PHnumber,PHtext)) {
        SerialUSB.println("No hay lectura de datos de SIM");
      } else {
        strncpy(stringbuffer+28,PHtext,8);
      }
    }
    strncpy(deviceID,stringbuffer,36);
    SerialUSB.print(F("deviceID leido de tarjeta SIM: "));
  }
  SerialUSB.println(deviceID);
  SerialUSB.println(F("Introduzca un nuevo deviceID o ENTER para usar el actual:"));
  enterline(replybuffer);
  if (replybuffer[0] != 0) {
    strncpy(deviceID,replybuffer,36);
    memset(PHtext,0,15);
    strncpy(PHtext,deviceID,14);
    if (! fona.WritePhonebook(6,"0",PHtext)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    memset(PHtext,0,15);
    strncpy(PHtext,deviceID+14,14);
    if (! fona.WritePhonebook(7,"0",PHtext)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    memset(PHtext,0,15);
    strncpy(PHtext,deviceID+28,8);
    if (! fona.WritePhonebook(8,"0",PHtext)) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    else {
      SerialUSB.println(F("Escritura en tarjeta SIM OK"));
    }
    
  }
  SerialUSB.print(F("deviceID establecido: "));
  SerialUSB.println(deviceID);
  memset(PHtext,0,15);
  memset(PHnumber,0,15);
  // Relay read and process
  if (! fona.ReadPhonebook(9,PHnumber,PHtext)) {
         SerialUSB.println(F("No hay lectura de datos de SIM"));
  } 
  if (strcmp(PHnumber,"") == 0) {
    SerialUSB.print(F("Indice de tarjeta SIM esta en blanco. Valor de Relay por defecto: "));
  } else {
    relay = atoi(PHnumber);
    SerialUSB.print(F("Relay leido de tarjeta SIM: "));
  }
  SerialUSB.println(relay);
  SerialUSB.println(F("Introduzca un 1 para encender el relay o 0 para apagar el relay:"));
  enterline(replybuffer);
  if (replybuffer[0] != 0) {
    relay = atoi(replybuffer);

    if (relay == 1) {
        strncpy(PHnumber,"1",1);
        SerialUSB.println("Comprobando que el relay este encendido");
        digitalWrite(PIN_OFF, LOW);
        digitalWrite(PIN_ON, HIGH);
    } else if (relay == 0) {
        strncpy(PHnumber,"0",1);
        SerialUSB.println("Comprobando que el relay este apagado");
        digitalWrite(PIN_ON, LOW);
        digitalWrite(PIN_OFF, HIGH);
    }
    delay(1000);
    digitalWrite(PIN_OFF, LOW);
    digitalWrite(PIN_ON, LOW);
    if (! fona.WritePhonebook(9,PHnumber,"")) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    else {
      SerialUSB.println(F("Escritura en tarjeta SIM OK"));
    }
    
  }
  SerialUSB.print(F("Valor de Relay establecido: "));
  SerialUSB.println(relay);
  
  memset(PHnumber,0,15);
  // OBD type read and process
  if (! fona.ReadPhonebook(10,PHnumber,PHtext)) {
         SerialUSB.println(F("No hay lectura de datos de SIM"));
  } 
  if (strcmp(PHnumber,"") == 0) {
    SerialUSB.print(F("Indice de tarjeta SIM esta en blanco. OBD2 por defecto: "));
  } else {
    obdflag = atoi(PHnumber);
    SerialUSB.print(F("OBD2 leido de tarjeta SIM: "));
  }
  SerialUSB.println(obdflag);
  SerialUSB.println();
  SerialUSB.println(F("Tabla de valores OBD2:"));
  SerialUSB.println(F(" 1: ISO 9141 (init 5 baud) > Pin 1"));
  SerialUSB.println(F(" 2: KWP (fast init) > Pin 1"));
  SerialUSB.println(F(" 3: CAN Bus Standard (11-bit header) > Pin 2 y 3"));
  SerialUSB.println(F(" 4: SIMULACION"));
  SerialUSB.println(F(" 5: CAN Bus (11 bit header) Suzuki > Pin 2 y 3"));
  SerialUSB.println(F(" 6: CAN Bus Standard (29-bit header) > Pin 2 y 3"));
  SerialUSB.println(F(" 7: Suzuki KWP > Pin 1"));
  SerialUSB.println(F("\r\nIntroduzca un nuevo tipo de OBD2 o ENTER para usar el actual:"));
  enterline(replybuffer);
  if (replybuffer[0] != 0) {
    obdflag = atoi(replybuffer);
    sprintf(PHnumber,"%d",obdflag);
    if (! fona.WritePhonebook(10,PHnumber,"")) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    else {
      SerialUSB.println(F("Escritura en tarjeta SIM OK"));
    }
  }
  SerialUSB.print(F("OBD2 establecido: "));
  SerialUSB.println(obdflag);
  

  memset(PHnumber,0,15);
  // Engine volume read and process
  if (! fona.ReadPhonebook(11,PHnumber,PHtext)) {
         SerialUSB.println(F("No hay lectura de datos de SIM"));
  } 
  if (strcmp(PHnumber,"") == 0) {
    SerialUSB.print(F("Indice de tarjeta SIM esta en blanco. Volumen de motor por defecto: "));
  } else {
    EngineVol = atof(PHnumber);
    SerialUSB.print(F("Volumen de motor leido de tarjeta SIM: "));
  }
  SerialUSB.println(EngineVol*100,0);
  SerialUSB.println(F("\r\nIntroduzca un nuevo volumen de motor en CC (por ejemplo 1600) o ENTER para usar el actual:"));
  enterline(replybuffer);
  if (replybuffer[0] != 0) {
    EngineVol = atof(replybuffer)/1000;

    strncpy(PHnumber,replybuffer,2);
    if (! fona.WritePhonebook(11,PHnumber,"")) {
      SerialUSB.println(F("Fallo en guardar en tarjeta SIM"));
    }
    else {
      SerialUSB.println(F("Escritura en tarjeta SIM OK"));
    }
    
  }
  SerialUSB.print(F("Volumen de motor establecido: "));
  SerialUSB.println(EngineVol*100,0);
  
  SerialUSB.println(F("\r\n***** Configuracion finalizada! *****\r\n"));
  printmenu();

}

void webtest() {
  SerialUSB.println(F("\r\nPrueba de subida a pagina web\r\n"));
  SerialUSB.println(F("Encendiendo GPRS.."));
   if (!fona.enableGPRS(true)) {
      SerialUSB.println(F("Fallo en encender GPRS. Compruebe la potencia y status de la red (opcion 2)"));
    return;
    } else {
      SerialUSB.println(F("GPRS Habilitado!"));
      
      rtctimestamp(timestamp);
      SerialUSB.println(F("Hora y fecha del sistema: "));  SerialUSB.println(timestamp);

      SerialUSB.println(F("Iniciando HTTP POST. Mensaje por enviar:"));
      
      uint16_t statuscode;
      uint16_t lengthp;
      pdata.begin();
      pdata.print("{\"datetime\":\"");
      pdata.print(timestamp);
      pdata.print("\",\"device\":\"");
      pdata.print(deviceID);
      pdata.print("\",\"lgpTime\":"); 
      pdata.print(0);
      pdata.print(",\"lgpPercentage\":");
      pdata.print(0);
      pdata.print(",\"lgpLh\":");
      pdata.print(0);
      pdata.print(",\"lgpKml\":");
      pdata.print(0);
      pdata.print(",\"petrolTime\":");
      pdata.print(0);
      pdata.print(",\"petrolPercentage\":");
      pdata.print(0);
      pdata.print(",\"petrolLh\":");
      pdata.print(0);
      pdata.print(",\"petrolKml\":");
      pdata.print(0);
      pdata.print(",\"owner\":\"");
      pdata.print(ownerID);
      pdata.print("\",\"lgpL\":\"");
      pdata.print(0);
      pdata.print("\",\"petrolL\":\"");
      pdata.print(0);
      pdata.print("\",\"latitude\":");
      pdata.print(lat,4);
      pdata.print(",\"longitude\":");
      pdata.print(lon,4);
      pdata.print("}");
     
      SerialUSB.println(data);

      if (fona.HTTP_POST_start(serverurl, F("application/json"), (uint8_t *) data, strlen(pdata),&statuscode,&lengthp)) {
          while (lengthp > 0) {
            while (fona.available()) {
              char c = fona.read();
              //SerialUSB.write(c);
             sprintf(stringbuffer,"%c",c);
            printlog(stringbuffer);
              
              lengthp--;
              if (! lengthp) break;
            }
          }

                    
          SerialUSB.println(F("\r\nEnvio a pagina web Exitoso!\r\n*****"));
          
        }
        else {
          
          SerialUSB.print(F("Fallo en envio a pagina web! Codigo de Error: "));
          
          SerialUSB.println(statuscode);
          SerialUSB.print(F("Verifique que los ID de usuario (owner) y dispositivo (device) hayan sido programados previamente\r\n y sean correctos con la pagina web"));
        }
        fona.HTTP_POST_end();
      
    }
    if (!fona.enableGPRS(false))
         SerialUSB.println(F("Fallo al intentar apagar GPRS."));
       else
        SerialUSB.println(F("GPRS desconectado"));
      
}


void testOBD() {
  bool exitobd = false;
  while (!exitobd) {
    SerialUSB.println(F("\r\n***** Test de lectura OBD2. *****"));
    SerialUSB.println(F("Instrucciones:\r\n1. Verificar que los pines del conector OBD2 esten conectados al dispositivo correctamente:"));
    SerialUSB.println(F("OBD2 pin  7 al pin 1 del conector y conectado internamente a LIN_IO"));
    SerialUSB.println(F("OBD2 pin  6 al pin 2 del conector y conectado internamente a CAN_H"));
    SerialUSB.println(F("OBD2 pin 14 al pin 3 del conector y conectado internamente a CAN_L"));
    SerialUSB.println(F("\r\nSeleccione un tipo de comunicacion:"));
    SerialUSB.println(F(" 1: ISO 9141 (init 5 baud) > Pin 1"));
    SerialUSB.println(F(" 2: KWP (fast init) > Pin 1"));
    SerialUSB.println(F(" 3: CAN Bus Standard (11-bit header) > Pin 2 y 3"));
    SerialUSB.println(F(" 4: SIMULACION"));
    SerialUSB.println(F(" 5: CAN Bus (11 bit header) Suzuki > Pin 2 y 3"));
    SerialUSB.println(F(" 6: CAN Bus Standard (29-bit header) > Pin 2 y 3"));
    SerialUSB.println(F(" 7: Suzuki KWP > Pin 1"));
    SerialUSB.println(F(" 0: Salir del modo de prueba OBD2"));
    uint8_t tempobd = 0;
    enterline(replybuffer);
    if (replybuffer[0] != 0) {
      tempobd = atoi(replybuffer);
    }
    switch (tempobd) {
      case '1': {
        break;
      }
      case '2': {
        break;
      }
      case '3': {
        break;
      }
      case '4': {
        SerialUSB.println(F(" 4: SIMULACION - favor escoger otro tipo"));
        break;
      }
      case '5': {
        break;
      }
      case '6': {
        break;
      }
      case '7': {
        break;
      }
      case '0': {
        SerialUSB.println(F("Saliendo del modo de prueba OBD2"));
        exitobd = true;
        break;
      }
      default: {
        break;
      }
    } // switch 
  } // while ! exitobd
}
