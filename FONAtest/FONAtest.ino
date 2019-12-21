/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

/*
THIS CODE IS STILL IN PROGRESS!

Open up the serial console on the Arduino at 115200 baud to interact with FONA

Note that if you need to set a GPRS APN, username, and password scroll down to
the commented section below at the end of the setup() function.
*/
#include "Adafruit_FONA.h"

#define FONA_RST 4
#define FONA_TX 5
#define FONA_RX 6
#define FONA_PWRKEY 7

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
//#include <SoftwareSerialUSB.h>
//SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
//SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
  HardwareSerial *fonaSerial = &Serial1;

// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);

uint8_t type;

void setup() {
  while (!SerialUSB);


  SerialUSB.begin(115200);
  SerialUSB.println(F("FONA basic test"));
  SerialUSB.println(F("Initializing....(May take 3 seconds)"));
  SerialUSB.println(F("PWR KEY Toggle"));
  pinMode(FONA_PWRKEY, OUTPUT);
  digitalWrite(FONA_PWRKEY, HIGH);
  delay(10);
  digitalWrite(FONA_PWRKEY, LOW);
  delay(1000);
  digitalWrite(FONA_PWRKEY, HIGH);
  
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    SerialUSB.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  SerialUSB.println(F("FONA is OK"));
  SerialUSB.print(F("Found "));
  switch (type) {
    case FONA800L:
      SerialUSB.println(F("FONA 800L")); break;
    case FONA800H:
      SerialUSB.println(F("FONA 800H")); break;
    case FONA808_V1:
      SerialUSB.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      SerialUSB.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      SerialUSB.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      SerialUSB.println(F("FONA 3G (European)")); break;
    default: 
      SerialUSB.println(F("???")); break;
  }
  
  // Print module IMEI number.
  char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    SerialUSB.print("Module IMEI: "); SerialUSB.println(imei);
  }

  // Optionally configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
  //fona.setGPRSNetworkSettings(F("your APN"), F("your username"), F("your password"));

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  printMenu();
}

void printMenu(void) {
  SerialUSB.println(F("-------------------------------------"));
  SerialUSB.println(F("[?] Print this menu"));
  SerialUSB.println(F("[a] read the ADC 2.8V max (FONA800 & 808)"));
  SerialUSB.println(F("[b] read the Battery V and % charged"));
  SerialUSB.println(F("[C] read the SIM CCID"));
  SerialUSB.println(F("[U] Unlock SIM with PIN code"));
  SerialUSB.println(F("[i] read RSSI"));
  SerialUSB.println(F("[n] get Network status"));
  SerialUSB.println(F("[v] set audio Volume"));
  SerialUSB.println(F("[V] get Volume"));
  SerialUSB.println(F("[H] set Headphone audio (FONA800 & 808)"));
  SerialUSB.println(F("[e] set External audio (FONA800 & 808)"));
  SerialUSB.println(F("[T] play audio Tone"));
  SerialUSB.println(F("[P] PWM/Buzzer out (FONA800 & 808)"));

  // FM (SIM800 only!)
  SerialUSB.println(F("[f] tune FM radio (FONA800)"));
  SerialUSB.println(F("[F] turn off FM (FONA800)"));
  SerialUSB.println(F("[m] set FM volume (FONA800)"));
  SerialUSB.println(F("[M] get FM volume (FONA800)"));
  SerialUSB.println(F("[q] get FM station signal level (FONA800)"));

  // Phone
  SerialUSB.println(F("[c] make phone Call"));
  SerialUSB.println(F("[A] get call status"));
  SerialUSB.println(F("[h] Hang up phone"));
  SerialUSB.println(F("[p] Pick up phone"));

  // SMS
  SerialUSB.println(F("[N] Number of SMSs"));
  SerialUSB.println(F("[r] Read SMS #"));
  SerialUSB.println(F("[R] Read All SMS"));
  SerialUSB.println(F("[d] Delete SMS #"));
  SerialUSB.println(F("[s] Send SMS"));
  SerialUSB.println(F("[u] Send USSD"));
  
  // Time
  SerialUSB.println(F("[y] Enable network time sync (FONA 800 & 808)"));
  SerialUSB.println(F("[Y] Enable NTP time sync (GPRS FONA 800 & 808)"));
  SerialUSB.println(F("[t] Get network time"));

  // GPRS
  SerialUSB.println(F("[G] Enable GPRS"));
  SerialUSB.println(F("[g] Disable GPRS"));
  SerialUSB.println(F("[l] Query GSMLOC (GPRS)"));
  SerialUSB.println(F("[w] Read webpage (GPRS)"));
  SerialUSB.println(F("[W] Post to website (GPRS)"));

  // GPS
  if ((type == FONA3G_A) || (type == FONA3G_E) || (type == FONA808_V1) || (type == FONA808_V2)) {
    SerialUSB.println(F("[O] Turn GPS on (FONA 808 & 3G)"));
    SerialUSB.println(F("[o] Turn GPS off (FONA 808 & 3G)"));
    SerialUSB.println(F("[L] Query GPS location (FONA 808 & 3G)"));
    if (type == FONA808_V1) {
      SerialUSB.println(F("[x] GPS fix status (FONA808 v1 only)"));
    }
    SerialUSB.println(F("[E] Raw NMEA out (FONA808)"));
  }
  
  SerialUSB.println(F("[S] create Serial passthru tunnel"));
  SerialUSB.println(F("-------------------------------------"));
  SerialUSB.println(F(""));

}
void loop() {
  SerialUSB.print(F("FONA> "));
  while (! SerialUSB.available() ) {
    if (fona.available()) {
      SerialUSB.write(fona.read());
    }
  }

  char command = SerialUSB.read();
  SerialUSB.println(command);


  switch (command) {
    case '?': {
        printMenu();
        break;
      }

    case 'a': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          SerialUSB.println(F("Failed to read ADC"));
        } else {
          SerialUSB.print(F("ADC = ")); SerialUSB.print(adc); SerialUSB.println(F(" mV"));
        }
        break;
      }

    case 'b': {
        // read the battery voltage and percentage
        uint16_t vbat;
        if (! fona.getBattVoltage(&vbat)) {
          SerialUSB.println(F("Failed to read Batt"));
        } else {
          SerialUSB.print(F("VBat = ")); SerialUSB.print(vbat); SerialUSB.println(F(" mV"));
        }


        if (! fona.getBattPercent(&vbat)) {
          SerialUSB.println(F("Failed to read Batt"));
        } else {
          SerialUSB.print(F("VPct = ")); SerialUSB.print(vbat); SerialUSB.println(F("%"));
        }

        break;
      }

    case 'U': {
        // Unlock the SIM with a PIN code
        char PIN[5];
        flushSerial();
        SerialUSB.println(F("Enter 4-digit PIN"));
        readline(PIN, 3);
        SerialUSB.println(PIN);
        SerialUSB.print(F("Unlocking SIM card: "));
        if (! fona.unlockSIM(PIN)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }

    case 'C': {
        // read the CCID
        fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
        SerialUSB.print(F("SIM CCID = ")); SerialUSB.println(replybuffer);
        break;
      }

    case 'i': {
        // read the RSSI
        uint8_t n = fona.getRSSI();
        int8_t r;

        SerialUSB.print(F("RSSI = ")); SerialUSB.print(n); SerialUSB.print(": ");
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        SerialUSB.print(r); SerialUSB.println(F(" dBm"));

        break;
      }

    case 'n': {
        // read the network/cellular status
        uint8_t n = fona.getNetworkStatus();
        SerialUSB.print(F("Network status "));
        SerialUSB.print(n);
        SerialUSB.print(F(": "));
        if (n == 0) SerialUSB.println(F("Not registered"));
        if (n == 1) SerialUSB.println(F("Registered (home)"));
        if (n == 2) SerialUSB.println(F("Not registered (searching)"));
        if (n == 3) SerialUSB.println(F("Denied"));
        if (n == 4) SerialUSB.println(F("Unknown"));
        if (n == 5) SerialUSB.println(F("Registered roaming"));
        break;
      }

    /*** Audio ***/
    case 'v': {
        // set volume
        flushSerial();
        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          SerialUSB.print(F("Set Vol [0-8] "));
        } else {
          SerialUSB.print(F("Set Vol % [0-100] "));
        }
        uint8_t vol = readnumber();
        SerialUSB.println();
        if (! fona.setVolume(vol)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }

    case 'V': {
        uint8_t v = fona.getVolume();
        SerialUSB.print(v);
        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          SerialUSB.println(" / 8");
        } else {
          SerialUSB.println("%");
        }
        break;
      }

    case 'H': {
        // Set Headphone output
        if (! fona.setAudio(FONA_HEADSETAUDIO)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        fona.setMicVolume(FONA_HEADSETAUDIO, 15);
        break;
      }
    case 'e': {
        // Set External output
        if (! fona.setAudio(FONA_EXTAUDIO)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }

        fona.setMicVolume(FONA_EXTAUDIO, 10);
        break;
      }

    case 'T': {
        // play tone
        flushSerial();
        SerialUSB.print(F("Play tone #"));
        uint8_t kittone = readnumber();
        SerialUSB.println();
        // play for 1 second (1000 ms)
        if (! fona.playToolkitTone(kittone, 1000)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }

    /*** FM Radio ***/

    case 'f': {
        // get freq
        flushSerial();
        SerialUSB.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        SerialUSB.println();
        // FM radio ON using headset
        if (fona.FMradio(true, FONA_HEADSETAUDIO)) {
          SerialUSB.println(F("Opened"));
        }
        if (! fona.tuneFMradio(station)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("Tuned"));
        }
        break;
      }
    case 'F': {
        // FM radio off
        if (! fona.FMradio(false)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }
    case 'm': {
        // Set FM volume.
        flushSerial();
        SerialUSB.print(F("Set FM Vol [0-6]:"));
        uint8_t vol = readnumber();
        SerialUSB.println();
        if (!fona.setFMVolume(vol)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }
    case 'M': {
        // Get FM volume.
        uint8_t fmvol = fona.getFMVolume();
        if (fmvol < 0) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.print(F("FM volume: "));
          SerialUSB.println(fmvol, DEC);
        }
        break;
      }
    case 'q': {
        // Get FM station signal level (in decibels).
        flushSerial();
        SerialUSB.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        SerialUSB.println();
        int8_t level = fona.getFMSignalLevel(station);
        if (level < 0) {
          SerialUSB.println(F("Failed! Make sure FM radio is on (tuned to station)."));
        } else {
          SerialUSB.print(F("Signal level (dB): "));
          SerialUSB.println(level, DEC);
        }
        break;
      }

    /*** PWM ***/

    case 'P': {
        // PWM Buzzer output @ 2KHz max
        flushSerial();
        SerialUSB.print(F("PWM Freq, 0 = Off, (1-2000): "));
        uint16_t freq = readnumber();
        SerialUSB.println();
        if (! fona.setPWM(freq)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }

    /*** Call ***/
    case 'c': {
        // call a phone!
        char number[30];
        flushSerial();
        SerialUSB.print(F("Call #"));
        readline(number, 30);
        SerialUSB.println();
        SerialUSB.print(F("Calling ")); SerialUSB.println(number);
        if (!fona.callPhone(number)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("Sent!"));
        }

        break;
      }
    case 'A': {
        // get call status
        int8_t callstat = fona.getCallStatus();
        switch (callstat) {
          case 0: SerialUSB.println(F("Ready")); break;
          case 1: SerialUSB.println(F("Could not get status")); break;
          case 3: SerialUSB.println(F("Ringing (incoming)")); break;
          case 4: SerialUSB.println(F("Ringing/in progress (outgoing)")); break;
          default: SerialUSB.println(F("Unknown")); break;
        }
        break;
      }
      
    case 'h': {
        // hang up!
        if (! fona.hangUp()) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }

    case 'p': {
        // pick up!
        if (! fona.pickUp()) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("OK!"));
        }
        break;
      }

    /*** SMS ***/

    case 'N': {
        // read the number of SMS's!
        int8_t smsnum = fona.getNumSMS();
        if (smsnum < 0) {
          SerialUSB.println(F("Could not read # SMS"));
        } else {
          SerialUSB.print(smsnum);
          SerialUSB.println(F(" SMS's on SIM card!"));
        }
        break;
      }
    case 'r': {
        // read an SMS
        flushSerial();
        SerialUSB.print(F("Read #"));
        uint8_t smsn = readnumber();
        SerialUSB.print(F("\n\rReading SMS #")); SerialUSB.println(smsn);

        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsn, replybuffer, 250)) {
          SerialUSB.println("Failed!");
          break;
        }
        SerialUSB.print(F("FROM: ")); SerialUSB.println(replybuffer);

        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          SerialUSB.println("Failed!");
          break;
        }
        SerialUSB.print(F("***** SMS #")); SerialUSB.print(smsn);
        SerialUSB.print(" ("); SerialUSB.print(smslen); SerialUSB.println(F(") bytes *****"));
        SerialUSB.println(replybuffer);
        SerialUSB.println(F("*****"));

        break;
      }
    case 'R': {
        // read all SMS
        int8_t smsnum = fona.getNumSMS();
        uint16_t smslen;
        int8_t smsn;

        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
          smsn = 0; // zero indexed
          smsnum--;
        } else {
          smsn = 1;  // 1 indexed
        }

        for ( ; smsn <= smsnum; smsn++) {
          SerialUSB.print(F("\n\rReading SMS #")); SerialUSB.println(smsn);
          if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
            SerialUSB.println(F("Failed!"));
            break;
          }
          // if the length is zero, its a special case where the index number is higher
          // so increase the max we'll look at!
          if (smslen == 0) {
            SerialUSB.println(F("[empty slot]"));
            smsnum++;
            continue;
          }

          SerialUSB.print(F("***** SMS #")); SerialUSB.print(smsn);
          SerialUSB.print(" ("); SerialUSB.print(smslen); SerialUSB.println(F(") bytes *****"));
          SerialUSB.println(replybuffer);
          SerialUSB.println(F("*****"));
        }
        break;
      }

    case 'd': {
        // delete an SMS
        flushSerial();
        SerialUSB.print(F("Delete #"));
        uint8_t smsn = readnumber();

        SerialUSB.print(F("\n\rDeleting SMS #")); SerialUSB.println(smsn);
        if (fona.deleteSMS(smsn)) {
          SerialUSB.println(F("OK!"));
        } else {
          SerialUSB.println(F("Couldn't delete"));
        }
        break;
      }

    case 's': {
        // send an SMS!
        char sendto[21], message[141];
        flushSerial();
        SerialUSB.print(F("Send to #"));
        readline(sendto, 20);
        SerialUSB.println(sendto);
        SerialUSB.print(F("Type out one-line message (140 char): "));
        readline(message, 140);
        SerialUSB.println(message);
        if (!fona.sendSMS(sendto, message)) {
          SerialUSB.println(F("Failed"));
        } else {
          SerialUSB.println(F("Sent!"));
        }

        break;
      }

    case 'u': {
      // send a USSD!
      char message[141];
      flushSerial();
      SerialUSB.print(F("Type out one-line message (140 char): "));
      readline(message, 140);
      SerialUSB.println(message);

      uint16_t ussdlen;
      if (!fona.sendUSSD(message, replybuffer, 250, &ussdlen)) { // pass in buffer and max len!
        SerialUSB.println(F("Failed"));
      } else {
        SerialUSB.println(F("Sent!"));
        SerialUSB.print(F("***** USSD Reply"));
        SerialUSB.print(" ("); SerialUSB.print(ussdlen); SerialUSB.println(F(") bytes *****"));
        SerialUSB.println(replybuffer);
        SerialUSB.println(F("*****"));
      }
    }

    /*** Time ***/

    case 'y': {
        // enable network time sync
        if (!fona.enableNetworkTimeSync(true))
          SerialUSB.println(F("Failed to enable"));
        break;
      }

    case 'Y': {
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          SerialUSB.println(F("Failed to enable"));
        break;
      }

    case 't': {
        // read the time
        char buffer[23];

        fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
        SerialUSB.print(F("Time = ")); SerialUSB.println(buffer);
        break;
      }


    /*********************************** GPS (SIM808 only) */

    case 'o': {
        // turn GPS off
        if (!fona.enableGPS(false))
          SerialUSB.println(F("Failed to turn off"));
        break;
      }
    case 'O': {
        // turn GPS on
        if (!fona.enableGPS(true))
          SerialUSB.println(F("Failed to turn on"));
        break;
      }
    case 'x': {
        int8_t stat;
        // check GPS fix
        stat = fona.GPSstatus();
        if (stat < 0)
          SerialUSB.println(F("Failed to query"));
        if (stat == 0) SerialUSB.println(F("GPS off"));
        if (stat == 1) SerialUSB.println(F("No fix"));
        if (stat == 2) SerialUSB.println(F("2D fix"));
        if (stat == 3) SerialUSB.println(F("3D fix"));
        break;
      }

    case 'L': {
        // check for GPS location
        char gpsdata[120];
        fona.getGPS(0, gpsdata, 120);
        if (type == FONA808_V1)
          SerialUSB.println(F("Reply in format: mode,longitude,latitude,altitude,utctime(yyyymmddHHMMSS),ttff,satellites,speed,course"));
        else 
          SerialUSB.println(F("Reply in format: mode,fixstatus,utctime(yyyymmddHHMMSS),latitude,longitude,altitude,speed,course,fixmode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,reserved3,C/N0max,HPA,VPA"));
        SerialUSB.println(gpsdata);

        break;
      }

    case 'E': {
        flushSerial();
        if (type == FONA808_V1) {
          SerialUSB.print(F("GPS NMEA output sentences (0 = off, 34 = RMC+GGA, 255 = all)"));
        } else {
          SerialUSB.print(F("On (1) or Off (0)? "));
        }
        uint8_t nmeaout = readnumber();

        // turn on NMEA output
        fona.enableGPSNMEA(nmeaout);

        break;
      }

    /*********************************** GPRS */

    case 'g': {
        // turn GPRS off
        if (!fona.enableGPRS(false))
          SerialUSB.println(F("Failed to turn off"));
        break;
      }
    case 'G': {
        // turn GPRS on
        if (!fona.enableGPRS(true))
          SerialUSB.println(F("Failed to turn on"));
        break;
      }
    case 'l': {
        // check for GSMLOC (requires GPRS)
        uint16_t returncode;

        if (!fona.getGSMLoc(&returncode, replybuffer, 250))
          SerialUSB.println(F("Failed!"));
        if (returncode == 0) {
          SerialUSB.println(replybuffer);
        } else {
          SerialUSB.print(F("Fail code #")); SerialUSB.println(returncode);
        }

        break;
      }
    case 'w': {
        // read website URL
        uint16_t statuscode;
        int16_t length;
        char url[80];

        flushSerial();
        SerialUSB.println(F("NOTE: in beta! Use small webpages to read!"));
        SerialUSB.println(F("URL to read (e.g. wifitest.adafruit.com/testwifi/index.html):"));
        SerialUSB.print(F("http://")); readline(url, 79);
        SerialUSB.println(url);

        SerialUSB.println(F("****"));
        if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
          SerialUSB.println("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

            // SerialUSB.write is too slow, we'll write directly to Serial register!
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            SerialUSB.write(c);
#endif
            length--;
            if (! length) break;
          }
        }
        SerialUSB.println(F("\n****"));
        fona.HTTP_GET_end();
        break;
      }

    case 'W': {
        // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[80];
        char data[80];

        flushSerial();
        SerialUSB.println(F("NOTE: in beta! Use simple websites to post!"));
        SerialUSB.println(F("URL to post (e.g. httpbin.org/post):"));
        SerialUSB.print(F("http://")); readline(url, 79);
        SerialUSB.println(url);
        SerialUSB.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
        readline(data, 79);
        SerialUSB.println(data);

        SerialUSB.println(F("****"));
        if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
          SerialUSB.println("Failed!");
          break;
        }
        while (length > 0) {
          while (fona.available()) {
            char c = fona.read();

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            UDR0 = c;
#else
            SerialUSB.write(c);
#endif

            length--;
            if (! length) break;
          }
        }
        SerialUSB.println(F("\n****"));
        fona.HTTP_POST_end();
        break;
      }
    /*****************************************/

    case 'S': {
        SerialUSB.println(F("Creating SERIAL TUBE"));
        while (1) {
          while (SerialUSB.available()) {
            delay(1);
            fona.write(SerialUSB.read());
          }
          if (fona.available()) {
            SerialUSB.write(fona.read());
          }
        }
        break;
      }

    default: {
        SerialUSB.println(F("Unknown command"));
        printMenu();
        break;
      }
  }
  // flush input
  flushSerial();
  while (fona.available()) {
    SerialUSB.write(fona.read());
  }

}

void flushSerial() {
  while (SerialUSB.available())
    SerialUSB.read();
}

char readBlocking() {
  while (!SerialUSB.available());
  return SerialUSB.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //SerialUSB.print(c);
  }
  SerialUSB.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    SerialUSB.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //SerialUSB.println(F("SPACE"));
      break;
    }

    while (SerialUSB.available()) {
      char c =  SerialUSB.read();

      //SerialUSB.print(c, HEX); SerialUSB.print("#"); SerialUSB.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //SerialUSB.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}
