// **** INCLUDES *****
#include "ArduinoLowPower.h"

// External interrupt on pin 0 (use pin 0 to 24, except pin 4 on Arduino Zero)
bool looping = false;

void setup()
{
	// Wait for serial USB port to open
	while(!SerialUSB);
	SerialUSB.println("***** ATSAMD21 Standby Mode Example *****");
	
	// ***** IMPORTANT *****
	// Delay is required to allow the USB interface to be active during
	// sketch upload process
	SerialUSB.println("Entering standby mode in 3 seconds:");
	delay(3000);
    

}

void loop() 
{
	SerialUSB.println("Entering standby mode.");
	SerialUSB.println("Apply low signal to wake the processor.");
	SerialUSB.println("Zzzz...");
	// Detach USB interface
	USBDevice.detach();
  // Enter standby mode
  LowPower.sleep(10000);  
  // Attach USB interface
  USBDevice.attach();
  looping = true;
  // Serial USB is blazing fast, you might miss the messages

 
  // Wait for user response
  while(!SerialUSB.available());
  while(SerialUSB.available() > 0)
  {
		SerialUSB.read();
     delay(1000);
      SerialUSB.println("Awake!");
      SerialUSB.println("Send any character to enter standby mode again");
	}
  
}
