// Jolty Sample for Packet Serial
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <Sabertooth.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(19, 18); //RX, TX
SoftwareSerial mySerial2(17,16);
Sabertooth ST(128, mySerial); // The Sabertooth is on address 128. We'll name its object ST.
                    // If you've set up your Sabertooth on a different address, of course change
                    // that here. For how to configure address, etc. see the DIP Switch Wizard for
                    //   Sabertooth - http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                    //   SyRen      - http://www.dimensionengineering.com/datasheets/SyrenDIPWizard/start.htm
                    // Be sure to select Packetized Serial Mode for use with this library.
                    //
                    // On that note, you can use this library for SyRen just as easily.
                    // The diff-drive commands (drive, turn) do not work on a SyRen, of course, but it will respond correctly
                    // if you command motor 1 to do something (ST.motor(1, ...)), just like a Sabertooth.
                    //
                    // In this sample, hardware serial TX connects to S1.
                    // See the SoftwareSerial example in 3.Advanced for how to use other pins.
Sabertooth ST2(128, mySerial2);
                                        
void setup()
{
  mySerial.begin(9600);
  mySerial2.begin(9600);
  Serial.begin(9600); // 9600 is the default baud rate for Sabertooth packet serial.
  ST.autobaud(); // Send the autobaud command to the Sabertooth controller(s).
                 // NOTE: *Not all* Sabertooth controllers need this command.
                 //       It doesn't hurt anything, but V2 controllers use an
                 //       EEPROM setting (changeable with the function setBaudRate) to set
                 //       the baud rate instead of detecting with autobaud.
                 //
                 //       If you have a 2x12, 2x25 V2, 2x60 or SyRen 50, you can remove
                 //       the autobaud line and save yourself two seconds of startup delay
  ST.motor(1, 0);
  ST2.autobaud();
  ST2.motor(1,0);
}


void loop() {
  // put your main code here, to run repeatedly:
  byte input[2];
   
  Serial.readBytes(input,2); 
   
   unsigned int OdroidInInt = input[1]+input[0]*256;
   unsigned int CRC = 18;
   unsigned int msg = OdroidInInt % 2048;
   unsigned int checksum = OdroidInInt / 2048;
   unsigned int curr = msg*32 + checksum;

   int a, index;
   while (curr >= 32) {
    index = largestOneIndex(curr);
    a = CRC << (index - 4);
    curr ^= a;
   }
   if (curr) {
    // CRC came back bad
    // do some error handling here??
   }else{
  int power = input[1] % 128;
  if(input[1]/128 % 2 == 1){
    power *= -1;
  }
  int motorNumber = input[0]%8;
  if(motorNumber == 1){
  ST.motor(1, power);
  }else if(motorNumber == 2){
  ST.motor(2, power);
  }
   }
}
void Stop(){
  ST.stop();
  ST2.stop();
}
int largestOneIndex(unsigned int curr) {
  int i = 15;
  while (i >= 0) {
    if (curr & (1 << i)) {
      break;
    }
    i--;
  }
  return i;
}
