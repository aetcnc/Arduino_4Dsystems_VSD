///////////////// THIS IS SAMPLE CODE FOR EDUCATION AND INFORMATION ONLY ////////////////

// This program was written by Graeme Hulme-Jones of Affordable Engineering Technologies (graeme@affordengtech.com)
// as an educational resource only, to contribute to the feasibility of use of the Arduino platform in an industrial setting.

// This code is released under the GNU GENERAL PUBLIC LICENSE, Version 3, 29 June 2007, Copyright (C) 2007 Free Software Foundation, Inc.
// We make no guarantees of this code whatsoever and it is used entirely at your own risk!

// The last official update to this code was made on 10/01/2014.



#include <SimpleModbusMaster.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <genieArduino.h>

//////////////////// Port information ///////////////////

#define baud 19200       //The Modbus connection speed. Make sure this is the same on your device.
#define timeout 2000     //Once the master sends a request out, the slave has 2000 milliseconds to reply before a timeout. 
#define polling 10       // The Arduino (Master) sends a modbus request to the slave. The slave replies. "polling" is the amount of time the Arduino waits before sending another request!!
                         // Make sure your slave can get itself from transmit mode back to receive mode within this 10ms period, or you will get lost pakets and an eventual connection timeout.
                         // The SimpleModbusMasterV10 author recommends 100ms. We fine-tuned our application to 10ms. This does not mean it will definitely work for you. Start with 100-200ms!
#define retry_count 10   // The number of re-sends of a failed packet before a timeout.
#define TxEnablePin 20   // The pin number that the Modbus indication LED is on. Blinks once every time a full packet send is completed.

//////////////////// Assembly of the Modbus packets ///////////////

// We have eight 8 reads to do from the VSD and two 2 writes, a total of TEN packets need to be defined to carry the data over Modbus.
// The content of the packets is assigned with the packetPointer declarations further down.

enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,
  PACKET9,
  PACKET10,
  TOTAL_NO_OF_PACKETS // Leave this last entry. It tells the library how many packets are to be sent out.
};

Packet packets[TOTAL_NO_OF_PACKETS]; //Leave this line in here.

/////////////////////////VSD READS//////////////////////////////
packetPointer statusWordVSD = &packets[PACKET1];
packetPointer actualSpeedVSD = &packets[PACKET2];
packetPointer motorCurrentVSD = &packets[PACKET3];
packetPointer motorPowerVSD = &packets[PACKET4];
packetPointer motorVoltageVSD = &packets[PACKET5];
packetPointer motorTimeVSD = &packets[PACKET6];
packetPointer motorFrequencyVSD = &packets[PACKET7];
packetPointer motorTemperatureVSD = &packets[PACKET8];

/////////////////////////VSD WRITES/////////////////////////////
packetPointer commandWordVSD = &packets[PACKET9];
packetPointer userSetSpeedVSD = &packets[PACKET10];

////////VSD READ VARIABLES///////////
unsigned int readStatusWordVSD[1];
unsigned int readActualSpeedVSD[1];
unsigned int readMotorCurrentVSD[1];
unsigned int readMotorPowerVSD[1];
unsigned int readMotorVoltageVSD[1];
unsigned int readMotorTimeVSD[1];
unsigned int readMotorFrequencyVSD[1];
unsigned int readMotorTemperatureVSD[1];

////////VSD WRITE VARIABLES//////////
unsigned int writeControlWordVSD[1];
unsigned int writeUserSetSpeedVSD[1]={0};

///////4D DISPLAY READ VALUES///////
unsigned int enableButton;
unsigned int fwdButton;
unsigned int revButton;
unsigned int resetButton;
unsigned int estopButton;
unsigned int usersetspeedSlider;

//////4D DISPLAY WRITE VALUES///////
unsigned int actualSpeed;
unsigned int actualCurrent;
unsigned int actualVoltage;
unsigned int actualFrequency;
unsigned int actualTemp;


// High or Low variables as arrays (Not used)
unsigned int writeHigh[1]={1};
unsigned int writeLow[1]={0};

const int gndPin =  21;

unsigned int BUTTONSTATE=0;
unsigned int PREVBUTTONSTATE;


void setup()
{
 Serial.begin(19200);                                     // Start the Arduino IDE terminal in case we need it for debugging.
      
  genieBegin (GENIE_SERIAL_1, 115200);  //Serial1
  genieAttachEventHandler(myGenieEventHandler);           // This event handler function is written below in long format. It is what deals with the data coming back from the 4D Systems display.
  
  pinMode(4, OUTPUT);                                     // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(4, 1);                                     // Reset the Display via D4
  delay(100);
  digitalWrite(4, 0);                                     // unReset the Display via D4
  delay (3500);                                           //let the display start up
   
  // Set modes of some pins for LED outputs (Not used)
  pinMode(gndPin, OUTPUT);
  digitalWrite(gndPin, LOW);
   
  // Read all values from VSD... Constructs the Modbus packets for a particular slave address, gives the type of function to be performed, the starting address of the registers to be interrogated, 
  // the number of registers to be read, and the variable to read the data into.
  modbus_construct(statusWordVSD, 2, READ_HOLDING_REGISTERS, 8603, 1, readStatusWordVSD);
  modbus_construct(actualSpeedVSD, 2, READ_HOLDING_REGISTERS, 8604, 1, readActualSpeedVSD);
  modbus_construct(motorCurrentVSD, 2, READ_HOLDING_REGISTERS, 3204, 1, readMotorCurrentVSD);
  modbus_construct(motorPowerVSD, 2, READ_HOLDING_REGISTERS, 3211, 1, readMotorPowerVSD);
  modbus_construct(motorVoltageVSD, 2, READ_HOLDING_REGISTERS, 3207, 1, readMotorVoltageVSD);
  modbus_construct(motorTimeVSD, 2, READ_HOLDING_REGISTERS, 3233, 1, readMotorTimeVSD);
  modbus_construct(motorFrequencyVSD, 2, READ_HOLDING_REGISTERS, 3202, 1, readMotorFrequencyVSD);
  modbus_construct(motorTemperatureVSD, 2, READ_HOLDING_REGISTERS, 3209, 1, readMotorTemperatureVSD);
  
  // Write required values to VSD.. these are the registers which we need to write to on the VSD in order to get it to turn on and off.
  modbus_construct(commandWordVSD, 2, PRESET_MULTIPLE_REGISTERS, 8601, 1, writeControlWordVSD);
  modbus_construct(userSetSpeedVSD, 2, PRESET_MULTIPLE_REGISTERS, 8602, 1, writeUserSetSpeedVSD);
  //modbus_construct(clearFaultsVSD, 2, PRESET_MULTIPLE_REGISTERS, 8501, 1, writeClearFaultsVSD);
  
  // Configure the MODBUS connection... note the data format is SERIAL_8E1. We set it at this so that the Schneider VSD would be able to have a common data format with the Arduino library.
  modbus_configure(baud, SERIAL_8E1, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS);
  
  pinMode(21,OUTPUT); //(Not used)
  digitalWrite(21,LOW);
}

void loop()
{
  update_ALL();
  update_buttons();           // Checks which button state we are in and updates the Modbus messages and display objects accordingly.
  genieDoEvents();            // Check for any Message Reports from the 4D Systems display.
  
}

void update_ALL()
{
  
  while(!Serial1.available()) // ONLY update the Modbus (Which is on Serial2) if there is no data coming from the 4D Systems display on Serial 1.
  {
   modbus_update();           // Polls all of the registers on Modbus slave with address 2 (The Schneider VSD). The address is set under SETUP(). Reads from the VSD and writes to the VSD.
  }
  genieDoEvents();            // Check for any Message Reports from the 4D Systems display.
  update_display();           // Writes and reads required data to and from the objects on the 4D Systems display.
  genieDoEvents();            // Check for any Message Reports from the 4D Systems display.
}

///////////////////////////////////////////////////////////////CHECK THE BUTTON PRESSES//////////////////////////////////////////////////////


void update_buttons()
  {
  switch(BUTTONSTATE){
   
   case 0:
        
        while(BUTTONSTATE==0)
        {
          Serial.println(BUTTONSTATE);
          writeControlWordVSD[0]=0;                          // When no buttons are pressed, write a "0" to the VSD control word. This is a "stop command". See the Modbus control manual in the GITHUB Repository
                                                             // https://github.com/aetcnc/Arduino_4Dsystems_VSD/tree/master/PDF%20Resources Page 10 of the ATV312_comm_var PDF explains the command sequence to
                                                             // get the drive running.
          genieWriteObject(GENIE_OBJ_LED, 0x00, 0);
          genieWriteObject(GENIE_OBJ_LED, 0x01, 1);
          delay(10);
          modbus_update();
          update_ALL();
                  
        }
        
   break;
   
   case 1:
                          
        while(BUTTONSTATE==1)
        {
         Serial.println(BUTTONSTATE); 
         writeControlWordVSD[0]=6;                          // Case 1 means that the "Enable" button has been pressed, so write a "6" to the control word of the VSD. Then read back the status word so that we can
                                                            // see that the command was accepted and that a "7" can now be written to take the drive from "nST" to "Ready".
         genieWriteObject(GENIE_OBJ_LED, 0x00, 0);
         genieWriteObject(GENIE_OBJ_LED, 0x01, 1);
         //Serial.println("Writing Control word = 6");
         if(readStatusWordVSD[0]==545)
          {
            writeControlWordVSD[0]=7;                       // A "7" is now written to the command word to get the drive into the "Ready" state. These steps MUST be done before you can get the drive to switch on!
            //Serial.println("Writing Control word = 7");
            
          }
          delay(10);
          modbus_update();
          //delay(10);
          update_ALL();
          
        }
        
        
   break;
   
   case 2:                                                  // If only the FWD button is pressed, with no Enable button, then reset the button state on the display and in the variable on the Arduino which stores the state.
        
        while(BUTTONSTATE==2)
        {
         Serial.println(BUTTONSTATE);
         writeControlWordVSD[0]=0;
         genieWriteObject(GENIE_OBJ_LED, 0x00, 0);
         genieWriteObject(GENIE_OBJ_LED, 0x01, 1);
         genieWriteObject(GENIE_OBJ_USERBUTTON, 0x02, 0);
         fwdButton=0;
         delay(10);
         modbus_update();
         update_ALL();
        }
                
   break;
   
   case 3:
   
       while(BUTTONSTATE==3)
        {
          Serial.println(BUTTONSTATE);
          if(readStatusWordVSD[0]==547)
          {
            writeControlWordVSD[0]=15;                      // If the status word has changed to "547" then the drive is ready to switch on and "15" can be written to the command word. This will now cause the motor to
                                                            // rotate CLOCKWISE!
            genieWriteObject(GENIE_OBJ_LED, 0x00, 1);
            genieWriteObject(GENIE_OBJ_LED, 0x01, 0);
            //Serial.println("Writing Control word = 15");
          }
          delay(10);
          modbus_update();
          //delay(10);
          update_ALL();
          
        }
        
        //modbus_update();
        
   break;
   
   case 4:                                                  // If only the REV button is pressed, with no Enable button, then reset the button state on the display and in the variable on the Arduino which stores the state.
        
        while(BUTTONSTATE==4)
        {
         Serial.println(BUTTONSTATE);
         writeControlWordVSD[0]=0;
         genieWriteObject(GENIE_OBJ_LED, 0x00, 0);
         genieWriteObject(GENIE_OBJ_LED, 0x01, 1);
         genieWriteObject(GENIE_OBJ_USERBUTTON, 0x03, 0);
         revButton=0;
         delay(10);
         modbus_update();
         update_ALL(); 
        }
        
   break;
   
   case 5:
       
       while(BUTTONSTATE==5)
        {
          Serial.println(BUTTONSTATE);
          if(readStatusWordVSD[0]==547)                        // If the status word has changed to "547" then the drive is ready to switch on and "2063" can be written to the command word. This will now cause the motor to
                                                               // rotate COUNTER-CLOCKWISE!
          {
            writeControlWordVSD[0]=2063;
            genieWriteObject(GENIE_OBJ_LED, 0x00, 1);
            genieWriteObject(GENIE_OBJ_LED, 0x01, 0);
          }
          delay(10);
          modbus_update();
          update_ALL();
          
        }
        
        
        
   break;
   
   case 6:                                                      // If both the FWD and REV buttons are pressed, with no Enable button, then reset both button states on the display and in the variables on the Arduino which store the states.
                                                                // We also write a "0" to the control word to put the drive back into a shutdown state. (As would be a dangerous state request once Enable was pressed and could have damaged equipment or injured people.
        while(BUTTONSTATE==6)
        {
         Serial.println(BUTTONSTATE);
         writeControlWordVSD[0]=0;
         genieWriteObject(GENIE_OBJ_LED, 0x00, 0);
         genieWriteObject(GENIE_OBJ_LED, 0x01, 1);
         genieWriteObject(GENIE_OBJ_USERBUTTON, 0x02, 0);
         fwdButton=0;
         genieWriteObject(GENIE_OBJ_USERBUTTON, 0x03, 0);
         revButton=0;
         delay(10);
         modbus_update();
         update_ALL();
          
        }
        
   break;
   
   case 7:                                                      // If both the FWD and REV buttons are pressed, WITH the Enable button, then reset both button states on the display and in the variables on the Arduino which store the states.
                                                                // We also write a "0" to the control word to put the drive back into a shutdown state. (As it was a dangerous state request and could have damaged equipment or injured people.
        
        while(BUTTONSTATE==7)
        {
         writeControlWordVSD[0]=0;
         genieWriteObject(GENIE_OBJ_LED, 0x00, 0);
         genieWriteObject(GENIE_OBJ_LED, 0x01, 1);
         genieWriteObject(GENIE_OBJ_USERBUTTON, 0x02, 0);
         fwdButton=0;
         genieWriteObject(GENIE_OBJ_USERBUTTON, 0x03, 0);
         revButton=0;
         delay(10);
         modbus_update(); 
         update_ALL();
          
        }
        
   break;
   
   case 8:                                                      // For future use of the Fault reset funcion on the VSD. Not currently used.
        
        while(BUTTONSTATE==8)
        {
         update_ALL();
          
        }
        
   break;
   
   case 9:                                                      // For future use of the Fault reset funcion on the VSD. Not currently used.
        
        while(BUTTONSTATE==9)
        {
         update_ALL();
        }
        
   break;
   
   case 10:                                                     // For future use of the Fault reset funcion on the VSD. Not currently used.
        
        while(BUTTONSTATE==10)
        {
         update_ALL();
        }
        
   break;
   
   case 16:                                                     // THE ESTOP HAS BEEN PRESSED!!!!!!!! Send a "0" to the control word to shut the drive down immediately and reset all the buttons, sliders and button registers,
                                                                // as well as the Modbus speed register. We don't want to come out of E-stop and have the drive begin running! Everything must be started up again from scratch.
                                                                // This is not entirely correct, as the code below first writes to the display before killing the drive. This is not good! Always stop motion first and then
                                                                // give feedback to the user about what has happened. We have left it like this here to illustrate an important point!!!Remember, this code is NOT for commercial use!   
     genieWriteObject(GENIE_OBJ_USERBUTTON, 0x00, 0);
     enableButton=0;
     genieWriteObject(GENIE_OBJ_USERBUTTON, 0x02, 0);
     fwdButton=0;
     genieWriteObject(GENIE_OBJ_USERBUTTON, 0x03, 0);
     revButton=0;
     genieWriteObject(GENIE_OBJ_USERBUTTON, 0x04, 0);
     resetButton=0;
     genieWriteObject(GENIE_OBJ_SLIDER, 0x00, 0);
     usersetspeedSlider=0;
     writeUserSetSpeedVSD[0]=0;
     genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x01, 0);
     genieWriteObject(GENIE_OBJ_LED, 0x00, 0);
     genieWriteObject(GENIE_OBJ_LED, 0x01, 1);
        while(BUTTONSTATE==16)
        {
          Serial.println(BUTTONSTATE);
          writeControlWordVSD[0]=0;
          delay(10);
          modbus_update();
          update_ALL();
        }
        
        
        
   break;
   
   default:
   main:
   break;
    
  }
}

void checkState()                                                    // Bit-shifts and adds each of the button registers to create a unique number for each potential button press combination. This makes it easy to use a 
                                                                     // Switch-Case control structure to execute commands for different states.
{
  PREVBUTTONSTATE=BUTTONSTATE;
      
  BUTTONSTATE = (enableButton)+(fwdButton<<1)+(revButton<<2)+(resetButton<<3)+(estopButton<<4);
  
  if(BUTTONSTATE==11||BUTTONSTATE==12||BUTTONSTATE==13||BUTTONSTATE==14||BUTTONSTATE==15)
  {
    BUTTONSTATE=10;
  }
  
  if(BUTTONSTATE==17||BUTTONSTATE==18||BUTTONSTATE==19||BUTTONSTATE==20||BUTTONSTATE==21||BUTTONSTATE==22||BUTTONSTATE==23||BUTTONSTATE==24||BUTTONSTATE==25||BUTTONSTATE==26||BUTTONSTATE==27||BUTTONSTATE==28||BUTTONSTATE==29||BUTTONSTATE==30||BUTTONSTATE==31)
  {
    BUTTONSTATE=16;
  }
  Serial.println(BUTTONSTATE);    
}

void myGenieEventHandler(void)                                       // This is extremely important and must be adapted to whatever objects you have on your display. This is only going to work with OUR display example.
                                                                     // Make sure you read all of the 4D Syatems "Visi-Genie" application notes to see how to correctly control the display. http://www.4dsystems.com.au/appnotes/
{
  genieFrame Event;
  genieDequeueEvent(&Event);
  
  //If the cmd received is from a Reported Event
  if(Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    
    if (Event.reportObject.object == GENIE_OBJ_USERBUTTON)                // If the Reported Message was from a USERBUTTON
    {
      if (Event.reportObject.index == 0)                                  // If it was UserButton0
      {
        enableButton = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb;  // Save the state to "enableButton" to be used in checkState().  
                
      }
      else if (Event.reportObject.index == 2)                              // Otherwise, if it was UserButton2
      {
        fwdButton = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb;    
             
      }
      else if (Event.reportObject.index == 3)                              // Otherwise, if it was UserButton3
      {
        revButton = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb;    
        
      }
      else if (Event.reportObject.index == 4)                              // ...
      {
        resetButton = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb;    
        
      }
      else if (Event.reportObject.index == 1)                              // ...
      {
        estopButton = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb;    
        
      }
    }
    if (Event.reportObject.object == GENIE_OBJ_SLIDER)                // If the Reported Message was from a Slider
    {
      if (Event.reportObject.index == 0)                              // If Slider0
      {
        usersetspeedSlider = (Event.reportObject.data_msb << 8) + Event.reportObject.data_lsb;  // Slider0 data into the slider_val setpoint
        writeUserSetSpeedVSD[0]=usersetspeedSlider;
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x01, usersetspeedSlider);
                
      }
    }
  }
  // Now that we have read in whichever buttons were pressed, we can use the checkState() function to get a unique number for the button condition which we are in.
  checkState(); 
}

void update_display()
{
  static long waitPeriod = millis();
  int gaugeVal;                                                        // Not used
  char buffer[5];                                                      // Not used
  float motorCurrent = readMotorCurrentVSD[0];
  int voltage = readMotorVoltageVSD[0]/10;
  int current = readMotorCurrentVSD[0]/10;
  int frequency = readMotorFrequencyVSD[0]/10;
  int actualSpeed = abs(readActualSpeedVSD[0]); 
  
  if (millis() >= waitPeriod) 
  {
                                                                       // Update all of the neccessary objects on the display with values from the VSD modbus reads and the user speed input slider.
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x02, readMotorTemperatureVSD[0]);
    
    genieWriteObject(GENIE_OBJ_GAUGE, 0x00, readMotorTemperatureVSD[0]);
    
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x03, readMotorFrequencyVSD[0]);
    
    genieWriteObject(GENIE_OBJ_GAUGE, 0x01, frequency);
    
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x04, readMotorVoltageVSD[0]);
    
    genieWriteObject(GENIE_OBJ_GAUGE, 0x02, voltage);
        
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x05, readMotorCurrentVSD[0]);
    
    genieWriteObject(GENIE_OBJ_GAUGE, 0x03, current);
    
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x01, usersetspeedSlider);     
    
    genieWriteObject(GENIE_OBJ_LED_DIGITS, 0x00, actualSpeed);
    
       
                                                                      // Read all of the buttons and check their states. (We're not entirely sure that these lines are neccessary as the genieEventHandler() is supposedly interrupt driven
                                                                      // and should automatically update on a button press. This works as it is though. Feel free to test it out and let us know :)
    genieReadObject(GENIE_OBJ_USERBUTTON, 0x00);
    
    genieReadObject(GENIE_OBJ_USERBUTTON, 0x01);
    
    genieReadObject(GENIE_OBJ_USERBUTTON, 0x02);
    
    genieReadObject(GENIE_OBJ_USERBUTTON, 0x03);
    
    genieReadObject(GENIE_OBJ_USERBUTTON, 0x04);
    
    
    waitPeriod = millis() + 20;
  }
    
}

