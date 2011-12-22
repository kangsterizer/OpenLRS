// **********************************************************
// ******************   OpenLRS Tx Code   *******************
// ***  OpenLRS Designed by Melih Karakelle on 2010-2011  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************
// Version Number     : 1.10
// Latest Code Update : 2011-10-04
// Supported Hardware : OpenLRS Tx boards (M1 & M2) (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************

// ******************** OpenLRS DEVELOPERS ****************** 
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne) 
//

#include "config.h"

#include <EEPROM.h>



void setup() {   
        //RF module pins
        pinMode(SDO_pin, INPUT); //SDO
        pinMode(SDI_pin, OUTPUT); //SDI        
	pinMode(SCLK_pin, OUTPUT); //SCLK
        pinMode(IRQ_pin, INPUT); //IRQ
        pinMode(nSel_pin, OUTPUT); //nSEL
        
        //LED and other interfaces
        pinMode(Red_LED, OUTPUT); //RED LED
        pinMode(Green_LED, OUTPUT); //GREEN LED
        pinMode(BUZZER, OUTPUT); //Buzzer
        pinMode(BTN, INPUT); //Buton
        
           
        pinMode(PPM_IN, INPUT); //PPM from TX 
        pinMode(RF_OUT_INDICATOR, OUTPUT);
        
        Serial.begin(SERIAL_BAUD_RATE);
        
        
       #if (CONTROL_TYPE == 0)
         PPM_Pin_Interrupt_Setup // turnon pinchange interrupts
       #endif
       
      
       for (unsigned char i=0;i<8;i++) // set defoult servo position values.
          SetServoPos(i,3000); // set the center position

         TCCR1B   =   0x00;   //stop timer
         TCNT1H   =   0x00;   //setup
         TCNT1L   =   0x00;
         ICR1     =   60005;   // used for TOP, makes for 50 hz
         TCCR1A   =   0x02;   
         TCCR1B   =   0x1A; //start timer with 1/8 prescaler for measuring 0.5us PPM resolution
}


#if (CONTROL_TYPE == 0) 
//##### PPM INPUT INTERRUPT #####
//Port change interrupt detects the PPM signal's rising edge and calculates the signal timing from Timer1's value.
	
ISR(PPM_Signal_Interrupt){

unsigned int time_temp;
unsigned int servo_temp;
unsigned int servo_temp2;

if (PPM_Signal_Edge_Check) // Only works with rising edge of the signal
		    {
			time_temp = TCNT1; // read the timer1 value
                        TCNT1 = 0; // reset the timer1 value for next
			if (channel_no<14) channel_no++; 
                        
					
			if (time_temp > 8000) // new frame detection : >4ms LOW
				{	
				channel_count = channel_no;
				channel_no = 0;
				transmitted = 0;                               
				}
                                else
                                {
                                if ((time_temp>1500) && (time_temp<4500)) // check the signal time and update the channel if it is between 750us-2250us
                                  {
                                  //Servo_Buffer[(2*channel_no)-1] = (byte) (time_temp >> 8); // write the high byte of the value into the servo value buffer.
			          //Servo_Buffer[2*channel_no] =  (byte) (time_temp); // write the low byte of the value into the servo value buffer.                                    
                                  SetServoPos(channel_no-1,time_temp);
                                  }
                               }
			}

}
#endif 

//############ MAIN LOOP ##############
void loop() {
  
unsigned char i;

//wdt_enable(WDTO_1S);

RF22B_init_parameter(); 
frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RF module for this frequency, frequency hopping starts from this frequency.

sei();

digitalWrite(BUZZER, HIGH);
digitalWrite(BTN, HIGH);
Red_LED_ON ;
delay(100);	

Check_Button();

Red_LED_OFF;
digitalWrite(BUZZER, LOW);

digitalWrite(RF_OUT_INDICATOR,LOW);
digitalWrite(PPM_IN,HIGH);

transmitted = 0;
rx_reset;
	
time = millis();
old_time = time;

while(1)
              {    /* MAIN LOOP */	              
              time = millis();  
              if (_spi_read(0x0C)==0) // detect the locked module and reboot
                 {
                 Red_LED_ON;  
                 RF22B_init_parameter();
                 frequency_configurator(CARRIER_FREQUENCY);
                 rx_reset;
                 Red_LED_OFF;
                 }
              
              #if (CONTROL_TYPE==1)
              if (Serial.available()>3)    // Serial command received from the PC
                        {
                          int cmd = Serial.read();
                          if (cmd=='S')            // Command 'S'+ channel number(1 bytes) + position (2 bytes)
                          {
                            Red_LED_ON;
                            int ch = Serial.read();
                            Servo_Buffer[2*ch] = Serial.read();
                            Servo_Buffer[(2*ch)+1] = Serial.read();
                            Red_LED_OFF;
                          }
                        }
              #endif       


              #if (TELEMETRY_ENABLED==1)
        
               if (nIRQ_0)
                 {
                  Red_LED_ON;  
                  send_read_address(0x7f); // Send the package read command
		  for(i = 0; i<17; i++) //read all buffer 
			{ 
			 RF_Rx_Buffer[i] = read_8bit_data(); 
			}  
		  rx_reset(); 
                  
                  #if (TELEMETRY_MODE == 1)  // OpenLRS Standard Telemetry mode                
                      #if (Rx_RSSI_Alert_Level>0) 
                        Tx_RSSI = ((Tx_RSSI/5)*4) + (_spi_read(0x26)/5); // Read the RSSI value
                      #endif
                      #if (Rx_RSSI_Alert_Level>0) 
                        Rx_RSSI = ((Rx_RSSI/5)*4) + (RF_Rx_Buffer[1]/5); // Rx Rssi value from telemetry data
                      #endif
                      if ((Rx_RSSI < Rx_RSSI_Alert_Level)||(Tx_RSSI < Tx_RSSI_Alert_Level)) // RSSI level alerts
                         digitalWrite(BUZZER, HIGH);
                         else
                         digitalWrite(BUZZER, LOW);
                      #if (TELEMETRY_OUTPUT_ENABLED==1)
                        for(i = 0; i<16; i++) //write serial
                           Serial.print(RF_Rx_Buffer[i]);
                        Serial.println(int(RF_Rx_Buffer[16]));
                      #endif                  
                   #endif
                   
                   #if (TELEMETRY_MODE == 0)  // Transparent Bridge Telemetry mode                
                      #if (TELEMETRY_OUTPUT_ENABLED==1)
                        if (RF_Rx_Buffer[0]=='B') // Brige values
                           {
                             for(i = 2; i<RF_Rx_Buffer[1]+2; i++) //write serial
                              Serial.print(RF_Rx_Buffer[i]);
                           }   
                      #endif                  
                   #endif
                  
                  Red_LED_OFF;
                  Rx_Pack_Received = 0;
                 }	    
	      #endif

              #if (CONTROL_TYPE == 0)
              if ((transmitted==0) && (channel_count>3) && (channel_count<13))
              #else              
              if (time> old_time+20) // Automatic 50hz position transmit code for PC based serial control applications
              #endif        
                       {
                      old_time = time; 
		      //Green LED will be on during transmission  
	              Green_LED_ON ;


                      #if (TELEMETRY_MODE == 0) // Transparent Brige ground to air code
                          
                          byte total_rx_byte = Serial.available();  // Read the Serial RX buffer size
                          if (total_rx_byte>0)
                              {
                               RF_Tx_Buffer[0] = 'B';
                               if (total_rx_byte>15) total_rx_byte = 15; // Limit the package size as 15 byte
                               RF_Tx_Buffer[1]= total_rx_byte;
                               for (byte i=0;i<total_rx_byte;i++)
                                    RF_Tx_Buffer[2+i] = Serial.read();
                              }
                           else   
                       #endif  

                      {
                      //"S" header for Servo Channel data
                      RF_Tx_Buffer[0] = 'S';
		      for(i = 0; i<16; i++) // fill the rf-tx buffer with 8 channel (2x8 byte) servo signal
		          {
                          RF_Tx_Buffer[i+1] = Servo_Buffer[i];
			  }
                      }
                      	



                      // Send the data over RF
    		      to_tx_mode();
    		      transmitted = 1;
    	              
                      //Green LED will be OFF
                      Green_LED_OFF; 
  
                      //Hop to the next frequency
                      #if (FREQUENCY_HOPPING==1)
                         Hopping();
                      #endif   
                      
                      
                      #if (TELEMETRY_ENABLED==1) //Receiver mode enabled for the telemetry
                        rx_reset(); 
                        #if (Lost_Package_Alert != 0) // Lost Package Alert 
                          if (Rx_Pack_Received < Lost_Package_Alert) // last Rx packs didnt received
                              digitalWrite(BUZZER, LOW);
                              else
                              digitalWrite(BUZZER, HIGH);
                        #endif    
                        Rx_Pack_Received++;
                      #endif
                      
                      #if (DEBUG_MODE == 1)
                         if (time%100 < 10)
                         {
                           for(i = 0; i<8; i++) // fill the rf-tx buffer with 8 channel (2x8 byte) servo signal
		             {
                             Serial.print(int( (Servo_Buffer[(2*i)]*256) + Servo_Buffer[1+(2*i)]));
                             Serial.print(' ');
			     }
                          Serial.println(' ');
                         }
                      #endif  
			  
	              }
		}
		 
		 

}


