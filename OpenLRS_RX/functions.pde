// **********************************************************
// **                   OpenLRS Functions                  **
// **        Developed by Melih Karakelle on 2010-2011     **
// **          This Source code licensed under GPL         **
// **********************************************************
// Latest Code Update : 2011-10-04
// Supported Hardware : OpenLRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/


void INIT_SERVO_DRIVER(void)
{
   TCCR1B   =   0x00;   //stop timer
   TCNT1H   =   0x00;   //setup
   TCNT1L   =   0x00;
   ICR1   =   40000;   // used for TOP, makes for 50 hz
   
   TCCR1A   =   0x02;   
   TCCR1B   =   0x1A; //start timer with 1/8 prescaler for 0.5us PPM resolution
   
   TIMSK1 = _BV (TOIE1);   
} 

void RFM22B_Int()
{
 if (RF_Mode == Transmit) 
    {
     RF_Mode = Transmitted; 
    } 
 if (RF_Mode == Receive) 
    {
     RF_Mode = Received; 
    }  
}

void Red_LED_Blink(unsigned short blink_count)
  {
  unsigned char i;
  for (i=0;i<blink_count;i++)
     {
     delay(100);
     Red_LED_ON;
     delay(100);
     Red_LED_OFF;
     }
  }



void load_failsafe_values(){
 
  for (int i=0;i<8;i++)
      Servo_Buffer[i] =  (EEPROM.read(11+(2*i)) * 256) + EEPROM.read(12+(2*i));
  
  #if (DEBUG_MODE == 4)
      Serial.print("1:");
      Serial.println(Servo_Buffer[0]); // value x 0.5uS = PPM time, 3000 x 0.5 = 1500uS 
      Serial.print("2:");
      Serial.println(Servo_Buffer[1]);
      Serial.print("3:");
      Serial.println(Servo_Buffer[2]);
      Serial.print("4:");
      Serial.println(Servo_Buffer[3]);
      Serial.print("5:");
      Serial.println(Servo_Buffer[4]);
      Serial.print("6:");
      Serial.println(Servo_Buffer[5]);
      Serial.print("7:");
      Serial.println(Servo_Buffer[6]);
      Serial.print("8:");
      Serial.println(Servo_Buffer[7]); 
   #endif
     

}


void save_failsafe_values(void){

 for (int i=0;i<8;i++)
    {
     EEPROM.write(11+(2*i),Servo_Buffer[i] / 256); 
     EEPROM.write(12+(2*i),Servo_Buffer[i] % 256);
    } 
}

unsigned char check_modes(void){

//-- Serial PPM Selection (jumper between Ch1 and ch3)
pinMode(Servo3_OUT, INPUT); //CH3 input
digitalWrite(Servo3_OUT, HIGH); // pull up
digitalWrite(Servo1_OUT, HIGH); // CH1 is HIGH
delayMicroseconds(1);
if ( digitalRead(Servo3_OUT) == HIGH) 
	{
	digitalWrite(Servo1_OUT, LOW); // CH1 is LOW
	delayMicroseconds(1);
	if (digitalRead(Servo3_OUT) == LOW) // OK jumper plugged
			{
                        pinMode(Servo3_OUT, OUTPUT);
			return  1; //Serial PPM OUT
			}
	}
	

pinMode(Servo3_OUT, OUTPUT);

return  0; // Parallel PPM OUT
}

//############# FREQUENCY HOPPING FUNCTIONS #################
#if (FREQUENCY_HOPPING==1)
void Hopping(void)
    {
    hopping_channel++;
    if (hopping_channel>2) hopping_channel = 0;
    _spi_write(0x79, hop_list[hopping_channel]);
    
    #if (DEBUG_MODE == 5)
      Serial.println(int(hop_list[hopping_channel]));
    #endif  
    }
#endif

void Direct_Servo_Drive(void)
    {
    Servo_Position[AILERON] = Servo_Buffer[AILERON];  
    Servo_Position[ELEVATOR] = Servo_Buffer[ELEVATOR];  
    Servo_Position[THROTTLE] = Servo_Buffer[THROTTLE];  
    Servo_Position[RUDDER] = Servo_Buffer[RUDDER];  
    Servo_Position[RETRACTS] = Servo_Buffer[RETRACTS];  
    Servo_Position[FLAPS] = Servo_Buffer[FLAPS];  
    Servo_Position[AUX1] = Servo_Buffer[AUX1];  
    Servo_Position[AUX2] = Servo_Buffer[AUX2];  
    
    #if (DEBUG_MODE == 1)
           if ((Servo_Position[2]<2350) || (Servo_Position[2]>2360)) Serial.println(int(Servo_Position[2]));
    #endif  
    }
    
    
void Gyro_Stabilized_Servo_Drive(void)
    {
    Servo_Position[AILERON] = Servo_Buffer[AILERON] + (Gyro_ROLL / Gyro_Roll_Gain);  //Aileron  
    Servo_Position[ELEVATOR] = Servo_Buffer[ELEVATOR] + (Gyro_PITCH / Gyro_Pitch_Gain);  //Elevator  
    Servo_Position[THROTTLE] = Servo_Buffer[THROTTLE];
    Servo_Position[RUDDER] = Servo_Buffer[RUDDER] + (Gyro_YAW / Gyro_Yaw_Gain);  //Rudder    
    Servo_Position[RETRACTS] = Servo_Buffer[RETRACTS];  
    Servo_Position[FLAPS] = Servo_Buffer[FLAPS];  
    Servo_Position[AUX1] = Servo_Buffer[AUX1];  
    Servo_Position[AUX2] = Servo_Buffer[AUX2];  
    
    #if (DUAL_AILERON==1)
      #if (DUAL_AILERON_DIRECTION == 1)
        Servo_Position[DUAL_AILERON_SERVO] = Servo_Position[AILERON]  // Aileron flap 2
      #else
        Servo_Position[DUAL_AILERON_SERVO] = 6000 - Servo_Position[AILERON];  // reversed Aileron flap 2
      #endif      
    #endif
    }    

void Basic_Quadro_Copter_Servo_Drive(void)
    {
    //######### EXPERIMENTAL CODE PART, DONT USE IT ########  
    // It works with Wii Motion Plus  
    Servo_Position[0] = Servo_Buffer[2] + (Gyro_PITCH / Gyro_Pitch_Gain) - (Gyro_YAW / Gyro_Yaw_Gain);  //front motor 
    Servo_Position[1] = Servo_Buffer[2] - (Gyro_ROLL / Gyro_Roll_Gain) + (Gyro_YAW / Gyro_Yaw_Gain);  //right motor    
    Servo_Position[2] = Servo_Buffer[2] - (Gyro_PITCH / Gyro_Pitch_Gain) - (Gyro_YAW / Gyro_Yaw_Gain);  //back motor
    Servo_Position[3] = Servo_Buffer[2] + (Gyro_ROLL / Gyro_Roll_Gain) + (Gyro_YAW / Gyro_Yaw_Gain);  //left motor 
    
    if (Servo_Buffer[0]>3000) Servo_Position[1] += (Servo_Buffer[0] - 3000) / 4 ;
    if (Servo_Buffer[0]<3000) Servo_Position[3] += (3000 - Servo_Buffer[0]) / 4;
    if (Servo_Buffer[1]>3000) Servo_Position[0] += (Servo_Buffer[1] - 3000) / 4 ;
    if (Servo_Buffer[1]<3000) Servo_Position[2] += (3000 - Servo_Buffer[1]) / 4 ;
    
    Servo_Position[4] = Servo_Buffer[4];  
    Servo_Position[5] = Servo_Buffer[5];  
    Servo_Position[6] = Servo_Buffer[6];  
    Servo_Position[7] = Servo_Buffer[7];  
    }   


//######### TELEMETRY LOOP ############
void Telemetry_Write(void)
{
loop_counter++;
if (loop_counter>50) loop_counter=1; 

  
RF_Tx_Buffer[0]= 'T';
RF_Tx_Buffer[1]= Rx_RSSI;

#if defined(GPS)
if (loop_counter==1) SET_Telemetry(parts[1],10,1);
if (loop_counter==2) SET_Telemetry(parts[2],10,2);
if (loop_counter==3) SET_Telemetry(parts[3],10,3);
if (loop_counter==4) SET_Telemetry(parts[4],10,4);
if (loop_counter==5) SET_Telemetry(parts[5],10,5);
if (loop_counter==6) SET_Telemetry(parts[6],10,6);
if (loop_counter==7) SET_Telemetry(parts[7],10,7);
if (loop_counter==8) SET_Telemetry(parts[8],10,8);
if (loop_counter==9) SET_Telemetry(parts[9],10,9);
if (loop_counter==10) SET_Telemetry(parts[10],10,10);
#endif

#if defined(BMP085)

if (loop_counter==25) SET_Telemetry_i( (APM_BMP085.GetAltitude() ),5,1); 

#endif


to_tx_mode();
rx_reset(); 

// Clear buffer (dont use "for loop")
 RF_Tx_Buffer[2] = '0';
 RF_Tx_Buffer[3] = '0';
 RF_Tx_Buffer[4] = '0';
 RF_Tx_Buffer[5] = '0';
 RF_Tx_Buffer[6] = '0';
 RF_Tx_Buffer[7] = '0';
 RF_Tx_Buffer[8] = '0';
 RF_Tx_Buffer[9] = '0';
 RF_Tx_Buffer[10] = '0';
 RF_Tx_Buffer[11] = '0';
 RF_Tx_Buffer[12] = '0';
 RF_Tx_Buffer[13] = '0';
 RF_Tx_Buffer[14] = '0';
 RF_Tx_Buffer[15] = '0';
 RF_Tx_Buffer[16] = '0';
  
}  

//######## TELEMETRY TRANSPARENT BRIDGE #########
void Telemetry_Bridge_Write(void)
{

RF_Tx_Buffer[0]= 'B'; // Brige command

byte total_rx_byte = Serial.available();  // Read the Serial RX buffer size
if (total_rx_byte>15) total_rx_byte = 15; // Limit the package size as 15 byte

if (total_rx_byte > 0) 
    {
    RF_Tx_Buffer[1]= total_rx_byte;
    for (byte i=0;i<total_rx_byte;i++)
      RF_Tx_Buffer[2+i] = Serial.read();
    }

to_tx_mode();
rx_reset();     

// Clear buffer (dont use "for loop")
 RF_Tx_Buffer[1] = 0;
 RF_Tx_Buffer[2] = 0;
 RF_Tx_Buffer[3] = 0;
 RF_Tx_Buffer[4] = 0;
 RF_Tx_Buffer[5] = 0;
 RF_Tx_Buffer[6] = 0;
 RF_Tx_Buffer[7] = 0;
 RF_Tx_Buffer[8] = 0;
 RF_Tx_Buffer[9] = 0;
 RF_Tx_Buffer[10] = 0;
 RF_Tx_Buffer[11] = 0;
 RF_Tx_Buffer[12] = 0;
 RF_Tx_Buffer[13] = 0;
 RF_Tx_Buffer[14] = 0;
 RF_Tx_Buffer[15] = 0;
 RF_Tx_Buffer[16] = 0;
  
}  


