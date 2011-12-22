// **********************************************************
// **                   OpenLRS Functions                  **
// **       This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2011-09-26
// Supported Hardware : OpenLRS Tx boards (M1 & M2) (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************

//############# RED LED BLINK #################
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
  
//############# GREEN LED BLINK #################
void Green_LED_Blink(unsigned short blink_count)
  {
  unsigned char i;
  for (i=0;i<blink_count;i++)
     {
     delay(100);
     Green_LED_ON;
     delay(100);
     Green_LED_OFF;
     }
  }  

//############# FREQUENCY HOPPING #################
#if (FREQUENCY_HOPPING==1)
void Hopping(void){

hopping_channel++;
if (hopping_channel>2) hopping_channel = 0;

_spi_write(0x79, hop_list[hopping_channel]);

    #if (DEBUG_MODE == 5)
      Serial.println(int(hop_list[hopping_channel]));
    #endif  

}
#endif

//############# RF POWER SETUP #################
void Power_Set(unsigned short level)
{
  //Power Level value between 0-7
  //0 = +1 dBm
  //1 = +2 dBm
  //2 = +5 dBm
  //3 = +8 dBm
  //4 = +11 dBm
  //5 = +14 dBm
  //6 = +17 dBm
  //7 = +20 dB 
  if (level<8) _spi_write(0x6d, level);  
  
}

//############# BUTTON CHECK #################
void Check_Button(void)
{
unsigned long loop_time;


 if (digitalRead(BTN)==0) // Check the button
    {
    delay(1000); // wait for 1000mS when buzzer ON 
    digitalWrite(BUZZER, LOW); // Buzzer off
    
    time = millis();  //set the current time
    loop_time = time; 
    
    while ((digitalRead(BTN)==0) && (loop_time < time + 4000)) // wait for button reelase if it is already pressed.
        {
         loop_time = millis(); 
        }     
        
    //Check the button again. If it is already pressed start the binding proscedure    
    if (digitalRead(BTN)==0) // Binding Mode
        {
        randomSeed(analogRead(7)); //use empty analog pin as random value seeder.
        loop_time = millis(); // count the button pressed time for extra randomization
        
        digitalWrite(BUZZER, HIGH); //100ms single beep for binding mode.
        delay(100);
        digitalWrite(BUZZER, LOW);
        
        while((digitalRead(BTN)==0) ) {}; // wait for button release
        
        //Here is the code for binding. 
        time = millis();
        Binding_Mode(time-loop_time);  
      
        }    
    else // if button released, reduce the power for range test.
        {
        Power_Set(0); //set the minimum output power +1dbm
        }  
        
        
    }
  
  
}

//############# BINDING #################
void Binding_Mode(unsigned int btn_press_time)
{
 
 //randomSeed(analogRead(7)); //use empty analog pin as random value seeder.
 //randNumber = random(300);
 
 //we will write this part soon
 
  
}



void SetServoPos (unsigned char channel,int value)
{
  unsigned char ch = channel*2;
  Servo_Buffer[ch] = value/256;
  Servo_Buffer[ch+1] = value%256;
}
