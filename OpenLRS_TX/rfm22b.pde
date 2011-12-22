// **********************************************************
// **      RFM22B/Si4432 control functions for OpenLRS     **
// **       This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2011-09-26
// Supported Hardware : OpenLRS Tx/Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************
 
#define NOP() __asm__ __volatile__("nop") 

 
#define RF22B_PWRSTATE_READY    01 
#define RF22B_PWRSTATE_TX        0x09 


#define RF22B_PWRSTATE_RX       05 
#define RF22B_Rx_packet_received_interrupt   0x02 

#define RF22B_PACKET_SENT_INTERRUPT  04 
#define RF22B_PWRSTATE_POWERDOWN  00 

 
unsigned char ItStatus1, ItStatus2; 
 
 
typedef struct   
{ 
 unsigned char reach_1s    : 1; 
} FlagType; 
FlagType               Flag; 
 
unsigned char read_8bit_data(void); 
void to_tx_mode(void); 
void to_ready_mode(void); 
void send_8bit_data(unsigned char i); 
void send_read_address(unsigned char i); 
void _spi_write(unsigned char address, unsigned char data); 
void RF22B_init_parameter(void); 

void port_init(void);   
unsigned char _spi_read(unsigned char address); 
void Write0( void ); 
void Write1( void ); 
void timer2_init(void); 
void Write8bitcommand(unsigned char command); 
void to_sleep_mode(void); 
 
 
//***************************************************************************** 
//***************************************************************************** 

//-------------------------------------------------------------- 
void Write0( void ) 
{ 
    SCK_off;  
    NOP(); 
     
    SDI_off; 
    NOP(); 
     
    SCK_on;  
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write1( void ) 
{ 
    SCK_off;
    NOP(); 
     
    SDI_on;
    NOP(); 
     
    SCK_on; 
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write8bitcommand(unsigned char command)    // keep sel to low 
{ 
 unsigned char n=8; 
    nSEL_on;
    SCK_off;
    nSEL_off; 
    while(n--) 
    { 
         if(command&0x80) 
          Write1(); 
         else 
          Write0();    
              command = command << 1; 
    } 
    SCK_off;
}  


//-------------------------------------------------------------- 
unsigned char _spi_read(unsigned char address) 
{ 
 unsigned char result; 
 send_read_address(address); 
 result = read_8bit_data();  
 nSEL_on; 
 return(result); 
}  

//-------------------------------------------------------------- 
void _spi_write(unsigned char address, unsigned char data) 
{ 
 address |= 0x80; 
 Write8bitcommand(address); 
 send_8bit_data(data);  
 nSEL_on;
}  


//-------Defaults 38.400 baud---------------------------------------------- 
void RF22B_init_parameter(void) 
{ 
 ItStatus1 = _spi_read(0x03); // read status, clear interrupt   
 ItStatus2 = _spi_read(0x04); 
  _spi_write(0x06, 0x00);    // no wakeup up, lbd, 
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode 
  _spi_write(0x09, 0x7f);  // c = 12.5p   
  _spi_write(0x0a, 0x05); 
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio1 RX State 
  //  -- Old PCB --
  //  _spi_write(0x0b, 0x15);    // gpio0 RX State
  //  _spi_write(0x0c, 0x12);    // gpio1 TX State 
  _spi_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output 
  _spi_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION. 
  
  _spi_write(0x70, 0x00);    // disable manchest 
  
   
  /* // 38.4Kbps data rate
  _spi_write(0x6e, 0x09); //case RATE_384K 
  _spi_write(0x6f, 0xD5); //case RATE_384K
  
  _spi_write(0x1c, 0x02); // case RATE_384K
  _spi_write(0x20, 0x68);//  0x20 calculate from the datasheet= 500*(1+2*down3_bypass)/(2^ndec*RB*(1+enmanch)) 
  _spi_write(0x21, 0x01); // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0x3A); // 0x22    ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0x93); // 0x23 
  _spi_write(0x24, 0x02); // 0x24 
  _spi_write(0x25, 0x6B); // 0x25 
  _spi_write(0x2a, 0x1e); 
  */
    
  // 57.6Kbps data rate
  _spi_write(0x1c, 0x05); // case RATE_57.6K
  _spi_write(0x20, 0x45);//  0x20 calculate from the datasheet= 500*(1+2*down3_bypass)/(2^ndec*RB*(1+enmanch)) 
  _spi_write(0x21, 0x01); // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0xD7); // 0x22    ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0xDC); // 0x23 
  _spi_write(0x24, 0x03); // 0x24 
  _spi_write(0x25, 0xB8); // 0x25 
  _spi_write(0x2a, 0x1e); 
  
  _spi_write(0x6e, 0x0E); //case RATE_57.6K 
  _spi_write(0x6f, 0xBF); //case RATE_57.6K 
  

  _spi_write(0x30, 0x8c);    // enable packet handler, msb first, enable crc, 

  _spi_write(0x32, 0xf3);    // 0x32address enable for headere byte 0, 1,2,3, receive header check for byte 0, 1,2,3 
  _spi_write(0x33, 0x42);    // header 3, 2, 1,0 used for head length, fixed packet length, synchronize word length 3, 2, 
  _spi_write(0x34, 0x07);    // 7 default value or   // 64 nibble = 32byte preamble 
  _spi_write(0x36, 0x2d);    // synchronize word 
 _spi_write(0x37, 0xd4); 
 _spi_write(0x38, 0x00); 
 _spi_write(0x39, 0x00); 
 _spi_write(0x3a, RF_Header[0]);    // tx header 
 _spi_write(0x3b, RF_Header[1]); 
 _spi_write(0x3c, RF_Header[2]); 
 _spi_write(0x3d, RF_Header[3]); 
 _spi_write(0x3e, 17);    // total tx 17 byte 
 
  
  
    //RX HEADER
 _spi_write(0x3f, RF_Header[0]);   // check hearder 
 _spi_write(0x40, RF_Header[1]); 
 _spi_write(0x41, RF_Header[2]); 
 _spi_write(0x42, RF_Header[3]); 
 _spi_write(0x43, 0xff);    // all the bit to be checked 
 _spi_write(0x44, 0xff);    // all the bit to be checked 
 _spi_write(0x45, 0xff);    // all the bit to be checked 
 _spi_write(0x46, 0xff);    // all the bit to be checked 
  

  
  _spi_write(0x6d, 0x07); // 7 set power max power 
  _spi_write(0x79, 0x00);    // no hopping 
  _spi_write(0x7a, 0x06);    // 60khz step size (10khz x value) // no hopping 

  _spi_write(0x71, 0x23); // Gfsk, fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 
  //_spi_write(0x72, 0x1F); // frequency deviation setting to 19.6khz (for 38.4kbps)
  _spi_write(0x72, 0x2E); // frequency deviation setting to 28.8khz(for 57.6kbps)
  _spi_write(0x73, 0x00);   
  _spi_write(0x74, 0x00);    // no offset 
 

  //band 435.000
 
 _spi_write(0x75, 0x53);    
 _spi_write(0x76, 0x7D);    
 _spi_write(0x77, 0x00); 
}

void fast_init(void){

ItStatus1 = _spi_read(0x03); // read status, clear interrupt   
 ItStatus2 = _spi_read(0x04); 
  _spi_write(0x06, 0x00);    // no wakeup up, lbd, 
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode 
  _spi_write(0x09, 0x7f);  // c = 12.5p   
  _spi_write(0x0a, 0x05); 
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio RX State 
  //  -- Old PCB --
  //  _spi_write(0x0b, 0x15);    // gpio0 RX State
  //  _spi_write(0x0c, 0x12);    // gpio TX State 
  _spi_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output 
  _spi_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION. 
  
  


  _spi_write(0x70, 0x00);    // disable manchest 
  
  _spi_write(0x6e, 0x09); //case RATE_384K: // 38.4k 
  _spi_write(0x6f, 0xD5); //case RATE_384K: // 38.4k 


  _spi_write(0x1c, 0x02); // RATE_24K: // 2.4k
  _spi_write(0x20, 0x68);//0x20 calculate from the datasheet= 500*(1+2*down3_bypass)/(2^ndec*RB*(1+enmanch)) 
  _spi_write(0x21, 0x01); // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0x3A); // 0x22    ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0x93); // 0x23 
  _spi_write(0x24, 0x02); // 0x24 
  _spi_write(0x25, 0x6B); // 0x25 
  _spi_write(0x2a, 0x1e); 



}



//-------------------------------------------------------------- 
void send_read_address(unsigned char i) 
{ 
 i &= 0x7f; 
  
 Write8bitcommand(i); 
}  
//-------------------------------------------------------------- 
void send_8bit_data(unsigned char i) 
{ 
  unsigned char n = 8; 
  SCK_off;
    while(n--) 
    { 
         if(i&0x80) 
          Write1(); 
         else 
          Write0();    
         i = i << 1; 
    } 
   SCK_off;
}  
//-------------------------------------------------------------- 

unsigned char read_8bit_data(void) 
{ 
  unsigned char Result, i; 
  
 SCK_off;
 Result=0; 
    for(i=0;i<8;i++) 
    {                    //read fifo data byte 
       Result=Result<<1; 
       SCK_on;
       NOP(); 
       if(SDO_1) 
       { 
         Result|=1; 
       } 
       SCK_off;
       NOP(); 
     } 
    return(Result); 
}  
//-------------------------------------------------------------- 

//----------------------------------------------------------------------- 
void rx_reset(void) 
{ 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
  _spi_write(0x7e, 36);    // threshold for rx almost full, interrupt when 1 byte received 
  _spi_write(0x08, 0x03);    //clear fifo disable multi packet 
  _spi_write(0x08, 0x00);    // clear fifo, disable multi packet 
  _spi_write(0x07,RF22B_PWRSTATE_RX );  // to rx mode 
  _spi_write(0x05, RF22B_Rx_packet_received_interrupt); 
  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);  
}  
//-----------------------------------------------------------------------    

void to_rx_mode(void) 
{  
 to_ready_mode(); 
 delay(50); 
 rx_reset(); 
 NOP(); 
}  

//-------------------------------------------------------------- 
void to_tx_mode(void) 
{ 
 
 unsigned char i;
 to_ready_mode();
 
  _spi_write(0x08, 0x03);    // disABLE AUTO TX MODE, enable multi packet clear fifo 
  _spi_write(0x08, 0x00);    // disABLE AUTO TX MODE, enable multi packet, clear fifo 
  
    // ph +fifo mode 
  _spi_write(0x34, 7);     // 64 nibble = 32byte preamble 
  _spi_write(0x3e, 17);    // total tx 17 byte 

 for (i = 0; i<17; i++)
 { 
  _spi_write(0x7f, RF_Tx_Buffer[i]); 
 } 
 
 
 _spi_write(0x05, RF22B_PACKET_SENT_INTERRUPT);  
 ItStatus1 = _spi_read(0x03);      //read the Interrupt Status1 register 
 ItStatus2 = _spi_read(0x04); 
  _spi_write(0x07, RF22B_PWRSTATE_TX);    // to tx mode 

  while(nIRQ_1);

  to_ready_mode();

}  
//-------------------------------------------------------------- 
void to_ready_mode(void) 
{ 
 ItStatus1 = _spi_read(0x03);   
 ItStatus2 = _spi_read(0x04); 
 _spi_write(0x07, RF22B_PWRSTATE_READY); 
}  
//-------------------------------------------------------------- 
void to_sleep_mode(void) 
{ 
  _spi_write(0x07, RF22B_PWRSTATE_READY);  
   
  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);    
  _spi_write(0x07, RF22B_PWRSTATE_POWERDOWN); 

} 
//--------------------------------------------------------------   
  
void frequency_configurator(long frequency){

  // frequency formulation from Si4432 chip's datasheet
  // original formulation is working with mHz values and floating numbers, I replaced them with kHz values.
  frequency = frequency / 10;
  frequency = frequency - 24000;
  frequency = frequency - 19000; // 19 for 430–439.9 MHz band from datasheet
  frequency = frequency * 64; // this is the Nominal Carrier Frequency (fc) value for register setting
  
  byte byte0 = (byte) frequency;
  byte byte1 = (byte) (frequency >> 8);
  
  _spi_write(0x76, byte1);    
  _spi_write(0x77, byte0); 

}
 
