// ***********************************************************
// ***          OpenLRS Rx Configuration file               **
// **       This Source code licensed under GPL             **
// ***********************************************************
// Version Number     : 1.10
// Latest Code Update : 2011-10-04
// Supported Hardware : OpenLRS Tx boards (M1 & M2) (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************

//####### TX BOARD TYPE #######
//We have 2 option for OpenLRS TX, You can use original TX modules or just load the Tx firmware into the RX
// 0 = Original M1 Tx Board
// 1 = OpenLRS Rx Board works as TX, reads your PPM signals from first servo port.
// 2 = Original M2 Tx Board
#define TX_BOARD_TYPE 2

//######### DEBUG MODES ##########
// 0 = No Debug Output
// 1 = PPM signal analyzer
// 5 = Hopping Channel number from "Hopping" function
#define DEBUG_MODE 0

//######### TRANSMISSION VARIABLES ##########
#define CARRIER_FREQUENCY 435000  // 435Mhz startup frequency
#define FREQUENCY_HOPPING 1 // 1 = Enabled  0 = Disabled

//###### HOPPING CHANNELS #######
//Select the hopping channels between 0-255
// Default values are 13,54 and 23 for all transmitters and receivers, you should change it before your first flight for safety.
//Frequency = CARRIER_FREQUENCY + (StepSize(60khz)* Channel_Number) 
static unsigned char hop_list[3] = {13,54,23};



//###### RF DEVICE ID HEADERS #######
// Change this 4 byte values for isolating your transmission, RF module accepts only data with same header
static unsigned char RF_Header[4] = {'O','L','R','S'};  

//###### SERIAL PORT SPEED #######
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed


//####### CONTROL TYPE #######
//We have 2 control option for Tx system. 
//First method is standard PPM based control with any PPM generator like Futaba,Hitec,HK,Jr transmitters
//Second method is TTL level Serial control with a PC or any microcontroller.
//Serial control command is ; 'S'+ chr(channel number) + chr(position / 256) + chr(position % 256) 
// 0 = PPM
// 1 = Serial
#define CONTROL_TYPE 0


//###### TELEMETRY MODES ########
#define TELEMETRY_ENABLED 0  // 1 = Enabled(bidirectional mode)  0 = Disabled(unidirectional mode)
#define TELEMETRY_MODE 0 // 0 = Transparent Bridge(750 byte/second max) // 1 = Standard OpenLRS Telemetry
#define TELEMETRY_OUTPUT_ENABLED 1 // 1 = Enabled  0 = Disabled  //Enables the Serial Telemetry Data Output. If you need only Buzzer alerts, disable it for less processing time.

//###### RANGE ALERTS #######
// Alerts only works when the telemetry system active
// If you using the 7W booster, disable the alert modes and telemetry because our 7W booster is unidirectional
// 
#define Rx_RSSI_Alert_Level 0  // 40 is the package lost limit, 0 for disabling
#define Tx_RSSI_Alert_Level 0  // 40 is the package lost limit, 0 for disabling
#define Lost_Package_Alert 2 // 0 = Disabled, 1=Alert with each lost pack, 2=Alert with 2 or more lost package(suggested value) 




//############ VARIABLES ########################


unsigned char RF_Rx_Buffer[17];
unsigned char RF_Tx_Buffer[17]; 
unsigned char Telemetry_Buffer[8];


volatile unsigned char Servo_Buffer[30];	//servo positions

volatile unsigned char channel_no=0;
volatile unsigned int transmitted=1;
volatile unsigned char channel_count=0;

static unsigned char hopping_channel = 1;
unsigned long time,old_time; //system timer

unsigned char Rx_Pack_Received = 0;
unsigned char Rx_RSSI = 110;
unsigned char Tx_RSSI = 110;


//####### Board Pinouts #########

#if (TX_BOARD_TYPE == 0)
    #define PPM_IN A5
    #define RF_OUT_INDICATOR A4
    #define BUZZER 9
    #define BTN 10
    #define Red_LED 12
    #define Green_LED 11
    
    #define Red_LED_ON  PORTB |= _BV(4);
    #define Red_LED_OFF  PORTB &= ~_BV(4);
    
    #define Green_LED_ON   PORTB |= _BV(3);
    #define Green_LED_OFF  PORTB &= ~_BV(3);
    
    #define PPM_Pin_Interrupt_Setup  PCMSK1 = 0x20;PCICR|=(1<<PCIE1);
    #define PPM_Signal_Interrupt PCINT1_vect
    #define PPM_Signal_Edge_Check (PINC & 0x20)==0x20
    
#endif
    
#if (TX_BOARD_TYPE == 1)
    #define PPM_IN 5
    #define RF_OUT_INDICATOR 6
    #define BUZZER 7
    #define BTN 8
    
    #define Red_LED A3
    #define Green_LED A2
    
    #define Red_LED_ON  PORTC &= ~_BV(2);PORTC |= _BV(3);
    #define Red_LED_OFF  PORTC &= ~_BV(2);PORTC &= ~_BV(3);

    #define Green_LED_ON  PORTC &= ~_BV(3);PORTC |= _BV(2);
    #define Green_LED_OFF  PORTC &= ~_BV(3);PORTC &= ~_BV(2);
    
    #define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x20;PCICR|=(1<<PCIE2);
    #define PPM_Signal_Interrupt PCINT2_vect
    #define PPM_Signal_Edge_Check (PIND & 0x20)==0x20
    
#endif  

#if (TX_BOARD_TYPE == 2)
    #define PPM_IN 3
    #define RF_OUT_INDICATOR A0
    #define BUZZER 10
    #define BTN 11
    #define Red_LED 13
    #define Green_LED 12
    
    #define Red_LED_ON  PORTB |= _BV(5);
    #define Red_LED_OFF  PORTB &= ~_BV(5);
    
    #define Green_LED_ON   PORTB |= _BV(4);
    #define Green_LED_OFF  PORTB &= ~_BV(4);
    
    #define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
    #define PPM_Signal_Interrupt PCINT2_vect
    #define PPM_Signal_Edge_Check (PIND & 0x08)==0x08
    
#endif


//####### RFM22B Pinouts #########

#if ((TX_BOARD_TYPE == 0)||(TX_BOARD_TYPE == 1))
      //## RFM22B Pinouts for Public Edition (M1 or Rx v1)
      #define  nIRQ_1 (PIND & 0x08)==0x08 //D3
      #define  nIRQ_0 (PIND & 0x08)==0x00 //D3
      
      #define  nSEL_on PORTD |= (1<<4) //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTD |= (1<<2) //D2
      #define  SCK_off PORTD &= 0xFB //D2
      
      #define  SDI_on PORTC |= (1<<1) //C1
      #define  SDI_off PORTC &= 0xFD //C1
      
      #define  SDO_1 (PINC & 0x01) == 0x01 //C0
      #define  SDO_0 (PINC & 0x01) == 0x00 //C0
      
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin 2 
      #define IRQ_pin 3
      #define nSel_pin 4
#endif      

#if (TX_BOARD_TYPE == 2)
      //## RFM22B Pinouts for Public Edition (M2 or Rx v2)
      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= (1<<4) //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTD |= (1<<7) //D7
      #define  SCK_off PORTD &= 0x7F //D7
      
      #define  SDI_on PORTB |= (1<<0) //B2
      #define  SDI_off PORTB &= 0xFE //B2
      
      #define  SDO_1 (PINB & 0x02) == 0x02 //B3
      #define  SDO_0 (PINB & 0x02) == 0x00 //B3
      
      #define SDO_pin 9
      #define SDI_pin 8        
      #define SCLK_pin 7 
      #define IRQ_pin 2
      #define nSel_pin 4
#endif  

