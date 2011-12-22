// **********************************************************
// **               OpenLRS Wii Functions                  **
// **          Developed by Etienne Saint-Paul             **
// **          This Source code licensed under GPL         **
// **********************************************************
// Latest Code Update : 2011-06-19
// Supported Hardware : OpenLRS Rx board (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/

//################# Wii Motion Plus FUNCTIONS #################
// Based from http://randomhacksofboredom.blogspot.com
// See also http://wiibrew.org/wiki/Wiimote/Extension_Controllers

#ifdef WiiMotionPlus

#define Bset(data,val) data|=(val)
#define Bclr(data,val) data&=~(val)
#define Btest(data,val) ((data&(val))==(val))
#define Bchg(data,val) if (Btest(data,val)) Bclr(data,val); else Bset(data,val)
#define Bmov(data,data1,val) if (Btest(data,val)) Bset(data1,val); else Bclr(data1,val)



#define data_is_from()          (raw_data[5]&3)
#define data_is_from_wmp()      (data_is_from()==2)
#define data_is_from_nunchuck() (data_is_from()==0)
#define wmp_yaw()               (((raw_data[3]>>2)<<8)+raw_data[0])
#define wmp_pitch()             (((raw_data[4]>>2)<<8)+raw_data[1])
#define wmp_roll()              (((raw_data[5]>>2)<<8)+raw_data[2])

unsigned int wmp_get_corrected_val (unsigned int val,signed char is_slow)
{
  if (is_slow)
    return(val);
  unsigned long v = (val*2000)/440;
  return((unsigned int)val);
}

#define wmp_get_yaw()    (wmp_get_corrected_val(wmp_yaw(),Btest(raw_data[3],2)))
#define wmp_get_pitch()  (wmp_get_corrected_val(wmp_pitch(),Btest(raw_data[3],1)))
#define wmp_get_roll()   (wmp_get_corrected_val(wmp_roll(),Btest(raw_data[4],2)))

#define nck_get_accx()   ((raw_data[5]>>4)&1)+(raw_data[2]<<1)
#define nck_get_accy()   ((raw_data[5]>>5)&1)+(raw_data[3]<<1)
#define nck_get_accz()   (raw_data[5]>>6)+((raw_data[4]&0xfe)<<1)

unsigned int AccX_Offset,AccY_Offset,AccZ_Offset;

//--------------------------------------------------------------------------------

void i2c_write (unsigned char slave_adr,unsigned char reg,unsigned char val)
{
  Wire.beginTransmission(slave_adr);
  Wire.send(reg);
  Wire.send(val);
  Wire.endTransmission();
}

//--------------------------------------------------------------------------------

void InitNunchuck ()
{
  i2c_write(0x52,0xf0,0x55);
  i2c_write(0x52,0xfb,0);
}

//--------------------------------------------------------------------------------

signed char wmp_IsOn ()
{
  unsigned char test_data[6]={0x0,0x0,0xA6,0x20,0x0,0x05};
  signed char detected=true;
  Wire.requestFrom(0x53,6);
  for (int i=0;i<6;i++)
  {
    raw_data[i]=Wire.receive();
    if (raw_data[i]!=test_data[i])
      detected=false;
    Serial.print("0x");
    Serial.print(raw_data[i],HEX);
    Serial.print(",");
  }
  Serial.println("");
  return(!detected);            // Once Wm+ is activated, is doesn't give it's ID when reading at 0x53
}

void wmp_On ()
{                                // WM+ starts out deactivated at address 0x53
  i2c_write(0x53,0xf0,0x55);
  delay(250);
  i2c_write(0x53,0xfe,0x05);     // send 0x05 to address 0xFE to activate WM+ in pass-through mode for nunchuck, WM+ jumps to address 0x52 and is now active
  delay(250);
//  wmp_IsOn ();
}

void wmp_SendZero()
{
  Wire.beginTransmission(0x52);  // now at address 0x52
  Wire.send(0x00);               // send zero to signal we want info
  Wire.endTransmission();
}

void wmp_read_raw_data ()
{
    wmp_SendZero();
    Wire.requestFrom(0x52,6);
    for (int i=0;i<6;i++)
      raw_data[i]=Wire.receive();
}

//--------------------------------------------------------------------------------

void wmp_calibrateZeroes()
{
  for (int i=0;i<5;i++)     // 5 fake reading for balancing the system
  { 
    wmp_read_raw_data();
    delay(100);
  }

  Gyro_YAW_Zero=0;
  Gyro_PITCH_Zero=0;
  Gyro_ROLL_Zero=0;
  AccX_Offset=0;
  AccY_Offset=0;
  AccZ_Offset=0;

  unsigned long nbavg=0;
  unsigned long nbava=0;
  unsigned long y=0;
  unsigned long p=0;
  unsigned long r=0;
  unsigned long ax=0;
  unsigned long ay=0;
  unsigned long az=0;
  for (int i=0;i<20;i++)   // 10 reading for calculating average value
  { 
    wmp_read_raw_data();
    if (data_is_from_wmp())
    {
      y +=wmp_get_yaw();
      p +=wmp_get_pitch();
      r +=wmp_get_roll();
      nbavg++;
    }
    if (data_is_from_nunchuck())
    {
      ax+=nck_get_accx();
      ay+=nck_get_accy();
      az+=nck_get_accz();
      nbava++;
    }
    delay(4);              // In pass-through mode, you need to wait at least 3ms before reading again
  }
  if (nbavg>0)
  {
    Gyro_YAW_Zero=y/nbavg;
    Gyro_PITCH_Zero=p/nbavg;
    Gyro_ROLL_Zero=r/nbavg;
  }
  if (nbava>0)
  {
    AccX_Offset=ax/nbava;
    AccY_Offset=ay/nbava;
    AccZ_Offset=az/nbava;
  }
  if ((y==0)&&(p==0)&&(r==0)&&((ax+ay+az)==0))
    wmp_On();
}

void wmp_receiveData()
{
  wmp_read_raw_data();

  if (data_is_from_wmp())
  {
    Gyro_YAW=wmp_get_yaw()-Gyro_YAW_Zero; 
    Gyro_PITCH=wmp_get_pitch()-Gyro_PITCH_Zero; 
    Gyro_ROLL=wmp_get_roll()-Gyro_ROLL_Zero;
        
     #if (DEBUG_MODE == 6)
            Serial.print("yaw:");//see diagram on randomhacksofboredom.blogspot.com
            Serial.print(Gyro_YAW); //for info on which axis is which
            Serial.print(" pitch:");
            Serial.print(Gyro_PITCH);
            Serial.print(" roll:");
            Serial.println(Gyro_ROLL);
    #endif
    return;
  }
  if (data_is_from_nunchuck())
  {
    JoyX=raw_data[0];
    JoyY=raw_data[1];
    AccX=nck_get_accx()-AccX_Offset;
    AccY=nck_get_accy()-AccY_Offset;
    AccZ=nck_get_accz()-AccZ_Offset;
#if (DEBUG_MODE==6)
    Serial.print(AccX);Serial.print("\t");
    Serial.print(AccY);Serial.print("\t");
    Serial.print(AccZ);Serial.println(" ");
#endif
  }
}
#endif



