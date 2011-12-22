// ***********************************************************
// ***          OpenLRS Sensor Functions file               **
// **        Developed by Melih Karakelle on 2010-2011      **
// **       This Source code licensed under GPL             **
// ***********************************************************
// Latest Code Update : 2011-09-26
// Supported Hardware : OpenLRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/

//############# ACCELEROMETER FUNCTIONS #################

//function to write byte data into a register
void MMA7455_Init() {
    Wire.beginTransmission(0x1D);
    Wire.send(0x16);
    Wire.send(0x25);
    Wire.endTransmission();

}

void MMA7455_Calibrate(){

 int tmpx = 0;
 int tmpy = 0;
 int tmpz = 0; 

 acc_offx = 0;
 acc_offy = 0;
 acc_offz = 0;
 
 for (char i = 0;i<20;i++)
    {
    delay(10);  
    MMA7455_Read();
    tmpx += AccX;
    tmpy += AccY;
    tmpz += AccZ; 
    }  
 acc_offx = tmpx/20;
 acc_offy = tmpy/20;
 acc_offz = (tmpz/20) -64;
 
}

void MMA7455_Read(){  
    byte buff[8];
    
    Wire.beginTransmission(0x1D);
    Wire.send(0x06); //read from here
    Wire.endTransmission();
    Wire.requestFrom(0x1D, 3);
    
    int i = 0;
    
    while(Wire.available())    
        { 
        buff[i] = Wire.receive(); 
        i++;
        }
    Wire.endTransmission();
   
    
    AccX = buff[0];
    if (AccX>127) AccX = -(256 - AccX);
    AccY = buff[1];
    if (AccY>127) AccY = -(256 - AccY);
    AccZ = buff[2];
    if (AccZ>127) AccZ = -(256 - AccZ);
    
     AccX -= acc_offx;
     AccY -= acc_offy;
     AccZ -= acc_offz;
     
    
    #if (DEBUG_MODE == 7) 
          Serial.print("X: ");
          Serial.print(AccX);
          Serial.print(" Y: ");
          Serial.print(AccY);
          Serial.print(" Z: ");
          Serial.println(AccZ);
    #endif 
    
}




//======== HMC5883L Magnetometer ============

void initHMC5883L()
{
   delay(5);
   Wire.beginTransmission(0x1E);// 7 bits address 
   Wire.send(0x02);
   Wire.send(0x00); // continues reading
   Wire.endTransmission();
}

void HMC5883L_Read()
{
   Wire.beginTransmission(0x1E);
   Wire.send(0x03);
   Wire.endTransmission();
   delay(5);
   Wire.requestFrom(0x1E, 6);
  
    int i = 0;
    byte buff[6];
    while(Wire.available())    
        { 
        buff[i] = Wire.receive(); 
        i++;
        }
    Wire.endTransmission();
   
       
       MagX = buff[0] << 8;
       MagX |= buff[1];
       MagZ = buff[2] << 8;
       MagZ |= buff[3];
       MagY = buff[4] << 8;
       MagY |= buff[5];
       
       
       
     #if (DEBUG_MODE == 8)  
        Serial.print("X: ");
        Serial.print(MagX);
        Serial.print(" Y: ");
        Serial.print(MagY);
        Serial.print(" Z: ");
        Serial.println(MagZ);
     #endif    
       
}

//======== GPS ============
// based from http://www.arduino.cc/playground/Tutorials/GPS
#if defined(GPS)
 int byteGPS=-1;
 char linea[100] = "";
 char line_gprmc[100] = "";
 char line_gpgga[100] = "";
 int cont=0;
 int bien=0;
 int conta=0;
 int indices[13];
 
void GPS_Init(void)
{
 for (int i=0;i<300;i++){       // Initialize a buffer for received data
           linea[i]=' ';
         }   
}  


void GPS_read(void)
{
 byteGPS=Serial.read();         // Read a byte of the serial port
   
     linea[conta]=byteGPS;        // If there is serial port data, it is put in the buffer
     conta++; 
     if (conta>99) conta = 99; // buffer limitter
     
     //Serial.print(byteGPS, BYTE); 
     if (byteGPS==13){            // If the received byte is = to 13, end of transmission
       cont=0;
       bien=0;
       if (linea[1]=='$')
           {
           if ((linea[5]=='M') && (linea[6]=='C')) //$GPRMC
               {     
               strncpy(line_gprmc,linea,100);
               GPS_data_status = 1;//GPS_data ready!     
               } 
           if ((linea[5]=='G') && (linea[6]=='A')) //$GPGGA
               {                 
               for (int i=0;i<100;i++)
                   {
                   strncpy(line_gpgga,linea,100);
                   }     
               }             
           }
           
       Red_LED_ON;
       
       conta=0;                    // Reset the buffer
       //for (int i=0;i<300;i++){    //  
         //linea[i]=' ';             
       //}                 
     }
  
} 

void GPS_data_parser(void)
{
  


/* unsigned char comma_count = 0;
          
 for (int i=0;i<100;i++)
   {   
    if (line_gprmc[i]=='*') break;
    
    if (line_gprmc[i]==','){    
             comma_count++;             
    }else{
      
    switch (comma_count) {
          case 1://Time in UTC (HhMmSs)
            GPS_Time += line_gprmc[i];
            break;
          case 2://Status (A=OK,V=KO)
            GPS_Status += line_gprmc[i];
            break;
          case 3://Latitude
            GPS_Latitude += line_gprmc[i];
            break;  
          case 4://Direction (N/S)
            GPS_Latitude += line_gprmc[i];
            break;  
          case 5://Longitude
            GPS_Longitude += line_gprmc[i];
            break;  
          case 6://Direction (E/W)
            GPS_Longitude += line_gprmc[i];
            break;  
          case 7://Velocity in knots
            GPS_Speed += line_gprmc[i];
            break;  
          case 8://Heading in degrees
            GPS_Heading += line_gprmc[i];
            break;  
          case 9://Date UTC (DdMmAa)
            GPS_Date += line_gprmc[i];
            break;   
          case 10:// Magnetic variation degree
            GPS_Mag_Variation  += line_gprmc[i];
            break;   
          case 11://Magnetic variation (E/W)
            GPS_Mag_Variation += line_gprmc[i];
            break;     
          //default: 
            // if nothing else matches, do the default
            // default is optional
          }
        //Serial.print( liner[i]);
      }            
    
     
   }
  */
  
   
   char *last;
   char *token;
   char **item = parts;

   token = strtok_r(line_gprmc, ",", &last);
   while (token) {
        *(item++) = token;
        token = strtok_r(NULL, ",", &last);
   }
   *item = NULL; 
  
  
  /*
  GPS_Time = parts[1];
  GPS_Status = parts[2];
  GPS_Latitude = parts[3] ;
  GPS_Longitude = parts[5] ;
  //GPS_Altitude = "";
  GPS_Speed = parts[7];
  GPS_Heading = parts[8];
  GPS_Date = parts[9];  
  GPS_Mag_Variation = parts[10];
  */
  
  /*
  Serial.print("D:"); 
  Serial.print(GPS_Time);
  Serial.print("  "); 
  Serial.println(GPS_Latitude);
  */
  
  GPS_data_status = 0;
}


#endif


unsigned char SET_Telemetry(String telemetry_data, byte DeviceID, byte SensorID)
{

 unsigned char data_i = 4; //
 unsigned char comma_count = 0;
 unsigned char data_size = 0;
 RF_Tx_Buffer[2] = DeviceID; // Sensor Device ID
 RF_Tx_Buffer[3] = 4 << SensorID; // Value ID  

          
 for (int i=0;i<12;i++)
   {     
    if ((i+4)>16) break; 
    if (telemetry_data[i]==0x00) break;
    RF_Tx_Buffer[i+4] = telemetry_data[i];
    data_size++;
   } 
  RF_Tx_Buffer[3] += data_size;  
}  

unsigned char SET_Telemetry_i(int telemetry_data, byte DeviceID, byte SensorID)
{

 unsigned char data_i = 4; //
 unsigned char comma_count = 0;
 unsigned char data_size = 0;
 RF_Tx_Buffer[2] = DeviceID; // Sensor Device ID
 RF_Tx_Buffer[3] = 4 << SensorID; // Value ID  

          
 RF_Tx_Buffer[4] = (telemetry_data / 256);
 RF_Tx_Buffer[5] = (telemetry_data % 256);



  RF_Tx_Buffer[3] = 2;  
}  


