#include "ASM330LHH.h"
#include <RTC.h>
#include "I2Cdev.h"

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 13

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t delt_t = 0;                      // used to control display output rate
uint32_t sumCount = 0;                    // used to control display output rate
float pitch, yaw, roll;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval


//ASM330LHH definitions
#define ASM330LHH_intPin1 12  // interrupt1 pin definitions, significant motion
#define ASM330LHH_intPin2 4   // interrupt2 pin definitions, significant motion

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_12_5Hz, AODR_26Hz, AODR_52Hz, AODR_104Hz, AODR_208Hz, AODR_417Hz, AODR_833Hz, AODR_1667Hz, AODR_3333Hz, AODR_6667Hz
      GODR_12_5Hz, GODR_26Hz, GODR_52Hz, GODR_104Hz, GODR_208Hz, GODR_417Hz, GODR_833Hz, GODR_1667Hz, GODR_3333Hz, GODR_6667Hz
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_208Hz, GODR = GODR_417Hz;

float aRes, gRes;              // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0}; // offset biases for the accel and gyro
int16_t ASM330LHHData[7], ASM330LHHXLData[3], ASM330LHHGData[4];        // Stores the 16-bit signed sensor output
float   Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 

bool newASM330LHHXLData = false;
bool newASM330LHHGData  = false;

ASM330LHH ASM330LHH(&i2c_0); // instantiate LSM6DSM class


uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
bool alarmFlag = false; // for RTC alarm interrupt


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led off

  // Configure interrupt pins
  pinMode(ASM330LHH_intPin1, INPUT);
  pinMode(ASM330LHH_intPin2, INPUT);
 
  Wire.begin(); // designate I2C pins for master mode 
  Wire.setClock(400000);      // I2C frequency at 400 kHz  
  delay(1000);
 
  i2c_0.I2Cscan();

  // Read the ASM330LHH Chip ID register, this is a good test of communication
  Serial.println("ASM330LHH accel/gyro...");
  byte c = ASM330LHH.getChipID();  // Read CHIP_ID register for ASM330LHH
  Serial.print("ASM330LHH "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x6B, HEX);
  Serial.println(" ");
  delay(1000); 

  if(c == 0x6B) // check if ASM330LHH has acknowledged
  {
   Serial.println("ASM330LHH online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW);

   // get sensor resolutions, only need to do this once
   aRes = ASM330LHH.getAres(Ascale);
   gRes = ASM330LHH.getGres(Gscale);
   
   ASM330LHH.selfTest();

   ASM330LHH.reset();  // software reset ASM330LHH to default registers

   ASM330LHH.init(Ascale, Gscale, AODR, GODR);

   ASM330LHH.offsetBias(gyroBias, accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   delay(1000); 

   digitalWrite(myLed, HIGH);
   
  }
  else 
  {
  if(c != 0x6B) Serial.println(" ASM330LHH not functioning!"); 
  while(1){};
  }

  // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(ASM330LHH_intPin1, myinthandler1, RISING);  // define interrupt for intPin1 output of ASM330LHH
  attachInterrupt(ASM330LHH_intPin2, myinthandler2, RISING);  // define interrupt for intPin2 output of ASM330LHH
}

/* End of setup */

void loop() {

   // If intPin1 goes high, XL data registers have new data
   if(newASM330LHHXLData == true) {   // On interrupt, read data
      newASM330LHHXLData = false;     // reset newData flag

     ASM330LHH.readXLData(ASM330LHHXLData); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)ASM330LHHXLData[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)ASM330LHHXLData[1]*aRes - accelBias[1];   
     az = (float)ASM330LHHXLData[2]*aRes - accelBias[2]; 
   }
    
   // If intPin2 goes high, Gyro data registers have new data
   if(newASM330LHHGData == true) {   // On interrupt, read data
      newASM330LHHGData = false;     // reset newData flag

     ASM330LHH.readGData(ASM330LHHGData); // INT2 cleared on any read

   // Calculate the gyro value into actual degrees per second
     gx = (float)ASM330LHHGData[1]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)ASM330LHHGData[2]*gRes - gyroBias[1];  
     gz = (float)ASM330LHHGData[3]*gRes - gyroBias[2];     
   }
   // end sensor interrupt handling


    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved and the MPU9250 is awake
       alarmFlag = false;

    // Read RTC
   if(SerialDebug)
    {
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }
    
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    }

    Gtemperature = ((float) ASM330LHHGData[0]) / 256.0f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW);   
    }
}

/*  End of main loop */


void myinthandler1()
{
  newASM330LHHXLData = true;
}


void myinthandler2()
{
  newASM330LHHGData = true;
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
  

