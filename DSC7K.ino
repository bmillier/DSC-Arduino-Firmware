#include <Wire.h>
#include <math.h>
#include <EEPROM.h>


//        changed C5 to 3.9 to calibrate indium to 156.6   //was 3.85
// removed old ramp to start temp/stabilize routine and integrated it into the main ramp up routine
//  use slower ADC rate of 64 SPS
//  split ramp routine into 2 sections- 1 for sample 1 for reference
// re-coded ramp down and tested 
// added the T command for storing slope correction
// in setting the sample PWM width, multiply normal value by slope value (float value varying around 1.00)
// add code between the ramp up and rap down to stablize
// Dec 24/2015 Added calculations for Vref that is based upon the actual Single-ended voltage read at input A0
//             and which is Vref/ 8.5.  (7.5K and 1K resister divider)
//             Vofs is calculated as Vref * (1.5K/2.5K), which is the resistor ratio used for A3 input
//             all resistors are 0.1% precision 
//             
//   Added protection for open RTD sensor(s)   Did not use ADC comparator - couldn't get it to operate properly
// Dec 26/2015 corrected port pin from 3 to 8 for Peltier cooler drive
// added cmd=0; after all commands
// Dec 31 corrected bug in the slope correction routine. Only multiply the the accumulated Ref error term by slope parameter
// Leave the ref heater getting its calculated amount of heat.
// Slope value sent by PC host is sample heat/reference heat   at the top of an empty scan
// so use this number to adjust the reference accumulator value to match as closely as possible by multiplying ref accum by it 
// Jan 23 2016 - changed slope compensation. Don't compensate every reference value stored to accumulator, just
//  multiply accumator by slope when sending results every 1+ second
// strange bug found when VB sends a slope value <1. With or without a leading zero, value is read by parsefloat as 0
// same thing sent by Arduino serial monitor is OK !
// workaround- add 1.0 to slope at VB end, and subtract 1.0 in T command routine here
// checked out the calibration loads as they use parsefloat
// also added a printout of the slope value as part of the ? command
 
long V1,V2;
long Vref; 
long Vofs;
float RTD1,RTD2;
float sample, reference;
float SampleErrorTerm,ReferenceErrorTerm;
long SampleAccumulator,ReferenceAccumulator;
float C6A,C7A,C8A,C6B,C7B,C8B;     // UP RAMP vars
float C9A,C10A,C11A,C9B,C10B,C11B;  // DOWM RAMP vars
int const SamPerMin =1200;       // set for 20 Hz rate
int const minPWM=2;
int TickCount;
int DegPerMin;
float StartTemp;          // default =50C
float EndTemp;           // default =300C
int AcqMode;               // Baseline collection mode is default
int Pg;                 //default prop gain is 2000
int PgIncr;            //default prop gain increment value, per ramp degree =30 
int PropGain;
float Setpoint, SetpointIncr;
boolean ReadyFlag,AbortFlag,StabilityFlag;
float CurrentSampleDeviation,CurrentReferenceDeviation, SampleIntegral,ReferenceIntegral;
float LastSampleIntegral,LastReferenceIntegral;
float SampleIntIncr,ReferenceIntIncr;
int SamNum;
int SetpointInteger;
int SamPerDeg;
unsigned int Temp2,Temp3,Temp6,Temp7;
byte Temp;
float Temp5;
int Dev1,Dev2;
float RTDSetpoint;
int HeatSinkTemperature;
float MaxPWM;
int StabilityCount;
float St1,St2;
long BTemp;
float C5;
byte cmd=0;
float Slope;
byte SlopeSet;
float ft;

const float MinPWM = 2.0;

// SAMPLE default heater power equation coefficients
// Baseline 2nd order curve fit constants UP RAMP 
const float K6A = -408.15;    //constant term
const float K7A =0.03609;    // 2nd order term
const float K8A =34.196;     // 1st order term
// Baseline 2nd order curve fit constants DOWN RAMP
const float K9A= -1746;
const float K10A = 0.055;
const float K11A = 29.44;
// REFERENCE default heater power equation coefficients
// Baseline 2nd order curve fit constants UP RAMP
const float K6B = -413.62;
const float K7B = .0336;
const float K8B = 34.38;
//Baseline 2nd order curve fit constants DOWN RAMP
const float K9B = -1763;
const float K10B = 0.0526;
const float K11B = 29.688;

void setup(void)
{
  Wire.begin();
  Serial.begin(57600);
  Serial.println("DSC7 Unit 1");
  GetVoltageReference();    // measure actual value of LM4040C41IDBZR  regulator, and determine A3 offset voltage as well
  pinMode(8,OUTPUT);    //  Peltier Cooler drive Active High 
  // retrieve the slope variable from EEPROM- set it to 1.00 if EEPROM not written to already
   int eeAddress = 50;
  EEPROM.get(eeAddress,SlopeSet);
  if (SlopeSet == 0xFF) {
    Slope= 1.0;
    SlopeSet=1;
    EEPROM.put(eeAddress,SlopeSet);
    EEPROM.put(eeAddress+1,Slope);
  } 
  else {
    EEPROM.get(eeAddress+1,Slope); 
  }
  //Serial.print(Slope);
  LoadDefaults();
  InitPWMs();
  HeaterOff();
  }

void loop(void)
{
if (Serial.available() >0){
   cmd=Serial.read();
  Serial.write(cmd);
 }
 switch (cmd) {
   case '?':
    Serial.println("DSC 7 unit 1");
    Serial.print("Slope= ");
    Serial.println(Slope);
    cmd=0;
    break;
    case 'R':
    SetDegreesPerMinute();
    cmd=0;
    break;
    case 'L':
    SetStartTemp();
    cmd=0;
    break;
    case 'U':
    SetEndTemp();
    cmd=0;
    break;
    case 'M':
    SetAcqMode();
    cmd=0;
    break;
    case 'P':
    SetPg();
    cmd=0;
    break;
    case 'S':
    SetPgIncr();
    cmd=0;
    break;
   case 'Z':
    ReadSensors(); 
    cmd=0;
    break;
    case 'C':
    SetCooler();
    cmd=0;
    break;
    case 'A':
    DownloadParamsUp();
    cmd=0;
    break;
    case 'B':
    DownloadParamsDown();
    cmd=0;
    break;
    case 'D':
    PrintParameters();
    cmd=0;
    break;
   case 'G':
    PerformRun();
    cmd=0;
    break;  
   case 'J':
    SetSampleHeater();
    cmd=0;
    break;   
   case 'K':
    SetReferenceHeater();
    cmd=0;
    break; 
   case 'H':
    HeatSinkTemp();
    cmd=0;
    break;  
   case 'E':
    RetrieveNvParameters();
    cmd=0;
    break; 
    case 'F':
    StoreNvParameters();
    cmd=0;
    break;
    case 'T':
    SetSlope();
    cmd=0;
    break;
    default:
    cmd=0;
    break;
 }
}


void PerformRun(){
  SampleAccumulator=0;
  ReferenceAccumulator=0;
  SamPerDeg=(float)SamPerMin/(float) DegPerMin;
  SetpointIncr= (float)1.0/(float) SamPerDeg;
  Setpoint= 25.0;     //StartTemp;
  SampleIntIncr=0;     //' no integral increment needed until at starting temp 
  ReferenceIntIncr=0;
  TickCount=0;
  PropGain=Pg;
  TickCount=0;
  Temp=0;   // set phase at 0 (during ramp to starting temperature
  AbortFlag=false;

CalcIntegralUp();    // from this point on use curvefit data for integral term

/********************************************************************************
/  do Ramp up                                                                   * 
/********************************************************************************
*/

do {
  if (AbortFlag == true) {
    break;
   }
 
  TickCount++;
  if (TickCount == SamPerDeg) {
    Setpoint+=0.25;
    int z=(int)Setpoint;
    Setpoint= (float) z;
    if (Setpoint == StartTemp) {
      Temp=2;   //Set Phase =2
    }
    PropGain=PropGain + PgIncr;
    SendPhase();
  }
  Setpoint=Setpoint+SetpointIncr;            // ramp setpoint up
  Macro1S();    // get Sample temp
  Macro3S();
  SampleIntegral=SampleIntegral+SampleIntIncr;
  Macro2S();
  
  Macro1R();    // get Reference temp
  Macro3R();
  ReferenceIntegral=ReferenceIntegral +ReferenceIntIncr;
  Macro2R(); 
  
  if (TickCount >=SamPerDeg) {
    TickCount=0;
    MaxPWM=24995;
    if (Setpoint < 249) {
     MaxPWM=100 * Setpoint;
    }
   GetHeatSinkTemperature();
   SendResults();
   CalcIntegralUp();
  SampleAccumulator=0;
  ReferenceAccumulator=0;
  }
 // Check for an abort command rec'd
 if (Serial.available() >0)  {
      byte cmd;
      cmd=Serial.read();
      if (cmd == 'Q') {
        AbortFlag=true;
       HeaterOff();
       break;
      }
  }   
 } while (EndTemp >= Setpoint);
 
 if (AcqMode == 2) {
 
   //************************************************************************  
 //* now stabilize at upper temperature for 20 seconds                    *
 //************************************************************************
TickCount=0;
do {
  if (AbortFlag == true) {
    break;
   }
   TickCount++;
   Macro1S();    // get Sample temp
   Macro3S();
   Macro2S();
  
  Macro1R();    // get Reference temp
  Macro3R();
  Macro2R(); 
  GetHeatSinkTemperature();
 //  SendResults();
 //  CalcIntegralUp();
  SampleAccumulator=0;
  ReferenceAccumulator=0;
   // Check for an abort command rec'd
 if (Serial.available() >0)  {
      byte cmd;
      cmd=Serial.read();
      if (cmd == 'Q') {
        AbortFlag=true;
       HeaterOff();
       break;
      }
  }   
 } while (TickCount < 400);     // 20 seconds
 
   
   //**********************************************************************************
   //  Ramp Down if called for                                                        *
   //**********************************************************************************
  TickCount=0;
  CalcIntegralDown();
 
  do {
    TickCount++;
    Temp=5;    //set phase to 5
    if (TickCount == SamPerDeg) {
      Setpoint=Setpoint+ 0.25;
      int z = (int) Setpoint;
      Setpoint= (float) z;
      PropGain=PropGain -PgIncr;
      SendPhase();
      }
    Setpoint=Setpoint-SetpointIncr;
    Macro1S();    // get Sample temp
    Macro3S();
    SampleIntegral=SampleIntegral-SampleIntIncr;
    Macro2S();

    Macro1R();    // get Reference temp
    Macro3R();
    ReferenceIntegral=ReferenceIntegral -ReferenceIntIncr;
    Macro2R();     
    if (TickCount >= SamPerDeg) {
      TickCount=0;
      MaxPWM= 24995;
      if (Setpoint < 249){
        MaxPWM = Setpoint * 100;
      }
      GetHeatSinkTemperature();
      SendResults();
      CalcIntegralDown();
      SampleAccumulator=0;
      ReferenceAccumulator=0;
      }
 // Check for an abort command rec'd
     if (Serial.available() >0)  {
      byte cmd;
      cmd=Serial.read();
      if (cmd == 'Q') {
        AbortFlag=true;
       HeaterOff();
       break;
       }
     }         
  } while ( Setpoint >= StartTemp);
 }
 
 if (AbortFlag== true) {
   Serial.println("4 0 0 0");
 }
 else {
   Serial.println("3 0 0 0");
 } 
 HeaterOff();
cmd=0; 
}

void GetHeatSinkTemperature() {
   HeatSinkTemperature= analogRead(0); 
   if (HeatSinkTemperature <407 ) {
    digitalWrite(3,LOW);  // turn Peltier cooler off
    Temp=9;
   }
}   
   
void CalcIntegralUp() {
  SampleIntegral=C6A;
  Temp5 = Setpoint*Setpoint*C7A;
  SampleIntegral=SampleIntegral+Temp5;
  Temp5=Setpoint*C8A;
  SampleIntegral=SampleIntegral+Temp5;
  if (LastSampleIntegral >0 ){                    // derive the amount to increase the integral term at the  update rate  
    SampleIntIncr= (float) (SampleIntegral-LastSampleIntegral) /(float) SamPerDeg;
   }
  LastSampleIntegral=SampleIntegral;
  ReferenceIntegral=C6B;
  Temp5=Setpoint*Setpoint*C7B;
  ReferenceIntegral=ReferenceIntegral + Temp5;
  Temp5=Setpoint*C8B;
  ReferenceIntegral=ReferenceIntegral+Temp5;
  if (LastReferenceIntegral >0 ) {           // derive the amount to increase the integral term at the  update rate  
    ReferenceIntIncr= (float) (ReferenceIntegral-LastReferenceIntegral)/(float) SamPerDeg;
  }  
 LastReferenceIntegral=ReferenceIntegral;
}

void CalcIntegralDown() {
   SampleIntegral = C9A;
   Temp5 = Setpoint * Setpoint * C10A;
   SampleIntegral = SampleIntegral + Temp5;
   Temp5 = Setpoint * C11A;
   SampleIntegral = SampleIntegral + Temp5;
   SampleIntIncr = (float) (SampleIntegral - LastSampleIntegral) / (float) SamPerDeg;  // derive the amount to adjust the integral term at the  update rate
   LastSampleIntegral = SampleIntegral;
   ReferenceIntegral = C9B;
   Temp5 = Setpoint * Setpoint* C10B;
   ReferenceIntegral = ReferenceIntegral + Temp5;
   Temp5 = Setpoint * C11B;
   ReferenceIntegral = ReferenceIntegral + Temp5;
   ReferenceIntIncr = (float) (ReferenceIntegral - LastReferenceIntegral)/ (float) SamPerDeg;  // derive the amount to adjust the integral term at the  update rate
   LastReferenceIntegral = ReferenceIntegral;  
  }
 
void SendPhase() {
  Serial.print(Temp);
  Serial.print(" ");
  Serial.print(Setpoint);
  Serial.print(" ");
}

void SendResults() {
  Serial.print(Dev1);
  Serial.print(" ");
  Serial.print(Dev2);
  Serial.print(" ");
  BTemp= SampleAccumulator/(long) SamPerDeg;
  Temp2= (unsigned int) BTemp;
  Serial.print(Temp2);
  Serial.print(" ");
  BTemp=ReferenceAccumulator/ (long) SamPerDeg;
  // handle slope compensation
  ft = (float) BTemp;
  ft= ft * Slope; 
  Temp2 = (unsigned int) ft; 
//  Temp2= (unsigned int) BTemp;
  Serial.println(Temp2);
}

void Macro1S() {
  while (bitRead(TIFR1,TOV1) ==0) ;;     // wait til PWM1 period is up
  bitSet(TIFR1,TOV1);   // Clear Timer1 overflow flag
  sample =(GetSampleRTD() -(float)1000.0);
  // alpha varies from 3.9 At 0C to 3.648 at 450C
  // use a linear function to adjust this
  C5 = 3.9 - (0.0001535 * sample);   
  sample=sample/C5;   // convert RTD res to temperature 
}
void Macro1R() {
  while (bitRead(TIFR1,TOV1) ==0) ;;     // wait til PWM1 period is up
  bitSet(TIFR1,TOV1);   // Clear Timer1 overflow flag
  reference =(GetReferenceRTD() - (float)1000.0);
  C5= 3.9- (0.0001535 * reference);
  reference=reference/C5;
}

void Macro2S() {
  SampleErrorTerm= ((float)CurrentSampleDeviation * (float) PropGain)+ float(SampleIntegral);
   MaxPWM= (float)24995.0;
  if (Setpoint < 250) {
    MaxPWM= (float) 100.0 * (float) Setpoint;
  }
  if (SampleErrorTerm > MaxPWM) {
    SampleErrorTerm= MaxPWM;
  }
  if (SampleErrorTerm < MinPWM) {
    SampleErrorTerm=MinPWM;
  }
  SetHeaterS();
}

void Macro2R() {
  ReferenceErrorTerm = ((float) CurrentReferenceDeviation * (float) PropGain) + float(ReferenceIntegral);
  MaxPWM= (float)24995.0;
  if (Setpoint < 250) {
    MaxPWM= (float) 100.0 * (float) Setpoint;
  }
  if (ReferenceErrorTerm >MaxPWM) {
    ReferenceErrorTerm =MaxPWM;
  }
  if (ReferenceErrorTerm < MinPWM) {
    ReferenceErrorTerm = MinPWM;
  }
  SetHeaterR();
}

// calculates the sample  deviations in terms of deg X 100
void Macro3S(){
 CurrentSampleDeviation = Setpoint - sample;
 Temp5 = (float) 100.0 * CurrentSampleDeviation;
 // Limit deviation * 100 to integer range
 if (Temp5 > 32760) {
  Temp5 =32760;
 }
 if (Temp5 < -32760){
  Temp5 = -32760;
 }
 Dev1 = (int) Temp5;
} 

// calculates the reference deviations in terms of deg X 100
void Macro3R(){
 CurrentReferenceDeviation = Setpoint - reference;
 Temp5 =(float) 100.0 * CurrentReferenceDeviation;
  // Limit deviation * 100 to integer range
 if (Temp5 > 32760) {
  Temp5 =32760;
 }
 if (Temp5 < -32760){
  Temp5 = -32760;
 }
 Dev2 = (int) Temp5;
}

float GetReferenceRTD(void) {
 int voltage; 
 long V1, V2;
 float RTD1;
 voltage =  readADC(3); // Gives A2-A3 readings
 if (voltage > 32760) {
  AbortFlag=true;      // RTD Sensor open
 }
 //Serial.print(voltage);
 V2 = (long) voltage + Vofs;
 V1=Vref-V2;  // Vref as expressed in ADC counts
 RTD1= ((float) V2 * (float)1000.0)/(float)V1;
// Serial.println(RTD1);
 return RTD1;  // RTD1 in ohms
}

float GetSampleRTD(void) {
 int voltage;
  long  V1,V2;
 float RTD1;
 voltage =  readADC(2);  // Gives A1-A3 reading
 if (voltage >32760) {
  AbortFlag=true;    // RTD sensor open
 }
 
 V2 = (long) voltage + Vofs;      // A3 offset voltage in ADC counts
 V1=Vref-V2;    // Vref is expressed in ADC counts
 RTD1= ((float) V2 * (float) 1000.0)/(float)V1;   // RTD1 in ohms 
// Serial.println(RTD1);
 return RTD1;
}

float GetVoltageReference(void) {
  int t;
  float voltage;
  t = readADC(4);   // Gives A0 single-ended reading
  Vref= t * 8.5;     //  8.5 to 1 resistive divider placed between A0 and Vref (~4.096 V)
  Vofs = (Vref *1.5)/2.5;    // offset voltage is Vref passed  thru 1K and 1.5K divider
}

static void i2cwrite(uint8_t x) {
  #if ARDUINO >= 100
  Wire.write((uint8_t)x);
  #else
  Wire.send(x);
  #endif
}

static uint8_t i2cread(void) {
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

static void writeRegister(uint8_t i2cAddress, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(i2cAddress);
  i2cwrite((uint8_t)reg);
  i2cwrite((uint8_t)(value>>8));
  i2cwrite((uint8_t)(value & 0xFF));
  Wire.endTransmission();
}

static uint16_t readRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  i2cwrite(0x00);
  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, (uint8_t)2);
  return ((i2cread() << 8) | i2cread());  
}

uint16_t readADC(uint8_t channel) {      // 0-3 are differential, 4-7 are S.Ended
  uint16_t config =0x0963;               // 64 SPS & disable comparator 
    // Set channel field
  config |= channel<<12;
    // Set 'start single-conversion' bit
  config |= 0x8000;
  // Write config register to the ADC
  writeRegister(0x48, 0x01, config);
  // Wait for the conversion to complete- 16 ms for 64 SPS
  delay(16);
  // Read the conversion results
  return readRegister(0x48,0x00);  
}

void InitPWMs(void) {
 pinMode(9,OUTPUT);
 pinMode(10,OUTPUT);
 TCCR1A= 0xA2;
 TCCR1B= 0x1A;
 ICR1 =50000;
 OCR1A=2;
 OCR1B=2;
}
  
void SetHeaterS () {
  int tmp2;
  unsigned int temp3;
  // load sample heater value
  tmp2= (int) SampleErrorTerm;
  temp3=tmp2<<1;
  SampleAccumulator+=tmp2;
  OCR1A=temp3;
} 
  
void SetHeaterR () {
  int tmp2;
  unsigned int temp3;  
  // load reference heater value
  tmp2= (int) ReferenceErrorTerm;
  temp3=tmp2<<1;
  OCR1B=temp3;
//  ReferenceErrorTerm = ReferenceErrorTerm * Slope;
//  tmp2= (int) ReferenceErrorTerm;
  ReferenceAccumulator+=tmp2;
  
}
  
void LoadDefaults() {
 C6A = K6A;
 C7A = K7A;
 C8A = K8A;
 C9A = K9A;
 C10A = K10A;
 C11A = K11A;
 C6B = K6B;
 C7B = K7B;
 C8B = K8B;
 C9B= K9B;
 C10B = K10B;
 C11B = K11B;
 TickCount=0;
 DegPerMin=10;
 StartTemp=50.0;
 EndTemp=300.0;
 AcqMode=1;
 Pg=2000;     // was 1000
 PgIncr=15;   // was 15
}

void HeaterOff() {
  OCR1A=2;
  OCR1B=2;
}
void ReadSensors(){
 
  sample =(GetSampleRTD()-(float)1000.0);
  // alpha varies from 3.9 At 0C to 3.648 at 450C
  // use a linear function to adjust this
  C5 = 3.9 - (0.0001535 * sample);
  sample=sample/C5;   // convert RTD res to temperature 
  if (sample > 400.0) {
  Serial.print("Sample RTD Open");
  }
 else { 
 Serial.print(sample);
 }
 Serial.print(" ");
 reference =(GetReferenceRTD()- (float)1000.0);
 C5= 3.9- (0.0001535 * reference);
 reference=reference/C5;
 if (reference > 400.0) {
  Serial.print("Ref. RTD Open");
 }
 else {
  Serial.print(reference); 
 }

 Serial.print("  ");
 int HST= analogRead(0); 
  // in VB program, this is converted to deg C, using a rough formula derived in Excel spreadsheet using a 10K thermistor with Beta = 3860 
 // HST= HST/(int) 10;
 // HST= 75- HST;
  Serial.println(HST);
 cmd=0;
 
}

void HeatSinkTemp() {
   int HST= analogRead(0); 
   // convert to deg C, using a rough formula derived in Excel spreadsheet using a 10K thermistor with Beta = 3860 
   HST= HST/(int) 10;
   HST= 75- HST;
   Serial.println(HST);
} 

void SetSampleHeater() {
  int temp;
  unsigned int heat;
  temp=Serial.parseInt();
  Serial.println(temp);   //echo it
  heat=temp<<1;
  OCR1A=heat;
}

void SetReferenceHeater() {
  int temp;
  unsigned int heat;
  temp=Serial.parseInt();
  Serial.println(temp);   //echo it
  heat=temp<<1;
  OCR1B=heat;
}

void SetDegreesPerMinute() {
  DegPerMin=Serial.parseInt();
  Serial.println(DegPerMin);   //echo it
}

void SetStartTemp() {
  int st;
  st=Serial.parseInt();
  Serial.println(st);   //echo it
  StartTemp= (float) st;
}

void SetEndTemp() {
  int et;
  et=Serial.parseInt();
  Serial.println(et);   //echo it 
  EndTemp= (float) et;
}

void SetAcqMode() {
 AcqMode=Serial.parseInt();
 Serial.println(AcqMode);   //echo it
}

void SetPg() {
  Pg=Serial.parseInt();
  Serial.println(Pg);   //echo it
}

void SetPgIncr() {
  PgIncr = Serial.parseInt();
  Serial.println(PgIncr);   //echo it
}
void SetCooler() {
    int Coolerval;
  Coolerval=Serial.parseInt();
  Serial.println(Coolerval);   //echo it
  if ( Coolerval == 1) {
    CoolerOn();
  }
  else {
   CoolerOff();
  }
}  
 
void CoolerOn(){
 digitalWrite(8,HIGH);
}

void CoolerOff(){
  digitalWrite(8,LOW);
}

void RetrieveNvParameters() {
 int eeAddress = 1;
 EEPROM.get(eeAddress,C6A);
 EEPROM.get(eeAddress+4,C7A);
 EEPROM.get(eeAddress+8,C8A);
 EEPROM.get(eeAddress+12,C6B);
 EEPROM.get(eeAddress+16,C7B);
 EEPROM.get(eeAddress+20,C8B);
 EEPROM.get(eeAddress+24,C9A);
 EEPROM.get(eeAddress+28,C10A);
 EEPROM.get(eeAddress+32,C11A);
 EEPROM.get(eeAddress+36,C9B);
 EEPROM.get(eeAddress+40,C10B);
 EEPROM.get(eeAddress+44,C11B); 
cmd=0; 
}

void StoreNvParameters() {
 StoreNvUp();
 StoreNvDown();
 cmd=0;
}

void StoreNvUp(){
int eeAddress = 1;
 EEPROM.put(eeAddress,C6A);
 EEPROM.put(eeAddress+4,C7A);
 EEPROM.put(eeAddress+8,C8A);
 EEPROM.put(eeAddress+12,C6B);
 EEPROM.put(eeAddress+16,C7B);
 EEPROM.put(eeAddress+20,C8B); 
}

void StoreNvDown() {
 int eeAddress = 1;
 EEPROM.put(eeAddress+24,C9A);
 EEPROM.put(eeAddress+28,C10A);
 EEPROM.put(eeAddress+32,C11A);
 EEPROM.put(eeAddress+36,C9B);
 EEPROM.put(eeAddress+40,C10B);
 EEPROM.put(eeAddress+44,C11B);
}

void PrintParameters() {
 Serial.println(C6A); 
 Serial.println(C7A); 
 Serial.println(C8A); 
 Serial.println(C9A); 
 Serial.println(C10A); 
 Serial.println(C11A); 
 Serial.println(C6B); 
 Serial.println(C7B); 
 Serial.println(C8B); 
 Serial.println(C9B); 
 Serial.println(C10B); 
 Serial.println(C11B); 
 cmd=0;
}

void DownloadParamsUp() {
   while (Serial.available() < 4) ;;
  C6A=Serial.parseFloat();
  while (Serial.available() < 4) ;;
  C8A=Serial.parseFloat();      // retrieved this way, as PC program sends const, 1st then 2nd order terms
  while (Serial.available() < 4) ;;
  C7A=Serial.parseFloat();      // where this program stores them in order const, 2nd,1st order terms
  while (Serial.available() < 4) ;;
  C6B=Serial.parseFloat();
  while (Serial.available() < 4) ;;
  C8B=Serial.parseFloat();
  while (Serial.available() < 4) ;;
  C7B=Serial.parseFloat(); 
  StoreNvUp();
  cmd=0;
}

void DownloadParamsDown(){
  while (Serial.available() < 4) ;;
  C9A=Serial.parseFloat();
  while (Serial.available() < 4) ;;
  C11A=Serial.parseFloat();      // retrieved this way, as PC program sends in order: const, 1st then 2nd order terms
  while (Serial.available() < 4) ;;
  C10A=Serial.parseFloat();      // whereas this program stores them in order: const, 2nd,1st order terms
  while (Serial.available() < 4) ;;
  C9B=Serial.parseFloat();
  while (Serial.available() < 4) ;;
  C11B=Serial.parseFloat();
  while (Serial.available() < 4) ;;
  C10B=Serial.parseFloat();
  StoreNvDown();
  cmd =0;
}

void SetSlope() {
  while (Serial.available() < 4) ;;
  Slope=Serial.parseFloat();
  Slope = Slope-1;    // parse float has problems with slope <1.0 when VB sends it, VB adds 1.0, so we subtract 1.0 here
  SlopeSet=1;
  int eeAddress = 50;
  EEPROM.put(eeAddress,SlopeSet);
  EEPROM.put(eeAddress+1,Slope);
}
