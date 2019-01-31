#include <Wire.h>
#include <SPI.h>
#include <LMP91000.h>
#include <AD56X4.h>
#include <MemoryFree.h>;
#include "pgmStrToRAM.h"; //SerialUSB.println(freeMemory(), DEC);  // print how much RAM is available.


LMP91000 pStat = LMP91000();

const int AD56X4_SS_pin = 10;

const uint16_t opVolt = 3340;
const double v_tolerance = 0.0075;


//const uint16_t dacMin = 0; //0V
//const uint16_t dacMax = opVolt; //Vdd
//const uint16_t dacSpan = opVolt; //Vdd
const uint16_t dacResolution = pow(2,16)-1; //16-bit


bool debug = true;


//analog input pins to read voltages
const uint8_t LMP = A2;
const uint8_t C1 = A0;
const uint8_t C2 = A1;
const uint8_t DACRead = A3;
const uint8_t diffAmp = A6;
const uint8_t INA = A7;


void setup()
{
  Wire.begin();
  Serial.begin(115200);


  pinMode(AD56X4_SS_pin,OUTPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();
  
  AD56X4.reset(AD56X4_SS_pin,true);
  

  //enable the potentiostat
  delay(50);
  pStat.standby();
  delay(50);
  initLMP(0);
  delay(2000); //warm-up time for the gas sensor


  Serial.println(F("Ready!"));
}

void loop()
{
  //will hold the code here until a character is sent over the serial port
  //this ensures the experiment will only run when initiated
  while(!Serial.available());
  Serial.read();


  //prints column headings
  if(debug) Serial.println(F("Time(ms),Voltage,Sign,Bias Index,Bias,dacOut,DueDACValue,DAC Read,LMP,C1,C2"));
  else Serial.println(F("Time(ms),Voltage,LMP,C1,C2"));

  
  //lmpGain, cycles, startV(mV), endV(mV), vertex1(mV), vertex2(mV), stsepV(mV), rate (mV/s)
  runCV(4, 2, 0, 0, 600, -500, 2, 100);
}


void initLMP(uint8_t lmpGain)
{
  pStat.disableFET();
  pStat.setGain(lmpGain);
  pStat.setRLoad(0);
  pStat.setExtRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead();
  pStat.setBias(0);
  pStat.setPosBias();

  setOutputsToZero();
}


void setOutputsToZero()
{
  AD56X4.setChannel(AD56X4_SS_pin, AD56X4_SETMODE_INPUT, AD56X4_CHANNEL_B, 0);
  AD56X4.updateChannel(AD56X4_SS_pin, AD56X4_CHANNEL_B);
  pStat.setBias(0);
}


void runCV(uint8_t lmpGain, uint8_t cycles, int16_t startV,
           int16_t endV, int16_t vertex1, int16_t vertex2,
           int16_t stepV, uint16_t rate)
{
  initLMP(lmpGain);
  stepV = abs(stepV);
  rate = (1000.0*stepV)/rate;


  if(vertex1 > startV) runCVForward(cycles,startV,endV,vertex1,vertex2,stepV,rate);
  else runCVBackward(cycles,startV,endV,vertex1,vertex2,stepV,rate);
}



void runCVForward(uint8_t cycles, int16_t startV, int16_t endV,
                  int16_t vertex1, int16_t vertex2, int16_t stepV, uint16_t rate)
{
//  Serial.println("Forward");
//  Serial.println(startV);
//  Serial.println(endV);
//  Serial.println(vertex1);
//  Serial.println(vertex2);
//  Serial.println(stepV);
//  while(!Serial.available());
//  Serial.println(rate);
  
  int16_t j = startV;
  
  for(uint8_t i = 0; i < cycles; i++)
  {
    //j starts at startV
    for (j; j <= vertex1; j += stepV)
    {
      biasAndSample(j,rate);
//      Serial.println();
//      Serial.println();
//      Serial.println(freeMemory(), DEC);  // print how much RAM is available.
    }
    j -= 2*stepV;
  
  
    //j starts right below the first vertex
    for (j; j >= vertex2; j -= stepV)
    {
      biasAndSample(j,rate);
    }
    j += 2*stepV;
  
  
    //j starts right above the second vertex
    for (j; j <= endV; j += stepV)
    {
      biasAndSample(j,rate);
    }
    j -= 2*stepV;
    
  }

  setOutputsToZero();
}


void runCVBackward(uint8_t cycles, int16_t startV, int16_t endV,
                   int16_t vertex1, int16_t vertex2, int16_t stepV, uint16_t rate)
{
//  Serial.println("Backward");
//  Serial.println(startV);
//  Serial.println(endV);
//  Serial.println(vertex1);
//  Serial.println(vertex2);
//  Serial.println(stepV);
//  Serial.println(rate);
//
//  while(!Serial.available());
  
  int16_t j = startV;
  
  for(uint8_t i = 0; i < cycles; i++)
  {
    //j starts at startV
    for (j; j >= vertex1; j -= stepV)
    {
      biasAndSample(j,rate);
    }
    j += 2*stepV;
    

    //j starts right above vertex1
    for (j; j <= vertex2; j += stepV)
    {
      biasAndSample(j,rate);
    }
    j -= 2*stepV;
  

    //j starts right below vertex2
    for (j; j >= endV; j -= stepV)
    {
      biasAndSample(j,rate);
    }
    j += 2*stepV;
    
  }

  setOutputsToZero();
}



void biasAndSample(int16_t voltage, uint16_t rate)
{
  Serial.print(millis());
  Serial.print(F(","));
  Serial.print(voltage);
  Serial.print(F(","));
  
  setLMPBias(voltage);
  setVoltage(voltage);
  
  delay(rate);
  sampleOutputs();
  Serial.println();
}



void sampleOutputs()
{
  Serial.print(analogRead(DACRead));
  Serial.print(F(","));
  Serial.print(analogRead(LMP));
  Serial.print(F(","));
  Serial.print(analogRead(C1));
  Serial.print(F(","));
  Serial.print(analogRead(C2));
}



void setVoltage(int16_t voltage)
{
  uint16_t dacVout = 1500;
  uint8_t bias_setting = 0;

  if(abs(voltage) < 15) voltage = 15*(voltage/abs(voltage));    
  
  int16_t setV = dacVout*TIA_BIAS[bias_setting];
  voltage = abs(voltage);
  
  
  while(setV > voltage*(1+v_tolerance) || setV < voltage*(1-v_tolerance))
  {
    if(bias_setting == 0) bias_setting = 1;
    
    dacVout = voltage/TIA_BIAS[bias_setting];
    
    if (dacVout > opVolt)
    {
      bias_setting++;
      dacVout = 1500;

      if(bias_setting > NUM_TIA_BIAS) bias_setting = 0;
    }
    
    setV = dacVout*TIA_BIAS[bias_setting];    
  }


  pStat.setBias(bias_setting);
  AD56X4.setChannel(AD56X4_SS_pin, AD56X4_SETMODE_INPUT, AD56X4_CHANNEL_B, convertDACVoutToDACVal(dacVout));
  AD56X4.updateChannel(AD56X4_SS_pin, AD56X4_CHANNEL_B);


  if(debug)
  {
    Serial.print(bias_setting);
    Serial.print(F(","));
    Serial.print(TIA_BIAS[bias_setting]);
    Serial.print(F(","));
    Serial.print(dacVout);
    Serial.print(F(","));
    //Serial.print(convertDACVoutToDACVal(dacVout));
    Serial.print(convertDACVoutToDACVal(dacVout) >> 2);
    Serial.print(F(","));
  }
}



//Convert the desired voltage
uint16_t convertDACVoutToDACVal(uint16_t dacVout)
{
  //return (dacVout-dacMin)*((double)dacResolution/dacSpan);
  return dacVout*((double)dacResolution/opVolt);
}



void setLMPBias(int16_t voltage)
{
  signed char sign = (double)voltage/abs(voltage);
  
  if(sign < 0) pStat.setNegBias();
  else if (sign > 0) pStat.setPosBias();
  else {} //do nothing

  if(debug)
  {
    Serial.print(sign);
    Serial.print(F(","));
  }
}
