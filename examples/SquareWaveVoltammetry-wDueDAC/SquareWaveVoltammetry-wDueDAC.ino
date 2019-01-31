/*
 * FILENAME:  SquareWaveVoltammetry-wDueDAC.ino
 * AUTHOR:    Orlando Hoilett
 * VERSION:   V.1.0.0
 * 
 * 
 * DESCRIPTION
 * 
 * 
 * 
 * UPDATES
 * 
 * 
 * 
 * DISCLAIMER
 * Linnes Lab code, firmware, and software is released under the
 * MIT License (http://opensource.org/licenses/MIT).
 * 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2019 Linnes Lab, Purdue University
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 * 
 * 
 */


#include <Wire.h>
#include <LMP91000.h>


LMP91000 pStat = LMP91000();


uint16_t opVolt = 3310;
const double v_tolerance = 0.001;


const uint16_t dacMin = 550; //0.55V
const uint16_t dacMax = 2750; //2.75V
const uint16_t dacSpan = 2200; //2.2V
const uint16_t dacResolution = 4095; //12-bit


const bool debug = true;


void setup()
{
  Wire.begin();
  Serial.begin(115200);
  //Serial.begin(250000);

  //setting resolution to 12 bits
  analogReadResolution(12);
  analogWriteResolution(12);
  

  //enable the potentiostat
  delay(50);
  pStat.standby();
  delay(50);
  initLMP(5);
  delay(2000); //warm-up time for the gas sensor


  if(debug) Serial.println("Time(ms),Voltage,VF,Sign,Bias Index,Bias,dacOut,DueDACValue,Delay,DAC Read,LMP,C1,C2,Time(ms),Voltage,VR,Sign,Bias Index,Bias,dacOut,DueDACValue,Delay,DAC Read,LMP,C1,C2");
  else Serial.println("Time(ms),Volt,LMP,C1,C2,LMP,C1,C2");


  //will hold the code here until a character is sent over the serial port
  //this ensures the experiment will only run when initiated
  while(!Serial.available());
  Serial.read();


//  //lmpGain, startV, endV, pulse amplitude, stepV, frequency
//  runSWV(3, 0, 500, 20, 4, 60);
//  delay(3000);
//  
//  Serial.println("Run backwards");
  if(debug) Serial.println("Time(ms),Voltage,VF,Sign,Bias Index,Bias,dacOut,DueDACValue,Delay,DAC Read,LMP,C1,C2,Time(ms),Voltage,VR,Sign,Bias Index,Bias,dacOut,DueDACValue,Delay,DAC Read,LMP,C1,C2");
  else Serial.println("Time(ms),Volt,LMP,C1,C2,LMP,C1,C2");
  runSWVReverse(5, -50, -450, 50, 3, 250);
}


void loop()
{
  while(!Serial.available());
  Serial.read();
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
}


void runSWV(uint8_t lmpGain, int16_t startV, int16_t endV,
            uint8_t pulseAmp, int16_t stepV, uint8_t freq)
{
  initLMP(lmpGain);

  for (int16_t j = startV; j <= endV; j += stepV)   //Steps from the starting voltage to the end voltage increased by the volt_step
  {
    //POSITIVE pulse of square wave
    if(debug)
    {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(j);
      Serial.print(",");
      Serial.print(j + pulseAmp);
      Serial.print(",");
    }

    setLMPBias(j + pulseAmp);
    setVoltage(j + pulseAmp);
    delay((uint8_t)(1000.0 / (2*freq)));

    if(debug)
    {
      Serial.print((uint16_t)(1000.0 / (2*freq)));
      Serial.print(",");
      Serial.print(analogRead(A3)); //DAC output voltage
      Serial.print(",");
    }

    Serial.print(analogRead(A0)); //lmp output
    Serial.print(",");
    Serial.print(analogRead(A1)); //C1
    Serial.print(",");
    Serial.print(analogRead(A2)); //C2
    Serial.print(",");

//    Serial.print(pStat.getCurrent(analogRead(A0), opVolt, 12)); //lmp output
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A1), opVolt, 12)); //C1
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A2), opVolt, 12)); //C2
//    Serial.print(",");
    //delay(3000);



    ////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////
    //NEGATIVE pulse of square wave
    if(debug)
    {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(j);
      Serial.print(",");
      Serial.print(j - pulseAmp);
      Serial.print(",");
    }

    setLMPBias(j - pulseAmp);
    setVoltage(j - pulseAmp);
    delay((uint8_t)(1000.0 / (2*freq)));

    if(debug)
    {
      Serial.print((uint16_t)(1000.0 / (2*freq)));
      Serial.print(",");
      Serial.print(analogRead(A3)); //DAC output voltage
      Serial.print(",");
    }

    Serial.print(analogRead(A0)); //lmp output
    Serial.print(",");
    Serial.print(analogRead(A1)); //C1
    Serial.print(",");
    Serial.print(analogRead(A2)); //C2
    Serial.println();

//    Serial.print(pStat.getCurrent(analogRead(A0), opVolt, 12)); //lmp output
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A1), opVolt, 12)); //C1
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A2), opVolt, 12)); //C2
//    Serial.println();
    //delay(3000);
  }
  
}


void runSWVReverse(uint8_t lmpGain, int16_t startV, int16_t endV,
            uint8_t pulseAmp, int16_t stepV, uint8_t freq)
{
  initLMP(lmpGain);

  for (int16_t j = startV; j >= endV; j -= stepV)   //Steps from the starting voltage to the end voltage increased by the volt_step
  { 
    //POSITIVE pulse of square wave
    if(debug)
    {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(j);
      Serial.print(",");
      Serial.print(j + pulseAmp);
      Serial.print(",");
    }

    setLMPBias(j + pulseAmp);
    setVoltage(j + pulseAmp);
    delay((uint8_t)(1000.0 / (2*freq)));

    if(debug)
    {
      Serial.print((uint16_t)(1000.0 / (2*freq)));
      Serial.print(",");
      Serial.print(analogRead(A3)); //DAC output voltage
      Serial.print(",");
    }

    Serial.print(analogRead(A0)); //lmp output
    Serial.print(",");
    Serial.print(analogRead(A1)); //C1
    Serial.print(",");
    Serial.print(analogRead(A2)); //C2
    Serial.print(",");

//    Serial.print(pStat.getCurrent(analogRead(A0), opVolt, 12)); //lmp output
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A1), opVolt, 12)); //C1
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A2), opVolt, 12)); //C2
//    Serial.print(",");
    //delay(3000);



    ////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////
    //NEGATIVE pulse of square wave
    if(debug)
    {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(j);
      Serial.print(",");
      Serial.print(j - pulseAmp);
      Serial.print(",");
    }

    setLMPBias(j - pulseAmp);
    setVoltage(j - pulseAmp);
    delay((uint8_t)(1000.0 / (2*freq)));

    if(debug)
    {
      Serial.print((uint16_t)(1000.0 / (2*freq)));
      Serial.print(",");
      Serial.print(analogRead(A3)); //DAC output voltage
      Serial.print(",");
    }

    Serial.print(analogRead(A0)); //lmp output
    Serial.print(",");
    Serial.print(analogRead(A1)); //C1
    Serial.print(",");
    Serial.print(analogRead(A2)); //C2
    Serial.println();

//    Serial.print(pStat.getCurrent(analogRead(A0), opVolt, 12)); //lmp output
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A1), opVolt, 12)); //C1
//    Serial.print(",");
//    Serial.print(pStat.getCurrent(analogRead(A2), opVolt, 12)); //C2
//    Serial.println();
    //delay(3000);
  }
  
}


void setVoltage(int16_t voltage)
{
  uint16_t dacVout = 1500;
  uint8_t bias_setting = 0;

  if(abs(voltage) < 15) voltage = 15*(voltage/abs(voltage));    
  
  int16_t setV = dacVout*TIA_BIAS[bias_setting];
  voltage = abs(voltage);
  
  //outside the 10% tolerance range
  while(setV > voltage*(1+v_tolerance) || setV < voltage*(1-v_tolerance))
  {
    if(bias_setting == 0) bias_setting = 1;
    
    dacVout = voltage/TIA_BIAS[bias_setting];
    
    if (dacVout > dacMax)
    {
      bias_setting++;
      dacVout = 1500;
    }
    
    setV = dacVout*TIA_BIAS[bias_setting];    
  }


  pStat.setBias(bias_setting);
  analogWrite(DAC0, convertDACVoutToArduinoDueDACVal(dacVout));

  if(debug)
  {
    Serial.print(bias_setting);
    Serial.print(",");
    Serial.print(TIA_BIAS[bias_setting]);
    Serial.print(",");
    Serial.print(dacVout);
    Serial.print(",");
    Serial.print(convertDACVoutToArduinoDueDACVal(dacVout));
    Serial.print(",");
  }
}


//Convert the desired voltage
uint16_t convertDACVoutToArduinoDueDACVal(uint16_t dacVout)
{
  return (dacVout-dacMin)*((double)dacResolution/dacSpan);
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
    Serial.print(",");
  }
}
