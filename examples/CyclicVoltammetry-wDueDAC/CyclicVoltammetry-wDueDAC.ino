#include <Wire.h>
#include <LMP91000.h>


LMP91000 pStat = LMP91000();


uint16_t opVolt = 3310;
const double v_tolerance = 0.01;


const uint16_t dacMin = 550; //0.55V
const uint16_t dacMax = 2750; //2.75V
const uint16_t dacSpan = 2200; //2.2V
const uint16_t dacResolution = 4095; //12-bit


bool debug = true;


void setup()
{
  Wire.begin();
  Serial.begin(115200);

  //setting resolution to 12 bits
  analogReadResolution(12);
  analogWriteResolution(12);
  

  //enable the potentiostat
  delay(50);
  pStat.standby();
  delay(50);
  initLMP(0);  
  delay(2000); //warm-up time for the gas sensor
  
  
  if(debug) Serial.println("Time(ms),Voltage,Sign,Bias Index,Bias,dacOut,DueDACValue,DAC Read,LMP,C1,C2");
  else Serial.println("Time(ms),Voltage,LMP,C1,C2");


  //will hold the code here until a character is sent over the serial port
  //this ensures the experiment will only run when initiated
  while(!Serial.available());
  Serial.read();


  //lmpGain, cycles, startV, endV, stepV, rate
  runCV(0, 3, -470, 50, 3, 50);
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
  analogWrite(DAC0,0);
}



void runCV(int8_t lmpGain, int8_t cycles, int16_t startV,
           int16_t endV, int16_t vertex1, int16_t vertex2,
           int16_t stepV, int16_t rate)
{
  initLMP(lmpGain);

  rate = (1000.0*stepV)/rate;
  int16_t j = startV;


  //initial conditioning
  //change debug flag to avoid printing
  //out data from conditioning time
  debug = false;
  setLMPBias(j);
  setVoltage(j);
  delay(6000);
  debug = true;


  for(uint8_t i = 0; i < cycles; i++)
  {
    for (j; j <= endV; j += stepV)
    {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(j);
      Serial.print(",");
      
      setLMPBias(j);
      setVoltage(j);
      
      delay(rate);
      
      if(debug)
      {
        Serial.print(analogRead(A3)); //dac output voltage
        Serial.print(",");
      }
      Serial.print(analogRead(A0)); //lmp output
      Serial.print(",");
      Serial.print(analogRead(A1)); //C1
      Serial.print(",");
      Serial.print(analogRead(A2)); //C2
      Serial.println();
    }
    j -= 2*stepV;

    
    for (j; j >= startV; j -= stepV)
    {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(j);
      Serial.print(",");
      
      setLMPBias(j);
      setVoltage(j);
      
      delay(rate);

      if(debug)
      {
        Serial.print(analogRead(A3)); //dac output voltage
        Serial.print(",");
      }
      Serial.print(analogRead(A0)); //lmp output
      Serial.print(",");
      Serial.print(analogRead(A1)); //C1
      Serial.print(",");
      Serial.print(analogRead(A2)); //C2
      Serial.println();
    }
    j += 2*stepV;
    
  }

  analogWrite(DAC0,0);
  pStat.setBias(0);
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
