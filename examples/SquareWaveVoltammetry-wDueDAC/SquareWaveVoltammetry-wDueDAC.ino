#include <Wire.h>
#include <LMP91000.h>


LMP91000 pStat = LMP91000();

int settling_time = 1;

uint16_t opVolt = 3280;

uint16_t i = 0;

double bias[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24};
uint16_t  dacVal = 1500; //in milliVolts
uint8_t  bias_setting = 0;
const double v_tolerance = 0.1; //10%


uint16_t dacMin = 550; //0.55V
uint16_t dacMax = 2750; //2.75V
uint16_t dacSpan = 2200; //2.2V


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
  pStat.disableFET();
  pStat.setGain(3);
  pStat.setRLoad(0);
  pStat.setExtRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead();
  delay(2000); //warm-up time for the gas sensor


  Serial.println("Time(ms),DAC Val,Bias Setting,Voltage,LMPOut,C1,C2,DACOutMeasured,Time(ms),DAC Val,Bias Setting,Voltage,LMPOut,C1,C2,DACOutMeasured,ADCs 10-bit 3.3V");
  

  //will hold the code here until a character is sent over the serial port
  //this ensures the experiment will only run when initiated
  while(!Serial.available());
  Serial.read();

  //lmpGain, startV, endV, pulse amplitude, stepV, frequency
  runSWV(3, 20, 500, 20, 2, 50);
}

void loop()
{
//  uint16_t randNumber = random(dacMin,dacMax);
//
//  i = (randNumber-dacMin)*((double)4095/dacSpan);
//
//  analogWrite(DAC0,i);
//  Serial.print((double)randNumber/1000,2);
//  Serial.print(",");
//  Serial.println(i);
//  
//  
//  delay(2000);
}


//Convert the desired voltage 
void convertToArduinoDueDACVal()
{
  dacVal = (dacVal-dacMin)*((double)4095/dacSpan);

//  Serial.print("dacVal: ");
//  Serial.print(dacVal);
//  Serial.print(", ");
}


void runSWV(uint8_t lmpGain, int16_t startV, int16_t endV,
            uint8_t pulseAmp, int16_t stepV, uint16_t freq)
{
  //initializes LMP91000
  pStat.disableFET();
  pStat.setGain(lmpGain);
  pStat.setRLoad(0);
  pStat.setExtRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead();
  pStat.setBias(0);
  pStat.setPosBias();
  

  pStat.setPosBias();
  for (int16_t j = startV; j <= endV; j += stepV)   //Steps from the starting voltage to the end voltage increased by the volt_step
  {
//    Serial.print("j: ");
//    Serial.print(j);
//    Serial.print(", ");
//    
//    Serial.print("Pulse Up: ");
    setVoltage(j + pulseAmp);
//    Serial.print(", ");

//    Serial.print("j + pulseAmp: ");
//    Serial.print(j+pulseAmp);
//    Serial.print(",");
    
    
    convertToArduinoDueDACVal();
    analogWrite(DAC0,dacVal);
    if(j+pulseAmp < 0) pStat.setNegBias();
    else if (j+pulseAmp > 0) pStat.setPosBias();
    else {}
    pStat.setBias(bias_setting);
    delay(1000 / freq);
    

    Serial.print(millis());
    Serial.print(",");
    Serial.print(dacVal);
    Serial.print(",");
    Serial.print(bias[bias_setting]);
    Serial.print(",");
    Serial.print(j + pulseAmp);
    Serial.print(",");
    Serial.print(analogRead(A0));
    Serial.print(",");
    Serial.print(analogRead(A1));
    Serial.print(",");
    Serial.print(analogRead(A2));
    Serial.print(",");
    Serial.print(analogRead(A3)); //voltage output of the DAC
    Serial.print(",");
    //delay(3000);


    //Serial.print("Pulse Down: ");
    setVoltage(j - pulseAmp);
    //Serial.println();

//    Serial.print("j - pulseAmp: ");
//    Serial.println(j-pulseAmp);

    
    convertToArduinoDueDACVal();
    analogWrite(DAC0,dacVal);
    if(j-pulseAmp < 0) pStat.setNegBias();
    else if (j-pulseAmp > 0) pStat.setPosBias();
    else {}
    pStat.setBias(bias_setting);
    delay(1000 / freq);
    
    
    Serial.print(millis());
    Serial.print(",");
    Serial.print(dacVal);
    Serial.print(",");
    Serial.print(bias[bias_setting]);
    Serial.print(",");
    Serial.print(j - pulseAmp);
    Serial.print(",");
    Serial.print(analogRead(A0));
    Serial.print(",");
    Serial.print(analogRead(A1));
    Serial.print(",");
    Serial.print(analogRead(A2));
    Serial.print(",");
    Serial.print(analogRead(A3)); //voltage output of the DAC
    Serial.println();
    //delay(3000);

  }
}


int16_t setVoltage(int16_t num)
{
//  Serial.print("num: ");
//  Serial.print(num);

  
  if(abs(num) < 15) num = 15*(num/abs(num));
  
  dacVal = 1500;
  bias_setting = 0;
  
  
  int16_t setV = dacVal*bias[bias_setting];
  num = abs(num);
  
  //outside the 10% tolerance range
  while(setV > num*(1+v_tolerance) || setV < num*(1-v_tolerance))
  {
    if(bias_setting == 0) bias_setting = 1;
    
    dacVal = num/bias[bias_setting];
    
    //if (dacVal > 2950)
    if (dacVal > dacMax)
    {
      bias_setting++;
      dacVal = 1500;
    }
    
    setV = dacVal*bias[bias_setting];    
  }

//  Serial.print(", ");
//  Serial.print("dacVal: ");
//  Serial.print(dacVal);
//  Serial.print(", ");
//  Serial.print("bias_setting: ");
//  Serial.print(bias[bias_setting]);
  
  return setV;
}
