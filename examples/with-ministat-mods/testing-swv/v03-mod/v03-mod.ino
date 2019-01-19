#include <LMP91000.h>
#include <Wire.h>

LMP91000 pStat = LMP91000();

int16_t opVolt = 3300; //milliVolts if working with a 3.3V device
uint8_t resolution = 10; //10-bits

const uint8_t menb = 4;
const uint8_t mux = 3;

//const uint16_t half_period = 4000;
//const uint16_t half_period = 1000;
const uint16_t half_period = 200;
//const uint16_t half_period = 2;
double adcVal = 0;
const uint16_t offset = 511;


void setup()
{
  Wire.begin();
  Serial.begin(115200);


  pinMode(menb,OUTPUT);
  pinMode(mux,OUTPUT);
  digitalWrite(menb,LOW);
  digitalWrite(mux,LOW);


  pStat.standby();
  delay(3000); //warm-up time for the gas sensor


  pStat.disableFET();
  pStat.setGain(1);
  pStat.setRLoad(0);
  pStat.setIntRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead(); 
  pStat.setBias(0);
  pStat.setPosBias();
  delay(3000);
  //Serial.println("Time(ms),Index,Forward,Reverse,Difference");
  Serial.println("Time(ms),Index,ADC-Forward,adc-pre-invert,Forward,ADC-Reverse,adc-pre-invert,Reverse,Difference");

}


void loop()
{
  Serial.print(millis());
  Serial.print(",");
  Serial.print(2);
  Serial.print(",");
    
  pStat.setBias(3);
  delay(half_period);
  sampleCurrent();
  Serial.print(",");
    
  pStat.setBias(0);
  delay(half_period);
  sampleCurrent();
  Serial.println();

    
  for(int i = 4; i < 14; i++)
  {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(i-1);
    Serial.print(",");
    
    pStat.setBias(i);
    delay(half_period);
    sampleCurrent();
    Serial.print(",");
    
    pStat.setBias(i-2);
    delay(half_period);
    sampleCurrent();
    Serial.println();
  }

  while(!Serial.available());
  Serial.read();
}


void sampleCurrent()
{
  adcVal = analogRead(A0);
  Serial.print(adcVal);
  Serial.print(",");
  adcVal = -1*(adcVal-offset)+offset;
  Serial.print(adcVal);
  Serial.print(",");
  Serial.print(pow(10,6)*pStat.getCurrent(adcVal, opVolt/1000.0, resolution));
}
