#include <LMP91000.h>
#include <Wire.h>

LMP91000 pStat = LMP91000();

int16_t opVolt = 3300; //milliVolts if working with a 3.3V device
uint8_t resolution = 10; //10-bits

const uint8_t menb = 4;
const uint8_t mux = 3;

//const uint16_t half_period = 2;
const uint16_t half_period = 2200;
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
  pStat.setGain(2);
  pStat.setRLoad(0);
  pStat.setIntRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead();
  pStat.setBias(0);
  pStat.setPosBias();
  Serial.println("Time(ms),Index,Forward,Reverse");


  for(int i = 13; i > 3; i--)
  {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(i-1);
    Serial.print(",");
    
    pStat.setBias(i);
    delayMicroseconds(half_period);
    sampleCurrent();
    Serial.print(",");
    
    pStat.setBias(i-2);
    delayMicroseconds(half_period);
    sampleCurrent();
    Serial.println();
  }
  

  Serial.print(millis());
  Serial.print(",");
  Serial.print(2);
  Serial.print(",");
    
  pStat.setBias(3);
  delayMicroseconds(half_period);
  sampleCurrent();
  Serial.print(",");
    
  pStat.setBias(0);
  delayMicroseconds(half_period);
  sampleCurrent();
  Serial.println();

}


void loop()
{
}


void sampleCurrent()
{
  adcVal = analogRead(A0);
  adcVal = -1*(adcVal-offset)+offset;
  Serial.print(pow(10,6)*pStat.getCurrent(adcVal, opVolt/1000.0, resolution));
}
