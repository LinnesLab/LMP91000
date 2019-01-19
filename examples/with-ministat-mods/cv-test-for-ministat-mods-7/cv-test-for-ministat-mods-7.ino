#include <LMP91000.h>
#include <Wire.h>

LMP91000 pstat = LMP91000();

int rate = 200;
int settling_time = 50;

const uint8_t menb = 4;
const uint8_t mux = 3;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  analogReference(EXTERNAL);
  
  pinMode(menb,OUTPUT);
  pinMode(mux,OUTPUT);

  digitalWrite(menb,LOW);
  digitalWrite(mux,LOW);
  

  delay(50);
  pstat.standby();
  delay(50);
  pstat.disableFET();
  pstat.setGain(6);
  pstat.setRLoad(3);
  //pstat.setIntRefSource();
  pstat.setExtRefSource();
  pstat.setIntZ(1);
  pstat.setThreeLead();
  delay(2000); //warm-up time for the gas sensor


  while(!Serial.available());
  Serial.read();


  pstat.setPosBias();
  //pstat.setNegBias();
  pstat.setBias(13);

  Serial.println("Time(ms),ADC(10-bit 3.3V)");
}


void loop()
{
  int c1 = analogRead(A1);
  int c2 = analogRead(A0);
  int volt = analogRead(A2);
  
  Serial.print(millis());
  Serial.print(",");
  //Serial.print((volt*3.3/1023)*.24);
  Serial.print(volt);
  Serial.print(",");
  Serial.print(c1);
  Serial.print(",");
  Serial.print(c2);
  Serial.print(",");
  Serial.println(c2-c1);
  delay(50);
}
