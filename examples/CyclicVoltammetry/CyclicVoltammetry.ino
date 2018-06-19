#include <LMP91000.h>
#include <Wire.h>

LMP91000 pstat = LMP91000();

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  delay(50);
  pstat.standby();
  delay(50);
  pstat.disableFET();
  pstat.setThreeLead();
  pstat.setGain(2);


  for (int j = 0; j < 3; j++)
  {
    pstat.setNegBias();
    for (int i = 0; i < 11; i++)
    {
      pstat.setBias(i);
      Serial.print(i*-1);
      Serial.print(",");
      delay(1);
      Serial.println(analogRead(A0));
      delay(50);
    }
    for (int i = 10; i >= 0; i--)
    {
      pstat.setBias(i);
      Serial.print(i*-1);
      Serial.print(",");
      delay(1);
      Serial.println(analogRead(A0));
      delay(50);
    }
    pstat.setPosBias();
    for (int i = 1; i < 11; i++)
    {
      pstat.setBias(i);
      Serial.print(i*1);
      Serial.print(",");
      delay(1);
      Serial.println(analogRead(A0));
      delay(50);
    }
    for (int i = 10; i >= 0; i--)
    {
      pstat.setBias(i);
      Serial.print(i*1);
      Serial.print(",");
      delay(1);
      Serial.println(analogRead(A0));
      delay(50);
    }
  }
  
  pstat.setBias(0);

}

void loop() {
  // put your main code here, to run repeatedly:

}
