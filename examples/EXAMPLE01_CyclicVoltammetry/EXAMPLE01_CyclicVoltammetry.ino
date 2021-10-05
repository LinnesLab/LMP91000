#include <LMP91000.h>
#include <Wire.h>

LMP91000 pstat = LMP91000();


//settling_time variable determines the delay between changing
//the bias and sampling based on the desired scan rate and step
//potential the step potential is 66 mV for the LMP91000 at 3.3 V
//power and using the internal reference (or external reference
//of 3.3 V).
//
//figures out the delay needed to achieve a given scan rate
//delay is dependent on desired rate and number of steps taken
//more steps = smaller delay since we'll need to go by a bit faster
//to sample at more steps vs. less steps, but at the same rate
const uint16_t settling_time = (1000.0*66)/50;

//LMP91000 is powered by 3.3 V (3300 mV)
 //milliVolts (mV) power supply voltage
const uint16_t lmp_vcc = 3300;
//some Arduinos run on 5 V, some run on 3.3 V. Change this line accordingly
const float aref_voltage = 5.0;
//some Arduinos have a 12-bit ADC, some have a 10-bit ADC. Change this line accordingly.
const uint8_t adc_bits = 10;


void setup()
{
  Wire.begin();
  Serial.begin(9600);
  while(!Serial); //won't run code until serial monitor is open
  

  delay(50);
  pstat.standby();
  delay(50);
  pstat.disableFET();
  pstat.setGain(2); //chnage your gain as necessary. Bigger number higher gain. Lower number, lower gain.
  pstat.setRLoad(0);
  pstat.setIntRefSource();
  pstat.setIntZ(1);
  pstat.setThreeLead();
  delay(3000); //warm-up time for the gas sensor


  Serial.println("Time(ms),Voltage(mV),ADC(10-bit),Current(uA)");
  //Runs Cyclic Voltammetry (CV) from -0.792 to +0.792
  //with a 66 mV step potential and a scan rate of 50 mV/s
  //number of scans = 3
  for (int j = 0; j < 3; j++)
  {
    pstat.setNegBias();
    for (int i = 1; i <= 13; i++)
    {
      pstat.setBias(i);
      delay(settling_time);
      uint16_t adcVal = analogRead(A0);
      
      Serial.print(millis());
      Serial.print(",");
      Serial.print(-1*TIA_BIAS[i]*lmp_vcc);
      Serial.print(",");
      Serial.print(adcVal);
      Serial.print(",");
      //prints the current in units of microAmperes
      //Some Arduinos run on 5 V, some run on 3.3 V. Change aref_voltage accordingly.
      //Some Arduinos have a 12-bit ADC, some have a 10-bit ADC. Change adc_bits accordingly.
      Serial.print(pstat.getCurrent(adcVal, aref_voltage, adc_bits)*1000000,4);
      Serial.println();
    }
    for (int i = 12; i >= 0; i--)
    {
      pstat.setBias(i);
      delay(settling_time);
      uint16_t adcVal = analogRead(A0);
      
      Serial.print(millis());
      Serial.print(",");
      Serial.print(-1*TIA_BIAS[i]*lmp_vcc);
      Serial.print(",");
      Serial.print(adcVal);
      Serial.print(",");
      //prints the current in units of microAmperes
      //Some Arduinos run on 5 V, some run on 3.3 V. Change aref_voltage accordingly.
      //Some Arduinos have a 12-bit ADC, some have a 10-bit ADC. Change adc_bits accordingly.
      Serial.print(pstat.getCurrent(adcVal, aref_voltage, adc_bits)*1000000,4);
      Serial.println();
    }
    pstat.setPosBias();
    for (int i = 1; i <= 13; i++)
    {
      pstat.setBias(i);
      delay(settling_time);
      uint16_t adcVal = analogRead(A0);
      
      Serial.print(millis());
      Serial.print(",");
      Serial.print(TIA_BIAS[i]*lmp_vcc);
      Serial.print(",");
      Serial.print(adcVal);
      Serial.print(",");
      //prints the current in units of microAmperes
      //Some Arduinos run on 5 V, some run on 3.3 V. Change aref_voltage accordingly.
      //Some Arduinos have a 12-bit ADC, some have a 10-bit ADC. Change adc_bits accordingly.
      Serial.print(pstat.getCurrent(adcVal, aref_voltage, adc_bits)*1000000,4);
      Serial.println();
    }
    for (int i = 12; i >= 0; i--)
    {
      pstat.setBias(i);
      delay(settling_time);
      uint16_t adcVal = analogRead(A0);
      
      Serial.print(millis());
      Serial.print(",");
      Serial.print(TIA_BIAS[i]*lmp_vcc);
      Serial.print(",");
      Serial.print(adcVal);
      Serial.print(",");
      //prints the current in units of microAmperes
      //Some Arduinos run on 5 V, some run on 3.3 V. Change aref_voltage accordingly.
      //Some Arduinos have a 12-bit ADC, some have a 10-bit ADC. Change adc_bits accordingly.
      Serial.print(pstat.getCurrent(adcVal, aref_voltage, adc_bits)*1000000,4);
      Serial.println();
    }
  }
  
  pstat.setBias(0);

}


void loop()
{
}
