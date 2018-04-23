#include <SPI.h>
#include <KX022_SPI.h>

#define CS_PIN 7
KX022_SPI kx022(CS_PIN);
//KX022_SPI kx022(SPI,CS_PIN);// example of passing a SPI object / channel

void setup() 
{
  Serial.begin(115200);
  Serial.println("Starting test ");
  Serial.print("Init response was ");
  Serial.print(kx022.init());
}

void loop()
{
  Serial.print((float)kx022.getAccel(0));Serial.print(",");
  Serial.print((float)kx022.getAccel(1));Serial.print(",");
  Serial.println((float)kx022.getAccel(2));
  delay(100);
}
