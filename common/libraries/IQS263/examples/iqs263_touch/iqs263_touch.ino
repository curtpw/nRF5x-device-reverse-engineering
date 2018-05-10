#include "IQS263.h"

IQS263 s263(17);

void setup() {
  Serial.begin(115200);

  Serial.println("IQS263 example starting");

  if (s263.init()<0)
  {
	  Serial.println("Error in s263.init");
	  while(true);
  }
  else
  {
	  Serial.println("s263.init complete");
  }
}

void loop()
{
	iqs263EventData evt = s263.getEvents();
	if(evt.eventFlags)
	{
		Serial.print("Event 0x");	Serial.print(evt.eventFlags,HEX);
		Serial.print(" Touch 0x");	Serial.print(evt.touchFlags,HEX);
		Serial.print(" Wheel ");	Serial.print(evt.wheelPos);
		Serial.print(" Relative "); Serial.println(evt.relativePos);
	}
}