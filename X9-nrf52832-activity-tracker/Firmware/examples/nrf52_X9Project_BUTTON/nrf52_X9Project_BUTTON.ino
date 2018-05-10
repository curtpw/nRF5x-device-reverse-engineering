
/********************************************************************************************************/
/************************ DEFINITIONS *******************************************************************/
/********************************************************************************************************/

#define HR_LED_PIN         4
#define TOUCH_BUTTON       30
#define VIBRATE_PIN        8


/********************************************************************************************************/
/************************ SETUP *************************************************************************/
/********************************************************************************************************/

void setup() 
{

  /************ I/O BUTTON, LED, HAPTIC FEEDBACK *********************/
    //Set HR LED pin high/off 
    pinMode(HR_LED_PIN, OUTPUT);  digitalWrite(HR_LED_PIN, 1);

   //configure haptic feedback pin
    pinMode(VIBRATE_PIN, OUTPUT);  digitalWrite(VIBRATE_PIN, 0);

}

/********************************************************************************************************/
/************************ LOOP **************************************************************************/
/********************************************************************************************************/

void loop()
{     

  /*************** BUTTON MGMT *****************************************/
  //get capacitive touch button value from ADC
  int buttonValue = analogRead(TOUCH_BUTTON); //no touch 100-400 , touch 500-1023

  /*************** TEST BUTTON, HR LED AND VIBRATION MOTOR *************/
  //touching button turns on green HR LED and vibration motor for one second
  if(buttonValue > 500){
    digitalWrite(VIBRATE_PIN, 1);
    digitalWrite(HR_LED_PIN, 0);
    delay(1000);
    digitalWrite(VIBRATE_PIN, 0);
    digitalWrite(HR_LED_PIN, 1);
    delay(1000);
  }
 
} 

