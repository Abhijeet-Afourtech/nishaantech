/*
   NishaanTech 
   MCU + DHT + EEPROM I2C interface.

   Wake Up MCU for getting Humidity & Temperature ----- ALarm 1
   Wake Up RPi for doing its task ------   Alarm 2

   Testes on 28th Aug:
      RPI & MCU COMMUNICATION WORKING OK WITH HARDWARE SERIAL CONNECTION
      time, dht, ok(ack) & sleep(RPI off) signals.

   Tested On 6th Sep :
      5 min wake up for RPI 20 min interval for DHT storeage
      will sleep irrespective of RPI FB after 3 minutes.
      At first boot it will not wake up RPi 1st, will collect dht data & then will wake up rpi.
*/

#include <DS3231_Simple.h>
#include <avr/sleep.h>
#include "Adafruit_Si7021.h"
#include <TimeLib.h>
#include <Eeprom24C32_64.h>
#include <DS3232RTC.h>
#include <Time.h>
#include <Wire.h>


#define EEPROM_ADDRESS 0x57
#define MAX 100
#define rpiWake 9

Adafruit_Si7021 sensor = Adafruit_Si7021();
DS3231_Simple Clock;
static Eeprom24C32_64 eeprom(EEPROM_ADDRESS);

/* Variable Declaration & Initialization */

int wakePin = 3;                 // pin used for waking up
int fbPin = 2;                   // pin used for feedback      
volatile uint8_t mn;
volatile uint8_t dhtmn;
int c;
volatile uint8_t previous_mn;
volatile int i = 0;
uint8_t temp;
uint8_t hum;
volatile int j = 0;
unsigned long epoch;
int len;
char store[18];
word address = 0;
String inputString = "";
boolean stringComplete = false;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const int sleepInterval = 180000;


/* Interrupt Subroutine Function */
void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
  digitalWrite(outPin, HIGH);
  //Serial.println("Inside Interrupt....");
  //digitalWrite(9, LOW);
  //wake = 1;
}

void setup() {
  pinMode(wakePin, INPUT);
  pinMode(fbPin, INPUT);
  pinMode(rpiWake, OUTPUT);  // RPI WAKE/ POWER PIN
  //digitalWrite(rpiWake, LOW);
  digitalWrite(rpiWake, HIGH);
  Serial.begin(9600);
  Serial.println();
  Serial.println("START....");

  time_t t;
  tmElements_t tm;

  tm.Year = 2017 - 1970;
  tm.Month = 8;
  tm.Day = 29;
  tm.Hour = 16;
  tm.Minute = 14;
  tm.Second = 40;
  t = makeTime(tm);
  RTC.set(t);        //use the time_t value to ensure correct weekday is set
  setTime(t);

  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  Serial.println(now());

  // Initiliaze EEPROM library.
  eeprom.initialize();
  sensor.begin();
  Clock.begin();

  // We will set 2 alarms, the first alarm will fire at the 30th second of every minute
  // and the second alarm will fire every minute (at the 0th second)

  // First we will disable any existing alarms
  Clock.disableAlarms();
  /*
      // Get an initialized timestamp
      DateTime MyTimestamp = Clock.read();

      Serial.print("Current MIN  ");
      Serial.println(MyTimestamp.Minute);
      mn = MyTimestamp.Minute + 2;
      if (mn > 59) {
       mn = mn%10;
      }
      MyTimestamp.Minute = mn;

      Serial.print("Wake at This MIN ");
      Serial.println(mn);

      Clock.setAlarm(MyTimestamp, DS3231_Simple::ALARM_MATCH_MINUTE);

      Serial.println("Timer: Entering Sleep mode");
      attachInterrupt(1, wakeUpNow, LOW);

      delay(2000);

      sleepNow();     // sleep function called here
  */


  Serial.println("Waiting for alarms...");
  setAlarmDht();

  delay(500);

  setAlarm();

  //attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
  //wakeUpNow when pin 3 gets LOW

  sleepNow();
}


/* MCU Sleep Function */
void sleepNow()         // here we put the arduino to sleep
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     there is a list of sleep modes which explains which clocks and
     wake up sources are available in which sleep mode.

     In the avr/sleep.h file, the call names of these sleep modes are to be found:

     The 5 different modes are:
         SLEEP_MODE_IDLE         -the least power savings
         SLEEP_MODE_ADC
         SLEEP_MODE_PWR_SAVE
         SLEEP_MODE_STANDBY
         SLEEP_MODE_PWR_DOWN     -the most power savings

     For now, we want as much power savings as possible, so we
     choose the according
     sleep mode: SLEEP_MODE_PWR_DOWN

  */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here

  sleep_enable();          	// enables the sleep bit in the mcucr register
							// so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
     accidentally pushed interrupt button doesn't interrupt
     our running program. if you want to be able to run
     interrupt code besides the sleep function, place it in
     setup() for example.

     In the function call attachInterrupt(A, B, C)
     A   can be either 0 or 1 for interrupts on pin 2 or 3.

     B   Name of a function you want to execute at interrupt for A.

     C   Trigger mode of the interrupt pin. can be:
                 LOW        a low level triggers
                 CHANGE     a change in level triggers
                 RISING     a rising edge of a level triggers
                 FALLING    a falling edge of a level triggers

     In all but the IDLE sleep modes only LOW can be used.
  */

  attachInterrupt(1, wakeUpNow, LOW); // use interrupt 1 (pin 3) and run function
  // wakeUpNow when pin 3 gets LOW

  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(1);      // disables interrupt 1 on pin 3 so the
  // wakeUpNow code will not be executed
  // during normal running time.

}

/* Set Alarm for Temp and Humidity storage */
void setAlarmDht() {  
  // Get an initialized timestamp
  DateTime MyTimestamp = Clock.read();

  /*Serial.print("Current MIN  ");
    Serial.println(MyTimestamp.Minute);
    dhtmn = MyTimestamp.Minute + 1;
    if (dhtmn > 59) {
    dhtmn = dhtmn % 10;
    }
    MyTimestamp.Minute = dhtmn;
    MyTimestamp.Second = 05;

    Serial.print("Wake at This MIN ");
    Serial.println(dhtmn);
    */

  Serial.print("Current MIN  ");
  Serial.println(MyTimestamp.Second);
  //  dhtmn = MyTimestamp.Second + 5;
  dhtmn = MyTimestamp.Second + 20;
  if (dhtmn > 59) {
    //    dhtmn = dhtmn % 10;
    dhtmn = dhtmn % 20;
  }
  MyTimestamp.Second = dhtmn;
  //  MyTimestamp.Second = 05;

  Serial.print("Wake at This MIN ");
  Serial.println(dhtmn);


  //Clock.setAlarm(MyTimestamp, DS3231_Simple::ALARM_MATCH_SECOND_MINUTE);
  Clock.setAlarm(MyTimestamp, DS3231_Simple::ALARM_MATCH_SECOND);

  Serial.println("Timer: Entering Sleep mode");
  //  attachInterrupt(1, wakeUpNow, LOW);

  delay(2000);

  sleepNow();     // sleep function called here
}


/* Set Alarm for next RPi Wake Up */
void setAlarm() {

  // Get an initialized timestamp
  DateTime MyTimestamp = Clock.read();

  Serial.print("Current MIN  ");
  Serial.println(MyTimestamp.Minute);
  //    mn = MyTimestamp.Minute + 1;
  mn = MyTimestamp.Minute + 5;
  if (mn > 59) {
    mn = mn % 10;
  }
  MyTimestamp.Minute = mn;

  Serial.print("Wake at This MIN ");
  Serial.println(mn);

  /* Serial.print("Current MIN  ");
    Serial.println(MyTimestamp.Second);
    mn = MyTimestamp.Second + 10;
    if (mn > 59) {
     mn = mn % 10;
    }
    MyTimestamp.Second = mn;

    Serial.print("Wake at This MIN ");
    Serial.println(mn);
    */

  Clock.setAlarm(MyTimestamp, DS3231_Simple::ALARM_MATCH_MINUTE);
  //Clock.setAlarm(MyTimestamp, DS3231_Simple::ALARM_MATCH_SECOND);

  Serial.println("Timer: Entering Sleep mode");
  //attachInterrupt(1, wakeUpNow, LOW);

  delay(2000);

  sleepNow();     // sleep function called here
}

/* Time Setting Function */
void timeDateSet() {
  time_t t;
  tmElements_t tm;

  tm.Year = 2017 - 1970;
  tm.Month = 8;
  tm.Day = 21;
  tm.Hour = 16;
  tm.Minute = 27;
  tm.Second = 10;
  t = makeTime(tm);
  RTC.set(t);        //use the time_t value to ensure correct weekday is set
  setTime(t);

  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
  Serial.println(now());
}



void loop()
{
  Serial.println(now());
  delay(300);
  
  // To check the alarms we just ask the clock
  uint8_t AlarmsFired = Clock.checkAlarms();

  // RPI Feedback on GPIO pin to for sleep

  /*if (digitalRead(fbPin) == LOW) {
    setAlarm();
    delay(1000);
    //    digitalWrite(rpiWake, HIGH);
    //    digitalWrite(outPin, LOW);
    sleepNow();
  }*/


  // Then can check if either alarm is fired (there are 2 alarms possible in the chip)
  // by using a "bitwise and"

  if (AlarmsFired & 1)
  {
    Clock.printTo(Serial);
    Serial.println(": First alarm has fired! \n Read DHT....");
    temp = sensor.readTemperature();
    hum = sensor.readHumidity();
    epoch = now();
    sprintf(store, "%d,%d,%ld;", temp, hum, epoch);
    Serial.println(store);
    String stringStore = String(store);
    len = stringStore.length();

    Serial.println("ADDRESS :");
    Serial.println(address);

    for (int i = 0; i < len; i++) {
      eeprom.writeByte(address, store[i]);
      delay(10);
      address++;
    }

    Serial.println("ADDRESS :");
    Serial.println(address);
    setAlarmDht();
    digitalWrite(outPin, LOW);
  }

  if (AlarmsFired & 2)
  {
    static char sendDht[544];
    Clock.printTo(Serial);
    Serial.println(": Second alarm has fired! Wake Up RPI");
    Serial.println("Disable DHT ALARM.");
    Clock.disableAlarms();
    Serial.println("Waiting for Input From RPI...................");
    digitalWrite(rpiWake, LOW);  // turn ON RPi
    delay(200);
    previousMillis = millis();

    while (true) {
      delay(200);
      currentMillis = millis();

      inputString.remove(0);
      while (Serial.available() != 0) {
        // get the new byte:
        char inChar = (char)Serial.read();
        //    Serial.println(inChar);
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a ":" , set a flag
        if (inChar == ':') {
          //Serial.println("Complete.......");
          stringComplete = true;
          break;
        }
      }

      Serial.println(inputString);

      if (String(inputString) == "time:") {
        //Serial.println("TIME QUERY FROM RPi....");
        Serial.println(now());
        //inputString.remove(0);
      }

      if (String(inputString) == "size:") {
        Serial.println(address);
      }

      if (String(inputString) == "dht:") {
        memset(sendDht, 0, 544);
        //Serial.println("DHT data QUERY FROM RPi...");
        //Serial.println(address);
        //eeprom.readBytes(0, 300, dhtData);
        delay(100);
        for (int i = 0; i < address; i++) {
          //sendDht += dhtData[i];
          sendDht[i] = eeprom.readByte(i);
        }
        //String sendDht = String(dhtData);
        //Serial.println(dhtData);
        Serial.println(sendDht);
        delay(100);
        //inputString.remove(0);
      }

      if (String(inputString) == "ok:") {
        //address = 0;
        Serial.println("data received ack FROM RPi.");
        //inputString.remove(0);
      }

      if (String(inputString) == "sleep:") {
        address = 0;
        Serial.println("Power OFF QUERY FROM RPi.");
        //Serial.println("DONE Power OFF RPi.");
        delay(10);
        delay(2000);
        //delay(8000);
        digitalWrite(rpiWake, HIGH);
        delay(10);
        break;
        //inputString.remove(0);
      }

      if (currentMillis - previousMillis > sleepInterval) {
        address = 0;
        Serial.println("No Response from RPi...");
        //Serial.println("DONE Power OFF RPi.");
        delay(10);
        delay(2000);
        //delay(8000);
        digitalWrite(rpiWake, HIGH);
        delay(10);
        break;
      }
    }

    delay(100);

    //Serial.println("DONE WITH MCU RPI COMM SET DHT & NEXT RPI WAKE ALARM.....");
    setAlarmDht();
    delay(100);
    setAlarm();
    digitalWrite(outPin, LOW);
    //Serial.println("OUT OF 2nd ALARM  LOOP......");
    //Serial.println("END OF WHILE TRUE..........");
  }

} // Loop End


