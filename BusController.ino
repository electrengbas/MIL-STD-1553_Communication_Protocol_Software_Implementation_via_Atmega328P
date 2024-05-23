#include "mil_std_1553.h"
#include <stdio.h>
#include <Wire.h>
#include "RTClib.h"

#define TX_PIN  8  //pin where other side's RX pin is connected
#define RX_PIN  9  //pin where other side's TX pin is connected
#define CONTRAST_PIN 6  //LCD's contrast pin (optimal 20 pwm)
#define BACKGROUND_PIN A1 //LCD's background light pin, give HIGH to turn on

//Address specifications about the RT side
uint32_t RT_ADDRESS = 1;
uint32_t POT_SUB_ADDRESS = 1;
uint32_t THERM_SUB_ADDRESS = 2;
uint32_t MOTOR_SUB_ADDRESS = 3;
uint32_t RPM_SUB_ADDRESS = 4;

//store the values of each sensor and the motor
uint32_t pot_data = 0 ;
uint32_t therm_data = 0;
uint32_t motor_data = 0;
uint32_t rpm_data = 0;

bool demonstration_mode = false;

//Clock initialization and the variables for the time and data
RTC_DS3231 rtc;
char daysOfTheWeek[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
int Day, Month, Year, Secs, Minutes, Hours;
String day_of_week;
String myDate; 
String myTime; 


void setup() 
{
  Serial.begin(115200);
  Serial.print("BC initialize\n");
  mil.setup(TX_PIN, RX_PIN, MAN_2400);

  start_lcd();
  analogWrite(CONTRAST_PIN, 20);
  digitalWrite(BACKGROUND_PIN, HIGH);

  Wire.begin();
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
  }
  //This gets the current time from the pc and loads it to RTC module, uncomment it after setting the time once
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  int timeCounter = 0;
  for(timeCounter = 0; timeCounter < 5; timeCounter++){
    //at the beginning of the running BC, print the current time for 5 seconds
    DateTime now = rtc.now();
    Day = now.day(); 
    Month = now.month(); 
    Year = now.year();
    Secs = now.second(); 
    Hours = now.hour(); 
    Minutes = now.minute(); 
    day_of_week = daysOfTheWeek[now.dayOfTheWeek()]; 

    myDate = myDate +day_of_week+ " "+ Day + "/" + Month + "/" + Year ; 
    myTime = myTime + Hours +":"+ Minutes +":" + Secs ; 
    print_time_lcd(myDate, myTime);
    myDate = ""; 
    myTime = ""; 
    delay(1000);
  }

}

void loop() 
{
  demonstration_mode = digitalRead(A3);

  start_transceive(true , RT_ADDRESS, POT_SUB_ADDRESS, &pot_data, demonstration_mode);//get the data from the potentiometer
  start_transceive(true , RT_ADDRESS, THERM_SUB_ADDRESS, &therm_data, demonstration_mode);//get the data from the temperature sensor

  int avr = (therm_data + pot_data)/2;
  motor_data = (uint32_t) map(avr, 0, 1023, 0, 255);
  start_transceive(false, RT_ADDRESS, MOTOR_SUB_ADDRESS, &motor_data, demonstration_mode);//write the motor speed

  start_transceive(true , RT_ADDRESS, RPM_SUB_ADDRESS, &rpm_data, demonstration_mode);//get the rpm value
  
  double temperature = ((therm_data / 1023.0)*5000)/10.0;

  //if demonstration mode is off, then print the temperature and rpm on the  screen
  if(!demonstration_mode){
    //Serial.print("Temp: ");
    //Serial.println(therm_data);
    //Serial.print("RPM:");
    //Serial.println(rpm_data);
    print_temp_rpm(temperature, rpm_data);
  }
}