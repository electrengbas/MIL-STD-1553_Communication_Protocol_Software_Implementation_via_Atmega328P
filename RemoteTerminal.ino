#include "mil_std_1553.h"

#define TX_PIN 8
#define RX_PIN 9
#define POT_PIN A3
#define THERM_PIN A4
#define MOTOR_PIN 10
#define IN1_PIN A0
#define IN2_PIN 7
#define CONTRAST_PIN 6  //LCD's contrast pin (optimal 20 pwm)
#define BACKGROUND_PIN A5 //LCD's background light pin, give HIGH to turn on

//Address specifications about the RT side
uint32_t RT_ADDRESS = 1;
uint32_t POT_SUB_ADDRESS = 1;
uint32_t THERM_SUB_ADDRESS = 2;
uint32_t MOTOR_SUB_ADDRESS = 3;
uint32_t RPM_SUB_ADDRESS = 4;
uint32_t sub_address;
uint32_t temp_data;

//store the values of each sensor and the motor
uint32_t pot_data = 0;
uint32_t therm_data = 0;
uint32_t motor_data = 0;
uint32_t rpm_data = 0;

//rpm related variables and constants
#define ENCODEROUTPUT 360
const int HALLSEN_A = 3; // Hall sensor A connected to pin 3 (external interrupt)
volatile long encoderValue = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
int interval = 1000;

//Servo myServo;
bool demonstration_mode = false;

void setup(){
  mil.setup(TX_PIN, RX_PIN, MAN_2400);

  start_lcd();
  analogWrite(CONTRAST_PIN, 20);
  digitalWrite(BACKGROUND_PIN, HIGH);

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  encoderValue = 0;
  previousMillis = millis();
  attachInterrupt(digitalPinToInterrupt(HALLSEN_A), updateEncoder, RISING);

//end of set up
}

void loop(){
  demonstration_mode = digitalRead(13);

  currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    // RPM = (total encoder pulse in 1s / motor encoder output) x 60s
    rpm_data = (float)(encoderValue * 60 * 1000 / ENCODEROUTPUT / (currentMillis - previousMillis));
    previousMillis = currentMillis;
    encoderValue = 0;
  }

  int ret = wait_for_transceive(RT_ADDRESS, &sub_address, &motor_data, demonstration_mode);
  if(ret == 1){//this means the transmit command word has arrived. so the data from the required sensor should be read and sent to bc
    if(sub_address == POT_SUB_ADDRESS){
      pot_data = analogRead(POT_PIN);
      answer_to_bc(RT_ADDRESS, pot_data, demonstration_mode);
    } else if(sub_address == THERM_SUB_ADDRESS){
      therm_data = analogRead(THERM_PIN);
      answer_to_bc(RT_ADDRESS, therm_data, demonstration_mode);
    } else if(sub_address == RPM_SUB_ADDRESS){
      rpm_data = 50;
      answer_to_bc(RT_ADDRESS, rpm_data, demonstration_mode);
    }  
  } else if(ret == 0){//this means the receive command word has arrived and the motor_data is filled with the incoming data
    analogWrite(MOTOR_PIN, motor_data);
  }
}

void updateEncoder(){//each time hall sensor generates 1, increment encoderValue by one
  encoderValue++;
}