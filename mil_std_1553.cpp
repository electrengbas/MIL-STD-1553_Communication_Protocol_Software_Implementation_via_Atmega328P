#include "mil_std_1553.h"
#include "Arduino.h"
#include <string.h>
#include <stdio.h>

static int8_t RxPin = 255;

volatile static int16_t rx_sample = 0;
volatile static int16_t rx_last_sample = 0;
volatile static uint8_t rx_count = 0;
volatile static uint8_t rx_sync_count = 0;
volatile static uint8_t rx_mode = RX_MODE_IDLE;

static uint64_t rx_manBits = 0; //the received Mil_std_1553 40 bits
static uint8_t rx_numMB = 0; //the number of received Mil_std_1553 bits
static uint8_t rx_curWord = 0; //current received word number

static uint8_t rx_maxWords = 2;
static uint32_t rx_default_data[2];
static uint32_t* rx_data = rx_default_data;

///////---------------------------------------------------GENERAL FUNCTIONS START----------------------------------------------------///////
#include <LiquidCrystal.h>  
int rs = 11, en = 12, d4 = 2, d5 = 3, d6 = 5, d7 = 4;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void start_lcd(){
  lcd.begin(16, 2);
}

Mil_std_1553::Mil_std_1553(){ //constructor

}

void Mil_std_1553::setTxPin(uint8_t pin){
  TxPin = pin; // user sets the digital pin as output
  pinMode(TxPin, OUTPUT); 
}

void Mil_std_1553::setRxPin(uint8_t pin){
  ::RxPin = pin; // user sets the digital pin as output
  pinMode(::RxPin, INPUT); 
}

void Mil_std_1553::setupTransmit(uint8_t pin, uint8_t SF){
  setTxPin(pin);
  speedFactor = SF;
  uint16_t compensationFactor = 4; 
  delay1 = (HALF_BIT_INTERVAL >> speedFactor) - compensationFactor;
  delay2 = (HALF_BIT_INTERVAL >> speedFactor) - 2;
}

void Mil_std_1553::setupReceive(uint8_t pin, uint8_t SF){
  setRxPin(pin);
  ::MANRX_SetupReceive(SF);
}

void Mil_std_1553::setup(uint8_t Tpin, uint8_t Rpin, uint8_t SF){
  setupTransmit(Tpin, SF);
  setupReceive(Rpin, SF);
}

void Mil_std_1553::transmitArray(uint32_t numWords, uint32_t *data){
#if SYNC_BIT_VALUE
  for( int8_t i = 0; i < SYNC_PULSE_DEF; i++){ //send capture pulses
    sendOne(); //end of capture pulses
  }
  sendZero(); //start data pulse
#else
  for( int8_t i = 0; i < SYNC_PULSE_DEF; i++){ //send capture pulses
    sendZero(); //end of capture pulses
  }
  sendOne(); //start data pulse
#endif
 
  // Send the user data
  for (uint8_t i = 0; i < numWords; i++){
    uint32_t mask = 0x01; //mask to send bits
    uint32_t d = data[i];// ^ DECOUPLING_MASK;
    for (uint8_t j = 0; j < 20; j++){
      if ((d & mask) == 0)
        sendZero();
      else
        sendOne();
      mask <<= 1; //get next bit
    }//end of byte
  }//end of data

  // Send 3 terminatings 0's to correctly terminate the previous bit and to turn the transmitter off
#if SYNC_BIT_VALUE
  sendOne();
  sendOne();
  sendOne();
#else
  sendZero();
  sendZero();
  sendZero();
#endif
}

void Mil_std_1553::sendZero(void){
  delayMicroseconds(delay1);
  digitalWrite(TxPin, LOW);

  delayMicroseconds(delay2);
  digitalWrite(TxPin, HIGH);
}

void Mil_std_1553::sendOne(void){
  delayMicroseconds(delay1);
  digitalWrite(TxPin, HIGH);

  delayMicroseconds(delay2);
  digitalWrite(TxPin, LOW);
}

void Mil_std_1553::beginReceiveArray(uint8_t maxWords, uint32_t *data){
  ::MANRX_BeginReceiveWords(maxWords, data);
}

uint8_t Mil_std_1553::receiveComplete(void){
  return ::MANRX_ReceiveComplete();
}

void MANRX_SetupReceive(uint8_t speedFactor){
  pinMode(RxPin, INPUT);
  TCCR2A = _BV(WGM21); // reset counter on match
  TCCR2B = _BV(CS22); // 1/64 prescaler
  OCR2A = (128 >> speedFactor) - 1; 
  TIMSK2 = _BV(OCIE2A); // turn on interrupt
  TCNT2 = 0; // set counter to 0
}

void MANRX_BeginReceiveWords(uint8_t maxWords, uint32_t *data){
  rx_maxWords = maxWords;
  rx_data = data;
  rx_mode = RX_MODE_PRE;
}

uint8_t MANRX_ReceiveComplete(void){
  return (rx_mode == RX_MODE_MSG);
}

void MANRX_SetRxPin(uint8_t pin){
  RxPin = pin;
  pinMode(RxPin, INPUT);
}//end of set transmit pin

void AddManBit(uint64_t *manBits, uint8_t *numMB, uint8_t *curByte, uint32_t *data, uint8_t bit){
  *manBits <<= 1;
  *manBits |= bit;
  (*numMB)++;
  if (*numMB == 40){
    uint32_t newData = 0;
    for (int8_t i = 0; i < 20; i++){
      // ManBits holds 40 bits of Mil_std_1553 data
      // 1 = HI,LO
      // 0 = LO,HI
      // We can decode each bit by looking at the top bit of each pair.
      newData <<= 1;
      uint8_t temp = (*manBits & 2)?1:0; 
      newData |= temp;// store the bit
      *manBits = *manBits >> 2; //get next data bit
    }

    data[*curByte] = newData;
    (*curByte)++;

    if( (*curByte) == 1){
      rx_maxWords = data[0];
    }
    
    *numMB = 0;
  } 
}

ISR(TIMER2_COMPA_vect){
  if (rx_mode < RX_MODE_MSG){ //receiving something
    rx_count += 8;
    // sample twice, only the same means a change.
    static uint8_t rx_sample_0=0;
    static uint8_t rx_sample_1=0;
    rx_sample_1 = digitalRead(RxPin);
    if( rx_sample_1 == rx_sample_0 ){
      rx_sample = rx_sample_1;
    }
    rx_sample_0 = rx_sample_1;

    //check sample transition
    uint8_t transition = (rx_sample != rx_last_sample);
  
    if (rx_mode == RX_MODE_PRE){
      // Wait for first transition to HIGH
      if (transition && (rx_sample == 1)){
        rx_count = 0;
        rx_sync_count = 0;
        rx_mode = RX_MODE_SYNC;
      }
    }
    else if (rx_mode == RX_MODE_SYNC){
      // Initial sync block
      if (transition){
        if( ( (rx_sync_count < (SYNC_PULSE_MIN * 2) )  || (rx_last_sample == 0)  ) && ( (rx_count < MinCount) || (rx_count > MaxCount))){
          // First 20 bits and all 1 bits are expected to be regular
          // Transition was too slow/fast
          rx_mode = RX_MODE_PRE;
        }
        else if((rx_last_sample == 1) && ((rx_count < MinCount) || (rx_count > MaxLongCount))){
          // 0 bits after the 20th bit are allowed to be a double bit
          // Transition was too slow/fast
          rx_mode = RX_MODE_PRE;
        }
        else{
          rx_sync_count++;
          
          if((rx_last_sample == 1) && (rx_sync_count >= (SYNC_PULSE_MIN * 2) ) && (rx_count >= MinLongCount)){
            // We have seen at least 10 regular transitions
            // Lock sequence ends with unencoded bits 01
            // This is encoded and TX as HI,LO,LO,HI
            // We have seen a long low - we are now locked!
            rx_mode    = RX_MODE_DATA;
            rx_manBits = 0;
            rx_numMB   = 0;
            rx_curWord = 0;
          }
          else if (rx_sync_count >= (SYNC_PULSE_MAX * 2) ){
            rx_mode = RX_MODE_PRE;
          }
          rx_count = 0;
        }
      }
    }
    else if (rx_mode == RX_MODE_DATA){
      // Receive data
      if (transition){
        if((rx_count < MinCount) ||
           (rx_count > MaxLongCount)){
          // wrong signal lenght, discard the message
          rx_mode = RX_MODE_PRE;
        }
        else{
          if(rx_count >= MinLongCount) {// was the previous bit a double bit?
            AddManBit(&rx_manBits, &rx_numMB, &rx_curWord, rx_data, rx_last_sample);
          }
          if ((rx_sample == 0) && (rx_curWord >= rx_maxWords)){
            rx_mode = RX_MODE_MSG;
          }
          else{
            // Add the current bit
            AddManBit(&rx_manBits, &rx_numMB, &rx_curWord, rx_data, rx_sample);
            rx_count = 0;
          }
        }
      }
    }
    
    // Get ready for next loop
    rx_last_sample = rx_sample;
  }
}
Mil_std_1553 mil;

const uint32_t COMMAND_SYNC = 6;
const uint32_t STATUS_SYNC = 6;
const uint32_t DATA_SYNC = 1;

uint32_t pilot_words[3] = {0};

int calculate_parity(uint32_t pilot_word){
    uint32_t i = 0;
    uint32_t mask = 1;
    int parity = 0;
    uint32_t number_of_ones = 0;
    for(i = 19; i > 0 ;i--){
      boolean temp = pilot_word & (mask << i);
      if(temp)
        number_of_ones++;
    }
    if(number_of_ones % 2 == 0)
      parity = 0;
    else
      parity = 1;
    return parity;
}

void generate_data_word(uint32_t* data_word_ptr, uint32_t data){
  uint32_t pilot_word = 0;
  pilot_word = pilot_word | (DATA_SYNC << 17);
  pilot_word = pilot_word | ((data & 65535) << 1);
  int parity = calculate_parity(pilot_word);
  pilot_word = pilot_word | parity;
  *data_word_ptr = pilot_word;
}

int parse_data_word(uint32_t pilot_word, uint32_t *data_ptr){
  int ret = 0;
  uint32_t sync = pilot_word >> 17;
  uint32_t temp_data = (pilot_word >> 1) & 65535;
  int parity_received = pilot_word & 1;
  int parity_calculated = calculate_parity(pilot_word);

  if(sync == DATA_SYNC &&  parity_received == parity_calculated){
    ret = 0;
    *data_ptr = temp_data;
  }
  else
    ret = -1;

    return ret;
}

void print_to_lcd(uint32_t word, char* second_line){
  uint8_t mask = 1;//the least significant 
  uint32_t temp = 0;
  lcd.clear();
  for(int i = 0; i < 20; i++){
    lcd.setCursor(i, 0);
    temp = (word >> 19 - i) & mask;
    lcd.print(temp);
  }

  lcd.setCursor(0, 1);                     
  lcd.print(second_line);
  for(int i = 0; i < 5; i++){
    lcd.scrollDisplayLeft();
      delay(250);
  }
  for(int i = 0; i < 5; i++){
    lcd.scrollDisplayRight();
    delay(250);
  }
  delay(1000);
  lcd.clear();
}

void serial_print_word(uint32_t word, char* second_line){
  uint8_t mask = 1;//the least significant 
  uint32_t temp = 0;
  for(int i = 0; i < 20; i++){
    temp = (word >> 19 - i) & mask;
    Serial.print(temp);
  }        
  Serial.println();   
  Serial.println(second_line);
  delay(2500);
}
///////---------------------------------------------------GENERAL FUNCTIONS START----------------------------------------------------///////


///////------------------------------------------------------BC FUNCTIONS START------------------------------------------------------///////
void print_temp_rpm(double temp, uint32_t rpm){
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.setCursor(0,7);
  lcd.print(temp);

  lcd.setCursor(0, 1);
  lcd.print("RPM: ");
  lcd.setCursor(0, 6);
  lcd.print(rpm);
  delay(25);
}

void print_time_lcd(String first_line, String second_line){
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(first_line);

  lcd.setCursor(0, 1);
  lcd.print(second_line);
}

void generate_command_word(uint32_t* command_word_ptr, uint32_t rt_address, int mode, uint32_t sub_address){
  uint32_t pilot_word = 0;
  pilot_word = pilot_word | (COMMAND_SYNC << 17);
  pilot_word = pilot_word | ((rt_address & 31) << 12);
  pilot_word = pilot_word | (mode << 11);
  pilot_word = pilot_word | ((sub_address & 31) << 6);
  //mode code is 0
  int parity = calculate_parity(pilot_word);
  pilot_word = pilot_word | parity;
  *command_word_ptr = pilot_word;
}

int check_status_word(uint32_t pilot_word, uint32_t rt_address){
  int ret = 0;
  uint32_t sync = pilot_word >> 17;
  uint32_t rt = (pilot_word >> 12) & 31;
  int message_error = (pilot_word >> 11) & 1;
  uint32_t instrumentation = (pilot_word >> 10) & 1;
  int parity_received = pilot_word & 1;
  int parity_calculated = calculate_parity(pilot_word);
  if(sync == STATUS_SYNC && rt == rt_address && message_error == 0 && instrumentation == 1 && parity_received == parity_calculated)
    ret = 0;
  else
    ret = -1;

  return ret;
}

int start_transceive(bool mode, uint32_t rt_address, uint32_t sub_address, uint32_t *data_ptr, bool demonstration_mode){
  //mode = true --> rt to bc (transmit command word), mode = false --> bc to rt (receive command word)
  int rt_of_command_word = (int) mode;
  int ret;
  char buffer[32] = {0};
  if(demonstration_mode){
    //Serial.println("start_trancieve\niscalled\n");
    lcd.clear();
    lcd.setCursor(0, 0);                          
    lcd.print("start_transceive");  
    lcd.setCursor(0, 1);        
    lcd.print("is called");  
    delay(1000);
  }

  if (mode){//RT TO BC (transmit command will be sent)
    while(true){
      memset(pilot_words, 0, 3);
      pilot_words[0] = 2;
      generate_command_word(&pilot_words[1], rt_address, rt_of_command_word, sub_address);
      delay(25);

      if(demonstration_mode){
        buffer[32] = {0};
        sprintf(buffer, "Trnsmt RT:%lu Sub:%lu", rt_address, sub_address);
        //serial_print_word(pilot_words[1], buffer);
        print_to_lcd(pilot_words[1], buffer);
        
        lcd.setCursor(0, 0);                        
        lcd.print("Waiting for RT");
        //Serial.println("Waiting for RT");
      }
  
      mil.transmitArray(pilot_words[0], pilot_words);
      memset(pilot_words, 0, 3);
      mil.beginReceiveArray(3, pilot_words);//wait for the status and data word
      while(!mil.receiveComplete()){ }

      ret = check_status_word(pilot_words[1], rt_address);//According to the return value of this function within the while loop that continues to repeat in case of an error, we get rid of the while loop
      if(demonstration_mode){
        buffer[32] = {0};
        sprintf(buffer, "Status Received:%s", ret==0?"ok":"not");
        //serial_print_word(pilot_words[1], buffer);
        print_to_lcd(pilot_words[1], buffer);
      }

      if(ret == -1){//0 represents a normal state, while -1 indicates an error state.
        continue;
      }

      ret = parse_data_word(pilot_words[2], data_ptr);//According to the return value of this function within the while loop that continues to repeat in case of an error, we get rid of the while loop
      if(demonstration_mode){
        buffer[32] = {0};
        sprintf(buffer, "Data Received:%lu", *data_ptr);
        //serial_print_word(pilot_words[2], buffer);
        print_to_lcd(pilot_words[2], buffer);
      }
      if(ret == 0){//0 represents a normal state, while -1 indicates an error state.
        break;
      }
    }//end while (RT TO BC)
  } else{//BC TO RT (receive command anda data word will be sent)
    while(true){
      memset(pilot_words, 0, 3);
      pilot_words[0] = 3;
      generate_command_word(&pilot_words[1], rt_address, rt_of_command_word, sub_address);
      generate_data_word(&pilot_words[2], *data_ptr);
      delay(25);

      if(demonstration_mode){
        buffer[32] = {0};
        sprintf(buffer, "Receive RT:%lu Sub:%lu", rt_address, sub_address);
        //serial_print_word(pilot_words[1], buffer);
        print_to_lcd(pilot_words[1], buffer);

        buffer[32] = {0};
        uint32_t myVar = 0;
        memcpy(&myVar, data_ptr, 2);
        sprintf(buffer, "Data Sent:%lu", myVar);
        //serial_print_word(pilot_words[2], buffer);
        print_to_lcd(pilot_words[2], buffer);

        //Serial.println("Waiting for RT");
        lcd.clear();
        lcd.print("Waiting for RT");
      }

      mil.transmitArray(pilot_words[0], pilot_words);

      memset(pilot_words, 0, 3);
      mil.beginReceiveArray(2, pilot_words); //wait for the status word
      while(!mil.receiveComplete()){ }
      ret = check_status_word(pilot_words[1], rt_address);
      if(demonstration_mode){
        buffer[32] = {0};
        sprintf(buffer, "Status Received:%s", ret==0?"ok":"not");
        //serial_print_word(pilot_words[1], buffer);
        print_to_lcd(pilot_words[1], buffer);
      }

      if(ret == 0){//0 represents a normal state, while -1 indicates an error state.
        break;
        }
    }
  }
  return ret;
}
///////-------------------------------------------------------BC FUNCTIONS END-------------------------------------------------------///////


///////------------------------------------------------------RT FUNCTIONS START------------------------------------------------------///////
void generate_status_word(uint32_t *status_word_ptr, uint32_t rt_address, int message_error){
  uint32_t pilot_word = 0;
  pilot_word = pilot_word | (STATUS_SYNC << 17);
  pilot_word = pilot_word | ((rt_address & 31) << 12);
  pilot_word = pilot_word | (message_error << 11);
  pilot_word = pilot_word | (1 << 10); //instrumentation bit
  //the remaining is 0
  int parity = calculate_parity(pilot_word);
  pilot_word = pilot_word | parity;
  *status_word_ptr = pilot_word;
}

int recover_command_word(uint32_t pilot_word, uint32_t rt_address, uint32_t *sub_address){
  int ret = 0;
  uint32_t sync = pilot_word >> 17;
  uint32_t rt = (pilot_word >> 12) & 31;
  int mode = (pilot_word >> 11) & 1;
  *sub_address = (pilot_word >> 6) & 31; 
  int parity_received = pilot_word & 1;
  int parity_calculated = calculate_parity(pilot_word);
  if(sync == COMMAND_SYNC && rt == rt_address && parity_received == parity_calculated)
    ret = mode;
  else
    ret = -1;

  return ret;
}

int wait_for_transceive(uint32_t rt_address, uint32_t *sub_address, uint32_t *data_ptr, bool demonstration_mode){
  int ret = 0;
  char buffer[32] = {0};
  memset(pilot_words, 0, 3);

  if(demonstration_mode){
    //Serial.println("Waiting for BC");
    lcd.clear();
    lcd.print("Waiting for RT");
  }

  mil.beginReceiveArray(3, pilot_words); //wait for the command word (or command and data word)
  while(!mil.receiveComplete()){ }
  //ret = 1 --> rt to bc (transmit command word), ret = 0 --> bc to rt (receive command word), ret = -1 error
  ret = recover_command_word(pilot_words[1], rt_address, sub_address);

  if(pilot_words[0] == 2 && ret == 1){//rt to bc (only transmit command is received)
    if(demonstration_mode){
      sprintf(buffer, "Trnsmt RT:%lu Sub:%lu", rt_address, *sub_address);
      //serial_print_word(pilot_words[1], buffer);
      print_to_lcd(pilot_words[1], buffer);
    }

    //Here nothing will be done and thus 1 is returned to the main. After that, main will call answer_to_bc function
  }
  else if(pilot_words[0] == 3 && ret == 0){//bc to rt (receive command and data word are received)
    ret = parse_data_word(pilot_words[2], data_ptr);
    
    if(demonstration_mode){
      sprintf(buffer, "Receive RT:%lu Sub:%lu", rt_address, *sub_address);
      //serial_print_word(pilot_words[1], buffer);
      print_to_lcd(pilot_words[1], buffer);

      buffer[32] = {0};
      uint32_t myVar = 0;
      memcpy(&myVar, data_ptr, 2);
      sprintf(buffer, "Data Received:%lu", myVar);
      //serial_print_word(pilot_words[2], buffer);
      print_to_lcd(pilot_words[2], buffer);
    }

    memset(pilot_words, 0, 3);
    pilot_words[0] = 2;
    generate_status_word(&pilot_words[1], rt_address, ret== -1?1:0);
    delay(25);

    if(demonstration_mode){
      buffer[32] = {0};
      sprintf(buffer, "Status Sent:%s", ret==0?"ok":"not");
      //serial_print_word(pilot_words[1], buffer);
      print_to_lcd(pilot_words[1], buffer);
    }
    
    mil.transmitArray(pilot_words[0], pilot_words);
  }
  else
    ret = -1;

  return ret;
}

int answer_to_bc(uint32_t rt_address, uint32_t data, bool demonstration_mode){//send status and data word
  memset(pilot_words, 0, 3);
  pilot_words[0] = 3;
  generate_status_word(&pilot_words[1], rt_address, 0);
  generate_data_word(&pilot_words[2], data);
  delay(25);

  if(demonstration_mode){
    char buffer[32] = {0};
    sprintf(buffer, "Status Sent:ok");
    //serial_print_word(pilot_words[1], buffer);
    print_to_lcd(pilot_words[1], buffer);
  
    buffer[32] = {0};
    uint32_t myVar = 0;
    memcpy(&myVar, &data, 2);
    sprintf(buffer, "Data Sent:%lu", myVar);
    //serial_print_word(pilot_words[2], buffer);
    print_to_lcd(pilot_words[2], buffer);
  }
  
  mil.transmitArray(pilot_words[0], pilot_words);
}
///////-------------------------------------------------------RT FUNCTIONS END-------------------------------------------------------///////