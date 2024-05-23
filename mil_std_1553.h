#ifndef MIL_STD_1553_h
#define MIL_STD_1553_h

#include "Arduino.h"

//timer scaling factors for different transmission speeds
#define MAN_300 0
#define MAN_600 1
#define MAN_1200 2
#define MAN_2400 3
#define MAN_4800 4
#define MAN_9600 5
#define MAN_19200 6
#define MAN_38400 7

#define     SYNC_PULSE_MIN  1
#define     SYNC_PULSE_DEF  3
#define     SYNC_PULSE_MAX  5

#define     SYNC_BIT_VALUE      0

//setup timing for receiver
#define MinCount        33  //pulse lower count limit on capture
#define MaxCount        65  //pulse higher count limit on capture
#define MinLongCount    66  //pulse lower count on double pulse
#define MaxLongCount    129 //pulse higher count on double pulse

//setup timing for transmitter
#define HALF_BIT_INTERVAL 3072 //(=48 * 1024 * 1000000 / 16000000Hz) microseconds for speed factor 0 (300baud)

#define RX_MODE_PRE 0
#define RX_MODE_SYNC 1
#define RX_MODE_DATA 2
#define RX_MODE_MSG 3
#define RX_MODE_IDLE 4

class Mil_std_1553
{
  public:
    Mil_std_1553(); //the constructor
    void setTxPin(uint8_t pin); //set the arduino digital pin for transmit. 
    void setRxPin(uint8_t pin); //set the arduino digital pin for receive.
    
    void setupTransmit(uint8_t pin, uint8_t SF = MAN_1200); //set up transmission
    void setupReceive(uint8_t pin, uint8_t SF = MAN_1200); //set up receiver
    void setup(uint8_t Tpin, uint8_t Rpin, uint8_t SF = MAN_1200); //set up receiver
    
    void transmitArray(uint32_t numWords, uint32_t *data); // transmit array of mil-std-1553 words
    
    void beginReceiveArray(uint8_t maxWords, uint32_t *data);
    uint8_t receiveComplete(void);
    uint8_t speedFactor;
    uint16_t delay1;
    uint16_t delay2;
    
  private:
    void sendZero(void);
    void sendOne(void);
    uint8_t TxPin;
};//end of class Mil_std_1553

// Cant really do this as a real C++ class, since we need to have an ISR
extern "C"
{
    extern void MANRX_SetRxPin(uint8_t pin);
    extern void MANRX_SetupReceive(uint8_t speedFactor = MAN_1200);
    extern void MANRX_BeginReceiveWords(uint8_t maxWords, uint32_t *data);
    extern uint8_t MANRX_ReceiveComplete(void);
    
    extern void start_lcd();
    extern void print_temp_rpm(double temp, uint32_t rpm);
    extern int calculate_parity(uint32_t pilot_word);
    extern void generate_data_word(uint32_t* data_word_ptr, uint32_t data);
    extern int parse_data_word(uint32_t pilot_word, uint32_t *data_ptr);
    extern void generate_command_word(uint32_t* command_word_ptr, uint32_t rt_address, int mode, uint32_t sub_address);
    extern int check_status_word(uint32_t pilot_word, uint32_t rt_address);
    extern void generate_status_word(uint32_t *status_word_ptr, uint32_t rt_address, int message_error);
    extern int start_transceive(boolean mode, uint32_t rt_address, uint32_t sub_adress, uint32_t *data_ptr, bool demonstration_mode);
    extern int wait_for_transceive(uint32_t rt_address, uint32_t *sub_adress, uint32_t *data_ptr, bool demonstration_mode);
    extern int recover_command_word(uint32_t pilot_word, uint32_t rt_address, uint32_t *sub_address);
    extern int answer_to_bc(uint32_t rt_address, uint32_t data, bool demonstration_mode);
    extern void print_time_lcd(String first_line, String second_line);
}
extern Mil_std_1553 mil;

#endif