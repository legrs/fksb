/*  for ESC setup

    key inpu
        1:min rotation signal setrs(0) 
        -:max rotation signal setrs(1024)

*/
#include <TWELITE>
#include <NWK_SIMPLE>
#include <BRD_APPTWELITE>
#include <string.h>
#include <cmath>


#define APP_ID 0x1234abcd
#define CHANNEL 13
#define REC_LID 0x01
#define LID 0x00
#define DAT_LEN 37
#define PKT_LEN 18

constexpr uint16_t A_ZERO =   819; // 16384 * 0.05 5% 1000microsecondsに相当
constexpr uint16_t A_MIN  =   861; // モーターが実際に回転を始めるduty(1単位で実測)
constexpr uint16_t A_MAX  =   1638; // 10%のduty
constexpr uint16_t A_RANGE =  777; // A_MAX - A_MIN
constexpr uint16_t DUTY_MAX = 16384; //ref https://mwx.twelite.info/latest1/api-reference/predefined_objs/timers#change_hz
/*
       pwmのピンの対応(LEDつけてしらべた) シルクnumberやえ
timer0 : 10
timer1 : 11
timer2 : 12
timer3 : 13
timer4 : 17
       */
void send(const uint8_t adr, const byte dat[PKT_LEN]);
void printByte(const byte buff[], const uint8_t len);
void printByte(const byte buff);

void setrs(const uint8_t num, const uint16_t val){ //num0,1,2 |-> Timer4,1,3   0 < val < 1024(1<<10)  0xFF00で切れない最小のpwm，0xFFFFで0電圧
    // 0.05 ~ 0.10 ( 50Hz )
    if(val == 0xFF00 || val == 0xFFFF || 0 <= val && val <= (1<<10)){
        float value;
        if(val == 0xFF00){
            value = A_ZERO;
        }else if(val==0xFFFF){
            value = 0;
        }else{
            value = val*(float)(A_RANGE) / (float)(1<<10)  + A_MIN;
        }
        switch(num){
            case 0:
                Timer4.change_duty((uint16_t)value, DUTY_MAX);
                break;
            case 1:
                Timer1.change_duty((uint16_t)value, DUTY_MAX);
                break;
            case 2:
                Timer3.change_duty((uint16_t)value, DUTY_MAX);
                break;
        }
    }
    return;
}

// benri
void printByte(const byte buff[], const uint8_t len){
    for(uint8_t i=0; i<len; i++){
        Serial.print(buff[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
}
void printByte(const byte buff){
    Serial.println(buff, HEX);
}

uint8_t index;

/*** the setup procedure (called on boot) */
void setup() {
    Timer1.setup();
    Timer1.begin(50,false,true);
    Timer1.change_duty(1);
    Timer1.change_hz(50,0);
    Timer3.setup();
    Timer3.begin(50,false,true);
    Timer3.change_duty(1);
    Timer3.change_hz(50,0);
    Timer4.setup();
    Timer4.begin(50,false,true);
    Timer4.change_duty(1);
    Timer4.change_hz(50,0);

    Serial << "--- program start ---" << crlf;

    index = 0;
        /******  twelite setup ******/

    the_twelite
        << TWENET::appid(APP_ID)    // set application ID (identify network group)
        << TWENET::channel(CHANNEL) // set channel (pysical channel)
        << TWENET::rx_when_idle();  // open receive circuit (if not set, it can't listen packts from others)
    auto&& nwksmpl = the_twelite.network.use<NWK_SIMPLE>();
    nwksmpl << NWK_SIMPLE::logical_id(LID) // set Logical ID. (0xFE means a child device with no ID)
            << NWK_SIMPLE::repeat_max(3);   // can repeat a packet up to this times. (being kind of a router)
    the_twelite.begin(); // start twelite!
    delay(100);


    //////////////////////////    これより下debug    //////////////////////////

    // loop stop
    //flag1 |= (1<<0);

}

uint16_t value;

/*** the loop procedure (called every event) */
void loop() {

    while(Serial.available())  {
        int c = Serial.read();

        Serial << char(c) << " ";

        switch(c) {
            case '1':
                setrs(0, 0xFF00);
                setrs(1, 0xFF00);
                setrs(2, 0xFF00);
                break;
            case '2':
                setrs(0, 0);
                break;
            case '3':
                setrs(1, 0);
                break;
            case '4':
                setrs(2, 0);
                break;
            case '5':
                //setrs(0, 400);
                break;
            case '6':
                //setrs(1, 400);
                break;
            case '7':
                //setrs(2, 400);
                break;
            case '8':
                //value = (uint16_t)(1024*0.7);
                //value = (uint16_t)(106);
                break;
            case '9':
                //value = (uint16_t)(1024*0.8);
                //value = (uint16_t)(107);
                break;
            case '0':
                //value = (uint16_t)(1024*0.9);
                //value = (uint16_t)(108);
                break;
            case '-':
                setrs(0, 1024);
                setrs(1, 1024);
                setrs(2, 1024);
                //value = (uint16_t)(1024*1.0);
                break;
            default:
                setrs(0, 0);
                setrs(1, 0);
                setrs(2, 0);
                //value = 0;
                break;
        }

        //Serial.println((float)value);
        //Serial.println(1024, HEX);
        //Serial.println(1024.0, DEC);
    }
        index++;

    delay(100);
}

void send(const uint8_t adr, const byte dat[]) {
    if (auto&& pkt = the_twelite.network.use<NWK_SIMPLE>().prepare_tx_packet()) {
        pkt << tx_addr(adr)  // 0..0xFF (LID 0:parent, FE:child w/ no id, FF:LID broad cast), 0x8XXXXXXX (long address)
            << tx_retry(0x0) // set retry (0x3 send four times in total)
            << tx_packet_delay(0,5,20); // send packet w/ delay (send first packet with randomized delay from 100 to 200ms, repeat every 20ms)
        pack_bytes(pkt.get_payload() // set payload data objects.
            , make_pair(dat, PKT_LEN) // string should be paired with length explicitly.
        );
        pkt.transmit();
    }
}
void on_rx_packet(packet_rx& rx, bool_t &handled) {
    byte com;
    expand_bytes(rx.get_payload().begin(), rx.get_payload().end()
                , com       // 8bytes of msg
    );
    Serial << com << mwx::crlf;
    if(com & (1<<     0)){
    }else if(com&(1<< 1)){
        //f.eraseAll();
    }else if(com&(1<< 2)){
    }else if(com&(1<< 3)){
    }else if(com&(1<< 4)){
    }else if(com&(1<< 5)){
    }
    /*
        0 :
        1 :
        2 :
        3 :
        4 :
        5 :
       */
    handled = true; //処理完了とする
}

