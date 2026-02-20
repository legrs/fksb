
#include <TWELITE>
#include <NWK_SIMPLE>
#include <BRD_APPTWELITE>
#include <string.h>
#include <cmath>

#define A_MMIN 640 //実際に出力するpwm
#define A_MNOR 3840
#define A_MAX 16384 //ref https://mwx.twelite.info/latest1/api-reference/predefined_objs/timers#change_hz

#define APP_ID 0x1234abcd
#define CHANNEL 13
#define REC_LID 0x01
#define LID 0x00
#define DAT_LEN 37
#define PKT_LEN 18

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

// set rotation speed
void setrs(const uint8_t num, const uint16_t val){ //num0,1,2 |-> Timer1,3,4   0 < val < 200
    //if( MMAX < val ) val = MMAX;
    //val += AMMIN;
    switch(num){
        case 0:
            Timer1.change_duty(val, A_MAX);
            break;
        case 1:
            Timer4.change_duty(val, A_MAX);
            break;
        case 2:
            Timer3.change_duty(val, A_MAX);
            break;
    }
}
void makePacket(byte dat[], uint8_t &index, byte q[8], uint16_t mp[3], byte &flag1, byte &flag2, uint8_t &solar){
    dat[0] = index;
    dat[1] = flag1;
    dat[2] = flag2;
    for(uint8_t j=3; j<3+8; j++){
        dat[j] = q[j-3];
    }//3 ~ 10

    //11 12
    //13 14
    //15 16
    memcpy(&dat[11], &mp[0], 2);
    memcpy(&dat[13], &mp[1], 2);
    memcpy(&dat[15], &mp[2], 2);

    //// 17
    dat[17] = solar;
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
                value = (uint16_t)(A_MAX*0.05);
                break;
            case '2':
                value = (uint16_t)(A_MAX*0.06);
                break;
            case '3':
                value = (uint16_t)(A_MAX*0.07);
                break;
            case '4':
                value = (uint16_t)(A_MAX*0.08);
                break;
            case '5':
                value = (uint16_t)(A_MAX*0.09);
                break;
            case '6':
                value = (uint16_t)(A_MAX*0.10);
                break;
                /*
            case '7':
                value = (uint16_t)(A_MAX*0.5);
                break;
            case '8':
                value = (uint16_t)(A_MAX*0.6);
                break;
            case '9':
                value = (uint16_t)(A_MAX*0.08);
                break;
            case '0':
                value = (uint16_t)(A_MAX*0.09);
                break;
            case '-':
                value = (uint16_t)(A_MAX*0.1);
                break;
                */
            default:
                value = 0;
                break;
        }

        Serial << (int)value << crlf;
        setrs(0, value);
        setrs(1, value);
        setrs(2, value);
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
/*


   flash のやつまだ書きとちゅうなんだからねっ！！


   やること
   - gyroとかの座標系を、しらべてほしいんだじぇ
   - flashの読み書きテスト
   - godotとR3でflash読むテスト
   - motoe まわせ


*/

