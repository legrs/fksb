// use twelite mwx c++ template library
// ------ fksBRes.cpp ------
// fksb れしーば これはただデータSerialにbyte列を出力するだけTWELITE MONOSTICKをpcに指す  これは
#include <TWELITE>
#include <NWK_SIMPLE>

#define APP_ID 0x1234abcd
#define CHANNEL 13
#define LID 0x01
#define SAT_LID 0x00

#define MAX_DAT_LEN 20

void send(uint8_t adr, byte com);

void setup() {
    the_twelite
        << TWENET::appid(APP_ID)    // set application ID (identify network group)
        << TWENET::channel(CHANNEL) // set channel (pysical channel)
        << TWENET::rx_when_idle();  // open receive circuit (if not set, it can't listen packts from others)
    auto&& nwksmpl = the_twelite.network.use<NWK_SIMPLE>();
    nwksmpl << NWK_SIMPLE::logical_id(LID) // set Logical ID. (0xFE means a child device with no ID)
            << NWK_SIMPLE::repeat_max(3);   // can repeat a packet up to three times. (being kind of a router)
    the_twelite.begin(); 
}


void loop() {
    while(Serial.available())  {
        int c = Serial.read();
        send(SAT_LID, char(c));
    }
}

void send(uint8_t adr, byte com) {
    if (auto&& pkt = the_twelite.network.use<NWK_SIMPLE>().prepare_tx_packet()) {
        pkt << tx_addr(adr)  // 0..0xFF (LID 0:parent, FE:child w/ no id, FF:LID broad cast), 0x8XXXXXXX (long address)
            << tx_retry(0x5) // set retry (0x3 send four times in total)
            << tx_packet_delay(0,5,20); // send packet w/ delay (send first packet with randomized delay from 100 to 200ms, repeat every 20ms)
        pack_bytes(pkt.get_payload() // set payload data objects.
            , com // string should be paired with length explicitly.
        );
        pkt.transmit();
    }
}
void on_rx_packet(packet_rx& rx, bool_t &handled) {
    //Serial << "got data " << mwx::crlf;
    // rx >> Serial; // debugging (display longer packet information)
    auto dat = rx.get_payload();
    Serial << dat;
    
    handled = true; //処理完了とする
}
