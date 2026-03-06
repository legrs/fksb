#include <TWELITE>

#define DAT_LEN 18

uint8_t index;
byte dat[18];

void setup() {
    Serial << "--- program start  ---" << crlf;
    memset(dat, 0x01, DAT_LEN);
    index = 0;
}

/*** loop procedure (called every event) */
void loop() {
    if (Serial.available()) {
        dat[0] = index;
        if(index % 5){
            dat[1] |= (1 << 0);
        }else{
            dat[1] &= ~(1 << 0);
        }
        dat[2] = index;
        Serial << dat;
        delay(100);
        index++;
        //Serial << "--- act4 ( for setial read with C#  test)  ---" << crlf;
    }
}
/*
    transmit dat construction

index           1
flag1           1
flag2           1
Orientation :   8 byte quaternion
mp              6 
solar           1
    sum         18 byte
*/
