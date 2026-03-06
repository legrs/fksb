/*

   *************: program for flash test ********:::

   */
#include <TWELITE>
#include <NWK_SIMPLE>
#include <BRD_APPTWELITE>
#include <string.h>
#include <cmath>

#define BNO_DEV 0x28
#define BNO_REG_A 0x08 //acceleration register adress(6)
#define BNO_REG_M 0x0E //magnetron register adress(6)
#define BNO_REG_G 0x14 //gyro(differential of pos) register adress(6)
#define BNO_REG_Q 0x20 //quaternion register adress(8byte)
#define BNO_REG_T 0x34 //templature register adress(1)
#define BNO_CALIB_STAT 0x35
#define BNO_OPR_MODE 0x3D
#define BNO_UNIT_SEL 0x3B
#define BNO_RST_PIN 10
#define ESC_PIN0 5
#define ESC_PIN1 19
#define ESC_PIN2 4
#define HOLD 4
#define BTN 8
#define IN 0 //A1Pinは番号0

#define M_MIN 80
#define M_NOR 200
#define M_MAX 380 //3200
#define A_FAC 819 // 16384 * 0.05 5%
#define DUTY_MAX 16384 //ref https://mwx.twelite.info/latest1/api-reference/predefined_objs/timers#change_hz
#define MMIND 1//1frameで，リカバー用に加えてもいいdpwm

#define APP_ID 0x1234abcd
#define CHANNEL 13
#define REC_LID 0x01
#define LID 0x00
#define FLASH_MAX 0x07FFFF
#define DAT_LEN 36
#define PKT_LEN 17

/*
       pwmのピンの対応(LEDつけてしらべた) シルクnumberやえ
timer0 : 10
timer1 : 11
timer2 : 12
timer3 : 13
timer4 : 17
       */
void send(const uint8_t adr, const byte dat[PKT_LEN], uint8_t len);
void printByte(const byte buff[], const uint8_t len);
void printByte(const byte buff);
void printByteBin(const byte buff);
void majiKeisandekiru3();

class I2C{
    private:
        byte adr = 0x00;

    public:
        void setAdr(const byte _adr){
            adr = _adr;
        }
        bool write(byte REG, byte VAL){
            if(auto&& wrt = Wire.get_writer(adr)){
                wrt << REG;
                wrt << VAL;
            }else{
                return false;
            }
            return true;
        }
        bool set(byte REG){
            if(auto&& wrt = Wire.get_writer(adr)){
                wrt << REG;
            }else{
                return false;
            }
            return true;
        }
        bool read(byte val[], uint8_t len){
            if(auto&& rdr = Wire.get_reader(adr, len)){
                for(uint8_t i=0; i<len; i++){
                    rdr >> val[i];
                }
            }else{
                return false;
            }
            return true;
        }
        //bool setread(byte REG, byte val[], uint8_t len){
        //    if(set(REG))
        //        return read(val, len);
        //    return false;
        //}
        bool readByte(byte REG, byte val){
            if(set(REG)){
                if(auto&& rdr = Wire.get_reader(adr, 1)){
                    rdr >> val;
                }else{
                    return false;
                }
                return true;
            }
            return false;
        }
};
class Flash{
    public:
        uint32_t cAdr = 0x00000000;
        void read(uint32_t adr, byte dat[], uint8_t len){
            if(auto&& trs = SPI.get_rwer()){
                trs << 0x03;
                trs << (byte)(adr >> 16);
                trs << (byte)(adr >> 8) ;
                trs << (byte)adr       ;
                delay(1);
                for(uint8_t i=0; i<4; i++){
                    trs << 0x00;
                    trs >> dat[0];
                }
                for(uint8_t i=0; i<len; i++){
                    trs << 0x00;
                    trs >> dat[i];
                }
            }
            //cAdr += len;
        }
        bool isWriteEnabled(){
            //return true;
            if(auto&& trs = SPI.get_rwer()){
                byte stat;
                trs << 0x05;
                delay(1);
                trs << 0x00;
                trs >> stat;
                trs << 0x00;
                trs >> stat;
                if(!(stat&(1<<0))){
                    return true;
                }else{
                    return false;
                }
            }
        }
        //WEL  !!  max 256 byte !!
        void write(uint32_t adr, byte dat[], uint8_t len, bool doWrite){ 
            if(doWrite){
                while(!isWriteEnabled()){
                    delay(1);
                }
                //WREN ( allow twelite to write)
                if(auto&& trs = SPI.get_rwer()){
                    trs << 0x06;
                }
                // wrenからすぐにprogramしないとダメ ここでstatus checkするな でもたぶんifは別々にすべき
                //write(PP)
                if(auto&& trs = SPI.get_rwer()){
                    trs << 0x02;
                    trs << (byte)(adr >> 16);
                    trs << (byte)(adr >> 8) ;
                    trs << (byte)adr       ;
                    for(uint8_t i=0; i<len; i++){
                        trs << dat[i];
                    }
                }
                cAdr += len;
            }
            return;
        }
        void eraseAll(){
            //RDSR status check
            while(!isWriteEnabled()){
                //これするときはよゆうあるから10ms
                delay(10);
            }
            //WREN ( allow twelite to write)
            if(auto&& trs = SPI.get_rwer()){
                trs << 0x06;
            }
            //CER 
            if(auto&& trs = SPI.get_rwer()){
                trs << 0xC7;
            }
            //delay(2000);
            while(!isWriteEnabled()){
                delay(200);
            }
            cAdr = 0;
            return;
        }
        void print(uint32_t adr, uint16_t len){
            if(auto&& trs = SPI.get_rwer()){
                byte dump;
                trs << 0x03;
                trs << (byte)(adr >> 16);
                trs << (byte)(adr >> 8) ;
                trs << (byte)adr       ;
                delay(1);
                for(uint8_t i=0; i<4; i++){
                    trs << 0x00;
                    trs >> dump;
                }
                for(uint16_t i=0; i<len+4; i++){
                    trs << 0x00; //dummy byte( to generate clock)
                    trs >> dump;
                    Serial.print(dump, HEX);
                    Serial.print(" ");
                }
                Serial.println("");
            }
        }
};
class Quat{
    public:
        float w;
        float x;
        float y;
        float z;
        constexpr Quat() : w(1),x(0),y(0),z(0){}
        Quat(float _w,float _x,float _y,float _z) : w(_w),x(_x),y(_y),z(_z){}
        friend Quat operator*(const Quat &q1 , const Quat &q2){
            return Quat(
                    -q1.x*q2.x - q1.y*q2.y - q1.z*q2.z + q1.w*q2.w,
                     q1.w*q2.x - q1.z*q2.y + q1.y*q2.z + q1.x*q2.w,
                     q1.z*q2.x + q1.w*q2.y - q1.x*q2.z + q1.y*q2.w,
                    -q1.y*q2.x + q1.x*q2.y + q1.w*q2.z + q1.z*q2.w
                    );
        }

        void setFromByte(const byte buffer[]){
            const float scale = (1.0 / (1 << 14));
            int16_t x1, y1, z1, w1;
            x1 = y1 = z1 = w1 = 0;
            w1 = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
            x1 = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
            y1 = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
            z1 = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
            w = w1 *  scale;
            x = x1 *  scale;
            y = y1 *  scale;
            z = z1 *  scale;
        }
        void print(){
            Serial << w << " "  << x << " " << y << " " << z << mwx::crlf;
        }
        Quat invert(){
            Quat q;
            q.w =  w;
            q.x = -x;
            q.y = -y;
            q.z = -z;
            return q;
        }

};
class Vec{
    public:
        float x;
        float y;
        float z;

        Vec() : x(0),y(0),z(0){}
        Vec(float _x,float _y,float _z) : x(_x),y(_y),z(_z){}
        // multiply
        friend Vec operator*(const Vec v , const float f){
            return Vec(v.x*f,v.y*f,v.z*f);
        }
        friend Vec operator/(const Vec v , const float f){
            return Vec(v.x/f,v.y/f,v.z/f);
        }
        // cross
        friend Vec operator%(const Vec v1 , const Vec v2){
            return Vec( 
                v1.y*v2.z - v1.z*v2.y,
                v1.z*v2.x - v1.x*v2.z,
                v1.x*v2.y - v1.y*v2.x
            );
        }
        // dot
        friend float operator*(const Vec v1 , const Vec v2){
            return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
        }

        void setFromByte(const byte buffer[], const uint8_t k){ //0: gyro, 1:acc, 2:mag
            float scale = 1.0;;
            switch(k){
                case 0:
                    scale = 900.0; // rad/s
                    break;
                case 1:
                    scale = 16.0; // uT
                    break;
                case 2:
                    scale = 100.0; // m/s^2
                    break;
                default:
                    break;
            }

            int16_t x1, y1, z1;
            x1 = y1 = z1 = 0;
            x1 = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
            y1 = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
            z1 = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
            x = x1 / scale;
            y = y1 / scale;
            z = z1 / scale;
        }
        void print(){
            Serial << x << " " << y << " " << z << mwx::crlf;
        }
        void clip(Vec thres){
            float fac[3] = {x/thres.x , y/thres.y , z/thres.z};
            float max = fac[0];
            float min = fac[0];
            if( max < fac[1] ){
                max = fac[1];
            }
            if( max < fac[2] ){
                max = fac[2];
            }
            if( fac[1] < min ){
                min = fac[1];
            }
            if( fac[2] < min ){
                min = fac[2];
            }

            if( abs(min) < abs(max) ){
                // clip on upper
                if( 1 < max ){
                    x /= max;
                    y /= max;
                    z /= max;
                }
            }else{
                if( min < -1 ){
                    x /= -min;
                    y /= -min;
                    z /= -min;
                }
            }
        }
        Vec rotate(Quat q){
            Quat q1(x,y,z,0);
            Quat q2 = q * q1 * q.invert();
            return Vec(q2.x,q2.y,q2.z);
        }
};
// set rotation speed
void setrs(const uint8_t num, const uint16_t val){ //num0,1,2 |-> Timer1,3,4   0 < val < 1024(1<<10)
    // 0.05 ~ 0.10 ( 50Hz )
    if(val == -1 || 0 <= val && val <= (1<<10)){
        float value;
        if(val == -1){
            value = 0;
        }else{
            value = A_FAC * (float)((1<<10) + val) / (float)(1<<10);
        }
        Serial.println(value);
        switch(num){
            case 0:
                Timer1.change_duty((uint16_t)value, DUTY_MAX);
                break;
            case 1:
                Timer4.change_duty((uint16_t)value, DUTY_MAX);
                break;
            case 2:
                Timer3.change_duty((uint16_t)value, DUTY_MAX);
                break;
        }
    }
}
void makePacket(byte dat[], uint8_t &index, byte q[8], uint16_t mp[3], byte &flag1, byte &flag2){
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
void printByteBin(const byte buff){
    for(uint8_t i=0; i<8; i++){
        if(buff&(1<<i)){
            Serial << "1";
        }else{
            Serial << "0";
        }
    }
    Serial << crlf;
}

I2C bno;
Flash f;

const float bpmoi[3] = {1000,1000,1000};
const float wpmoi = 100;

uint8_t index;
uint8_t temp; // temperature



byte flag1;
byte flag2;
byte buff[8];
uint16_t mp[3];
byte dat[DAT_LEN];

Quat ori; // orientation
Vec avel; // angular velocity = gyro
Vec acce;

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
    pinMode(BNO_RST_PIN, OUTPUT);
    digitalWrite(BNO_RST_PIN, LOW);
    delay(10);
    digitalWrite(BNO_RST_PIN, HIGH);

    Serial << "--- program start ---" << crlf;

    index = 0;
    flag1 = 0b00000000;
    flag2 = 0b00000000;
    // mp[0] = ----
    

        /****** setup spi   ******/


    SPI.begin(0 // DIO19をチップセレクトとして使用
              , { 5000000UL // クロック周波数 
            , SPI_CONF::MSBFIRST
            , SPI_CONF::SPI_MODE0
    });
    delay(1000);

    if(auto&& trs = SPI.get_rwer()){

            // このへん要検討 -----------------------------------------------------------||

        byte deb;
        trs << 0x9F;
        delay(1);
        trs << 0x00;
        trs >> deb;
        trs << 0x00;
        trs >> deb;
        if(deb==0x9D){
            Serial << "flash memory : ok" << mwx::crlf;
            flag2 |= (1<<2);
        }else{
            Serial << "flash memory : failed!" << mwx::crlf;
        }
    }

    //f.eraseAll();    

        /****** setup i2c   ******/

    Wire.begin();  
    delay(500);

    // bno setup ( https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
    bno.setAdr(BNO_DEV);
    if(bno.write(BNO_OPR_MODE, 0x08)){
        delay(20);
        // set units
        bno.write(0x3B, (0<<0)); 
        delay(10);
        bno.write(0x3B, (1<<1)); 
        delay(10);
        bno.write(0x3B, (0<<4)); 

        Serial << "bno : ok" << mwx::crlf;
        flag2 |= (1<<0);
    }else{
        Serial << "bno : failed!" << mwx::crlf;
    }

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
    flag1 |= (1<<7); // debug stop
    //flag1 |= (1<<0); // writing
    //flag1 |= (1<<1); // launched

    uint8_t vadr = 0;

    f.eraseAll();    

    delay(10);


        Serial.println(f.cAdr);
    for(uint8_t j=0x0A; j<0x0A+5; j++){
        buff[0] = j;
        f.write(f.cAdr, buff, 1, 1);
        buff[0] = flag1;
        f.write(f.cAdr, buff, 1, 1);
        buff[0] = flag2;
        f.write(f.cAdr, buff, 1, 1);

        for(uint8_t i=0; i<8; i++){
            buff[i] = i;
        }
        buff[0] = j;
        f.write(f.cAdr, buff, 8, 1);
        f.write(f.cAdr, buff, 6, 1);
        f.write(f.cAdr, buff, 6, 1);
        f.write(f.cAdr, buff, 6, 1);
        f.write(f.cAdr, buff, 6, 1);

        buff[0] = 100;
        f.write(f.cAdr, buff, 1, 1);

            Serial << (int)f.cAdr << crlf;
    }
    f.cAdr = 0;
    for(uint8_t j=0; j<5; j++){
        f.read(f.cAdr, dat, 36);
        f.cAdr+=36;
        printByte(dat, 36);
    }


}

/*** the loop procedure (called every event) */
void loop() {
    /*
    // do run? (only for debug!!)
    if(!(flag1&(1<<7))){
        // do write?
        if(flag1&(1<<0)){
            printByteBin(flag1);
            printByteBin(flag2);

            // is launched?
            if(flag1&(1<<1)){
                delay(30);
            }else{
                delay(100);
            }
            index++;
            if(f.cAdr > FLASH_MAX){
                f.cAdr = 0x000000;
                Serial << "------------------ memory full -----------------------" << crlf;
            }

            // -------------------- get data --

            // calibration status
            bno.set(BNO_CALIB_STAT);
            bno.read(buff, 1);
            if(buff[0]&(1<<6)){
                Serial << "6" << crlf;
            }
            if(buff[0]&(1<<7)){
                Serial << "7" << crlf;
            }
            //Serial.println(buff[0], BIN);

                // write inedex flags
            buff[0] = index;
            f.write(f.cAdr, buff, 1, 1);
            buff[0] = flag1;
            f.write(f.cAdr, buff, 1, 1);
            buff[0] = flag2;
            f.write(f.cAdr, buff, 1, 1);

                // quaternion
            bno.set(BNO_REG_Q);
            bno.read(buff, 8);
            ori.setFromByte(buff);
            //ori.print();
            f.write(f.cAdr, buff, 8, 1);

                // gyro
            bno.set(BNO_REG_G);
            bno.read(buff, 6);
            avel.setFromByte(buff,0);
            f.write(f.cAdr, buff, 6, 1);


                // -------------------- operate motors --

            // is launched?
            if(flag1&(1<<1)){
                // do control?
                if(flag1&(1<<2)){
                    majiKeisandekiru3();
                    //for(uint8_t i=0; i<3; i++){
                    //    setrs(i, mp[i]);
                    //}
                }else{
                    setrs(0, M_NOR);
                }
            }else{
                setrs(0, 0);
                setrs(1, 0);
                setrs(2, 0);
            }

                // acceralation
            bno.set(BNO_REG_A);
            bno.read(buff, 6);
            acce.setFromByte(buff,1);
            f.write(f.cAdr, buff, 6, 1);

                // write mps
            memcpy(&buff[0], &mp[0], 2);
            memcpy(&buff[2], &mp[1], 2);
            memcpy(&buff[4], &mp[2], 2);
            f.write(f.cAdr, buff, 6, 1);

                // magnetic
            bno.set(BNO_REG_M);
            bno.read(buff, 6);
            acce.setFromByte(buff,2);
            f.write(f.cAdr, buff, 6, 1);

                // temperature
            bno.set(BNO_REG_T);
            bno.read(buff, 1);
            temp = buff[0];
            f.write(f.cAdr, buff, 1, 1);

            if(index%10 == 0){
                makePacket(dat, index, buff, mp, flag1, flag2);
                send(REC_LID, dat, PKT_LEN);
                index++;
            }
        }else{

            // -------------------------- eco-mode --

            setrs(0, 0);
            setrs(1, 0);
            setrs(2, 0);
            delay(500);
            index++;

            // bno status
            bno.set(BNO_CALIB_STAT);
            bno.read(buff, 1);
            if(buff[0]&(1<<6)){
                Serial << "6" << crlf;
            }
            if(buff[0]&(1<<7)){
                Serial << "7" << crlf;
            }

            // acceralation
            bno.set(BNO_REG_A);
            bno.read(buff, 6);
            acce.setFromByte(buff,2);
            acce.print();

            if(index%2 == 0){
                dat[0] = index;
                dat[1] = flag1;
                dat[2] = flag2;
                memcpy(&dat[3], &buff, 6);

                send(REC_LID, dat, 9);
            }

            while(Serial.available())  {
                int c = Serial.read();

                switch(c) {
                    case 's': //prepare
                        f.cAdr = 0;
                        flag1 |= (1<<7);
                        memset(dat, 0, DAT_LEN);
                }
            }
        }
    }else{
        while(Serial.available())  {
            int c = Serial.read();
            switch(c) {
                case 'r': //read 1section
                    f.read(f.cAdr, dat, DAT_LEN);
                    Serial << dat;
                    f.cAdr += DAT_LEN;
                    break;
            }
        }
    }
    */
}

void majiKeisandekiru3(){

}
void send(const uint8_t adr, const byte dat[], uint8_t len) {
    if (auto&& pkt = the_twelite.network.use<NWK_SIMPLE>().prepare_tx_packet()) {
        pkt << tx_addr(adr)  // 0..0xFF (LID 0:parent, FE:child w/ no id, FF:LID broad cast), 0x8XXXXXXX (long address)
            << tx_retry(0x0) // set retry (0x3 send four times in total)
            << tx_packet_delay(0,5,20); // send packet w/ delay (send first packet with randomized delay from 100 to 200ms, repeat every 20ms)
        pack_bytes(pkt.get_payload() // set payload data objects.
            , make_pair(dat, (int)len) // string should be paired with length explicitly.
        );
        pkt.transmit();
    }
}
void on_rx_packet(packet_rx& rx, bool_t &handled) {
    //expand_bytes(rx.get_payload().begin(), rx.get_payload().end()
    //            , com       // 4bytes of msg
    //);
    auto dat = rx.get_payload();
    char com = dat[0];

    Serial << com << mwx::crlf;

    // correspond to keyboad
    // ここだけ書きかえればよい
    switch(com){
        case '1':
            index = 0;
            f.eraseAll();
            flag1 |=  (1<<0);
            break;
        case 'q':
            flag1 &= ~(1<<0);
            break;
        case '2':
            flag1 |=  (1<<1);
            break;
        case 'w':
            flag1 &= ~(1<<1);
            break;
        case '3':
            break;
        case 'e':
            break;
        // debug command
        case 'z':
            flag1 |=  (1<<7);
            break;
        case 'x':
            flag1 &= ~(1<<7);
            break;
    }
    handled = true; //処理完了とする
}



/*
    
    bit flags
    flag1
        0 : do write? 1=write
        1 : is launched?
        2 : do control?
        3 : 
        4 : 
        5 : 
        6 : 
        7 : 
        8 : is stopping running? 1=stopping
    flag2   
        0 : is bno connection ok?
        1 : is bno calibrated?
        2 : is flash connection ok?
        3 : is separated?
        4 : is landing?
        5 : 
        6 : 
        7 : 
        8 :

        operating modes ( 0x3D ) 0000<3:0>

          | accel | mag | gyro | relativeOri | absOri
    ------------------------------------------------------------
        0 |       |     |      |             |      
    ------------------------------------------------------------
        1 |   X   |     |      |             |      
    ------------------------------------------------------------
        2 |       |  X  |      |             |      
    ------------------------------------------------------------
        3 |       |     |  X   |             |      
    ------------------------------------------------------------
        4 |   X   |  X  |      |             |      
    ------------------------------------------------------------
        5 |   X   |     |  X   |             |      
    ------------------------------------------------------------
        6 |       |  X  |  X   |             |      
    ------------------------------------------------------------
        7 |   X   |  X  |  X   |             |      
    ------------------------------------------------------------
        8 |   X   |     |  X   |      X      |      
    ------------------------------------------------------------
        9 |   X   |  X  |      |             |   X  
    ------------------------------------------------------------
        a |   X   |  X  |      |      X      |      
    ------------------------------------------------------------
        b |   X   |  X  |  X   |             |   X  
    ------------------------------------------------------------
        c |   X   |  X  |  X   |             |   X  

*/
/*

    flash drive byte costruction

4*1024*1024/8bit/(30fps*5min*60sec) = 58.3 byte/frame

     0    index
     1    flag1
     2    flag2
     3-10 ori
    11-16 avel 
    17-22 acce
    23-28 mp
    29-34 mag
    35 temp

     
     
     
     
     

*/
/*


   flash のやつまだ書きとちゅうなんだからねっ！！


   やること
   - gyroとかの座標系を、しらべてほしいんだじぇ
   - flashの読み書きテスト
   - godotとR3でflash読むテスト
   - motoe まわせ


*/

