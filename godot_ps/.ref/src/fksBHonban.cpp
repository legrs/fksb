// use twelite mwx c++ template library
// ------ fksBHonban.cpp ------
// fksb本体のcode,本番用コード() 
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
#define BNO_CAL_DATA 0x35
#define BNO_RST_PIN 10
#define ESC_PIN0 5
#define ESC_PIN1 19
#define ESC_PIN2 4
#define HOLD 4
#define BNO_R 10
#define FLASH_BTN 8
#define IN 0 //A1Pinは番号0
#define MMIN 80
#define MNOR 200
#define MMAX 380 //3200
#define AMMIN 640 //実際に出力するpwm
#define AMNOR 3840
#define MMIND 1//1frameで，リカバー用に加えてもいいdpwm
#define APP_ID 0x1234abcd
#define CHANNEL 13
#define REC_LID 0x01
#define LID 0x00
#define FDAT_LEN 37
#define DAT_LEN 25
#define RED true

#define DEBUG_PIN 9
#define C1 0.5 //Δtとフレームの長さの比

using namespace std;
    /*
       pwmのピンの対応(LEDつけてしらべた)
timer1 : 5
timer2 : C
timer3 : I
timer4 : 8
2025 9 14 いつのまにかpwmピンの対応がかわった( どういうこと？)
       pwmのピンの対応(LEDつけてしらべた)
timer0 : 10
timer1 : 11
timer2 : 12
timer3 : 13
timer4 : 17
       */

const float I[3][3] = {  // freeCADでけさんxとyをいれかえてある
    {228420.0056165047, 8274.768464746485, -9381.85078913948}, 
    {8274.768464746485, 232671.22497855005, -1539.1150197159873},
    {-9381.85078913948, -1539.1150197159873, 126288.87099417835}
};



void send(uint8_t adr, byte dat[]);

struct I2C{
    bool write(byte ADR, byte REG, byte VAL){
        if(auto&& wrt = Wire.get_writer(ADR)){
            wrt << REG;
            wrt << VAL;
        }else{
            return false;
        }
        return true;
    }
    bool set(byte ADR, byte REG){
        if(auto&& wrt = Wire.get_writer(ADR)){
            wrt << REG;
        }else{
            return false;
        }
        return true;
    }
    bool read(byte ADR, byte val[], uint8_t len){
        if(auto&& rdr = Wire.get_reader(ADR, len)){
            for(uint8_t i=0; i<len; i++){
                rdr >> val[i];
            }
        }else{
            return false;
        }
        return true;
    }
    bool setread(byte ADR, byte REG, byte val[], uint8_t len){
        if(set(ADR, REG))
            return read(ADR, val, len);
        return false;
    }
    bool readByte(byte ADR, byte REG, byte val){
        if(set(ADR, REG)){
            if(auto&& rdr = Wire.get_reader(ADR, 1)){
                rdr >> val;
            }else{
                return false;
            }
            return true;
        }
        return false;
    }
};
struct Flash{
    uint32_t cAdr;
    //NORD
    void read(uint32_t adr, byte dat[], uint8_t len){
        if(auto&& trs = SPI.get_rwer()){
            trs << 0x03;
            trs << (byte)(adr >> 16);
            trs << (byte)(adr >> 8) ;
            trs << (byte)adr       ;
            delay(1);
            for(uint8_t i=0; i<4; i++){
                trs << 0x00;
                trs >> dat[i];
            }
            for(uint8_t i=0; i<len+4; i++){
                trs << 0x00; //dummy byte( to generate clock)
                trs >> dat[i];
            }
        }
        //cAdr += len;
    }
    //WEL  !!  max 256 byte !!
    void write(uint32_t adr, byte dat[], uint8_t len, bool doWrite){ 
        if(doWrite){
            //WREN ( allow twelite to write)
            if(auto&& trs = SPI.get_rwer()){
                trs << 0x06;
            }//write(PP)
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
    }
    void eraseAll(){
        //WREN ( allow twelite to write)
        if(auto&& trs = SPI.get_rwer()){
            trs << 0x06;
        }
        //CER 
        if(auto&& trs = SPI.get_rwer()){
            trs << 0xC7;
        }
        delay(3000);
        cAdr = 0;
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
I2C bno;
Flash f;
class Quat{
    public:
        float x;
        float y;
        float z;
        float w;
        void setFromByte(byte buffer[]){
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
        Quat conjugate(){
            Quat q;
            q.x = -x;
            q.y = -y;
            q.z = -z;
            q.w =  w;
            return q;
        }
        Quat concatenate(Quat q){
            Quat q2;
            q2.x =  w*q.x - z*q.y + y*q.z + x*q.w;
            q2.y =  z*q.x + w*q.y - x*q.z + y*q.w;
            q2.z = -y*q.x + x*q.y + w*q.z + z*q.w;
            q2.w = -x*q.x - y*q.y - z*q.z + w*q.w;
            return q2;
        }
        Quat() : w(0),x(0),y(0),z(0){}
        Quat(float _w,float _x,float _y,float _z) : w(_w),x(_x),y(_y),z(_z){}
};
class Vector{
    public:
        float x = 0;
        float y = 0;
        float z = 0;
        void setFromByte(byte buffer[], uint8_t k){ //0: gyro, 1:acc, 2:mag
            float scale = 1.0;;
            switch(k){
                case 0:
                    scale = 900.0;
                    break;
                case 1:
                    scale = 16.0;
                    break;
                case 2:
                    scale = 100.0;
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
        void rotate(Quat q){
            Quat p;
            p.w = 0;
            p.x = x;
            p.y = y;
            p.z = z;
            p = q.concatenate(p);
            p = p.concatenate(q.conjugate());
            x = p.x;
            y = p.y;
            z = p.z;
        }
        float dot(Vector v){
            return x*v.x + y*v.y + z*v.z;
        }
        Vector cross(Vector v){
            Vector v2;
            v2.x = y*v.z - z*v.y;
            v2.y = z*v.x - x*v.z;
            v2.z = x*v.y - y*v.x;
            return v2;
        }
        Vector() : x(0),y(0),z(0){}
        Vector(float _x,float _y,float _z) : x(_x),y(_y),z(_z){}
};

void rotate(uint8_t num, uint16_t val){ //num0,1,2 |-> Timer1,3,4   0 < val < 200
    if( MMAX < val ) val = MMAX;
    val += AMMIN;
    switch(num){
        case 0:
            Timer1.change_duty(val, 16384);
            break;
        case 1:
            Timer4.change_duty(val, 16384);
            break;
        case 2:
            Timer3.change_duty(val, 16384);
            break;
    }
}

void majiKeisanDekiru2(Vector p, Vector dp, uint16_t mp[3], uint8_t &recVal){

    float c = 8; //dtの何倍時間後に0になるべきとするか
    float dt = 0.033; //delay(33)より
    /* 値てきとう */
    float in = 4000; //慣性モメント 単位をIときちんと合わせておけばrad s^-2が出て来る
    float d = 8; // rad dt^-2  -> pwm  実測めんどくさいし勘できめて調整してもいいかな。。


    float f = d / in;
    /*            */
    Vector a; //目標の加える rad/dt/dt
    a.x = -2*p.x/(c*c) - dt*dp.x/c ;
    a.y = -2*p.y/(c*c) - dt*dp.y/c ;
    a.z = -2*p.z/(c*c) - dt*dp.z/c ;

    //座標変換 (globalからlocal(モーターの軸へ))

    Serial << mwx::crlf;
    p.print();
    dp.print();
    Serial << a.x*10 << " " << a.y*10 << " " << a.z*10 << mwx::crlf;

    float t[3];
    //オイラーの運動方程式より
    Vector Io;
    Io.x = I[0][0]*dp.x + I[0][1]*dp.y + I[0][2]*dp.z;
    Io.y = I[1][0]*dp.x + I[1][1]*dp.y + I[1][2]*dp.z;
    Io.z = I[2][0]*dp.x + I[2][1]*dp.y + I[2][2]*dp.z;

    Io = dp.cross(Io);

    t[0] = I[0][0]*a.x + I[0][1]*a.y + I[0][2]*a.z + dt * Io.x;
    t[1] = I[1][0]*a.x + I[1][1]*a.y + I[1][2]*a.z + dt * Io.y;
    t[2] = I[2][0]*a.x + I[2][1]*a.y + I[2][2]*a.z + dt * Io.z;


    //q.print();
    Serial << t[0]*f << " " << t[1]*f << " " << t[2]*f << mwx::crlf;


    if(RED){
        t[0] = t[0]; 
        t[1] = -t[1];
        t[2] = -t[2];
    }else{
    }

    for(uint8_t i=0; i<3; i++){
        if((mp[i] + t[i] * f) < MMIN){
            Serial << " a" << mwx::crlf;
            mp[i] = MMIN;
        }else if(MMAX < (mp[i] + t[i] * f)){
            mp[i] = MMAX;
        }else{
            mp[i] += t[i] * f;
        }
        /*
        if(200 < mp[i]){
            recVal = t[i] * f/MMIND;
            mp[i] = 200 - MMIND;
        }
        */
        if(mp[i] < MNOR){
            mp[i] += MMIND;
        }else if( MNOR < mp[i]){
            mp[i] -= MMIND;
        }
    }

    //Serial.print(mp[0], OCT);
    //Serial.print(" ");
    //Serial.print(mp[0], OCT);
    //Serial.print(" ");
    //Serial.print(mp[0], OCT);
    //Serial.println(" ");

}

void makePacket(byte dat[DAT_LEN], byte q1[8], byte q2[8], uint16_t mp[3], uint8_t &index, byte &flags, uint8_t &solar){
    for(uint8_t j=0; j<8; j++){
        dat[j] = q1[j];
    }//0-7
    for(uint8_t j=0; j<8; j++){
        dat[8 + j] = q2[j];
    }//8-15

    //16,17
    //18 19
    //20 21
    memcpy(&dat[16], &mp[0], 2);
    memcpy(&dat[18], &mp[1], 2);
    memcpy(&dat[20], &mp[2], 2);
    dat[22] = index;

    dat[23] = flags;

    dat[24] = solar;
}

void printByte(byte buf[],uint8_t len){
    for(uint8_t i=0; i<len; i++){
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");
}
// valiable
uint8_t debugI8 = 0;
byte debugByte;
bool debugBool;
uint8_t index;
uint8_t solar;
uint8_t temp;
uint8_t recVal;
byte flags; // debug bno sd 
byte flags2; // debug bno sd 
byte buffer[8];
byte tmpBuff[8];
uint16_t mp[3];
byte dat[DAT_LEN];

Quat apos;
Quat rpos; //現在の -
Quat targetPos;
Vector gyro; //rad/s
Vector acce; //m/ss
Vector mag;
Vector pos;

/*** setup procedure (run once at cold boot) */
void setup() {
        /******     pin setup      ******/

    // pwmを落としておく これをしないとescの信号線を繋いでいるときになぜか電圧が下がってその後のプログラムが動作しない
    Timer1.setup();
    Timer1.begin(50,false,true);
    Timer1.change_duty(1);
    Timer3.setup();
    Timer3.begin(50,false,true);
    Timer3.change_duty(1);
    Timer4.setup();
    Timer4.begin(50,false,true);
    Timer4.change_duty(1);
    Analogue.setup();
    Analogue.begin(IN);
    pinMode(FLASH_BTN, INPUT);

    pinMode(BNO_RST_PIN, OUTPUT);
    digitalWrite(BNO_RST_PIN, LOW);
    delay(10);
    digitalWrite(BNO_RST_PIN, HIGH);

    Serial << "--- program start ---" << mwx::crlf;

        /****** valiable setup     ******/

    index = 0;
    flags = 0b00000000;
    flags2 = 0b00000000;
    mp[0] = MNOR;
    mp[1] = MNOR;
    mp[2] = MNOR;

        /****** setup spi   ******/

    SPI.begin(0 // DIO19をチップセレクトとして使用
              , { 5000000UL // クロック周波数 
            , SPI_CONF::MSBFIRST
            , SPI_CONF::SPI_MODE0
    });
    delay(1000);
    //ちなみにここで待たないとキャパシタと昇圧コンバータを利用したときに，bnoが起動できない(電圧が立ち上がるのに時間がかかる？)

    rotate(0, 0);
    rotate(1, 0);
    rotate(2, 0);
    if(auto&& trs = SPI.get_rwer()){
        byte deb;
        trs << 0x9F;
        delay(1);
        trs << 0x00;
        trs >> deb;
        trs << 0x00;
        trs >> deb;
        if(deb==0x9D){
            flags |= (1 << 2);
            Serial << "flash memory : ok" << mwx::crlf;
        }else{
            Serial << "flash memory : failed!" << mwx::crlf;
        }
    }
    // set target quaternion
    byte initDat[FDAT_LEN];
    f.read(0x000000, initDat, FDAT_LEN);
    bool a = false;
    for(uint8_t i=0; i<8; i++){
        if(initDat[i] != 0xFF){
            a = true;
        }
    }
    printByte(initDat, 10);
    if(a){
        targetPos.setFromByte(initDat);
    }else{
        Serial << "setup data not found" << mwx::crlf;
    }

     // さいしょは目標姿勢などを保存
    f.cAdr = 0x000000;
    if(digitalRead(FLASH_BTN) == LOW){
        flags |= (1<<6);
    }else{
        Serial.println("erasing");
        f.eraseAll();    
        flags &= ~(1<<6);
        f.write(f.cAdr, initDat, FDAT_LEN, true);
    }
    Serial.println(f.cAdr, OCT);

        /****** setup i2c   ******/

    Wire.begin();  
    delay(500);
    // bno setup ( NDOF mode https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
    if(bno.write(BNO_DEV, 0x3D, 0x0C)){
        bno.write(BNO_DEV, 0x3B, (1<<1)); //gyroの単位を設定(bnoのdatasheetの69page)
        flags |= (1 << 1);
        Serial << "bno : ok" << mwx::crlf;
    }else{
        Serial << "bno : failed!" << mwx::crlf;
    }
    bno.set(BNO_DEV, BNO_REG_G); //ジャイロのMSBにセット

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

    flags |= (1<<0);






}


void loop() { 
    if(!(flags&(1<<6))){
        if(50 < index){
            flags &= (0 << 0);
        }
        if(f.cAdr > 0x07FFFF){
            flags &= ~(1 << 0);
            Serial << "flashIsFull" << mwx::crlf;
            flags |= (1<<0);
        }
        memset(buffer, 0, 8);

        bno.set(BNO_DEV, BNO_CAL_DATA);
        bno.read(BNO_DEV, buffer , 1);
        flags &= ~(1 << 3);
        if((buffer[0]&(1<<6)) && (buffer[0]&(1<<7))){
            //bno calok
            flags |= (1<<3);
        }


        buffer[0] = index;
         f.write(index*33, buffer, 1, (bool)(flags & (1<<0)));
         //printByte(buffer,1);

        bno.set(BNO_DEV, BNO_REG_G); //ジャイロのMSBにセット
        bno.read(BNO_DEV, buffer, 6);
        gyro.setFromByte(buffer,0);
        //gyro.print();
         f.write(f.cAdr, buffer, 6, (bool)(flags & (1<<0)));
         ////printByte(buffer,6);
        bno.set(BNO_DEV, BNO_REG_Q); //姿勢のMSBにセット
        bno.read(BNO_DEV, buffer, 8);
        apos.setFromByte(buffer); //absolut posture bnoからの生の値
         f.write(f.cAdr, buffer, 8, (bool)(flags & (1<<0)));
         //printByte(buffer,8);
        if(flags&(1<<4)){
            targetPos.w = apos.w;
            targetPos.x = apos.x;
            targetPos.y = apos.y;
            targetPos.z = apos.z;
            f.write(0x000000, buffer, 8, true);
            flags &= ~(1<<4);
        }
        if(!(targetPos.w==0 && targetPos.x==0&& targetPos.y==0&& targetPos.z==0)){
            rpos = apos.concatenate(targetPos.conjugate());
        }else{
            rpos.w = apos.w;
            rpos.x = apos.x;
            rpos.y = apos.y;
            rpos.z = apos.z;
        }
        float fac = 2*acos(rpos.w) / sqrt(1 - rpos.w*rpos.w); //sin(sita/2)
        if(isnan(fac)){
            pos.x = 0;
            pos.y = 0;
            pos.z = 0;
        }else{
            pos.x = rpos.x * fac;
            pos.y = rpos.y * fac;
            pos.z = rpos.z * fac; 
        }

        //絶対姿勢の共役でまわせば機体の座標系になる
        pos.rotate(apos.conjugate());
        //gyroの座標系まだわからない たぶん機体座標系なんじゃないかと
        //gyro.rotate(apos.conjugate());

        //pos.print();
        //マジ計算する mp書き替え
        majiKeisanDekiru2(pos, gyro, mp, recVal);
        Serial.print(mp[0], OCT);
        Serial.print(" ");
        Serial.print(mp[1], OCT);
        Serial.print(" ");
        Serial.print(mp[2], OCT);
        Serial.println(" ");
        // 回転さす
        for(uint8_t i=0; i<3; i++){
            if(flags&(1<<5)){
                if(flags&(1<<6)){ //deb
                    rotate(i, mp[i]*2 - 5);
                }else{
                    rotate(i, mp[i]);
                }
            }else{
                rotate(i, 0);
            }
        }
        
        if(!((index+4) % 4)){ ///0,4,8でtrue
            solar = 0;
            if(Analogue.available())
                solar = (uint8_t)(Analogue.read(IN)-7);
        
        //  send packet

            makePacket(dat,tmpBuff,buffer,mp,index,flags,solar);
            send(REC_LID, dat);
        }
        //前回のデータを更新
        for(uint8_t i=0; i<8; i++){
            tmpBuff[i] = buffer[i];
        }
        bno.set(BNO_DEV, BNO_REG_A); 
        bno.read(BNO_DEV, buffer, 6);
        acce.setFromByte(buffer,1);
         f.write(f.cAdr, buffer, 6, (bool)(flags & (1<<0)));
         //printByte(buffer,6);

        bno.set(BNO_DEV, BNO_REG_M); 
        bno.read(BNO_DEV, buffer, 6);
        mag.setFromByte(buffer,2);
         f.write(f.cAdr, buffer, 6, (bool)(flags & (1<<0)));
         //printByte(buffer,6);

        bno.set(BNO_DEV, BNO_REG_T); 
        bno.read(BNO_DEV, buffer, 1);
        temp = buffer[0];
        if(temp == 0){
            bno.write(BNO_DEV, 0x3D, 0x0C);
        }
         f.write(f.cAdr, buffer, 1, (bool)(flags & (1<<0)));
         //printByte(buffer,1);

        memcpy(dat, &mp[0], 2);
        memcpy(&dat[4], &mp[1], 2);
        memcpy(&dat[8], &mp[2], 2);
         f.write(f.cAdr, dat, 6, (bool)(flags & (1<<0)));

        buffer[0] = solar;
         f.write(f.cAdr, buffer, 1, (bool)(flags & (1<<0)));
         //printByte(buffer,1);
        buffer[0] = flags;
         f.write(f.cAdr, buffer, 1, (bool)(flags & (1<<0)));
        buffer[0] = flags2;
         f.write(f.cAdr, buffer, 1, (bool)(flags & (1<<0)));
         //printByte(buffer,1);
         //Serial.println("");
        // send dataset
        if(flags&(1<<0)){
            index++;
            delay(33);
        }else{
            index = 0;
            delay(132);
        }

    }else{
        if(digitalRead(FLASH_BTN) == HIGH){
            if(f.cAdr <= 0x07FFFF){
                byte dat[FDAT_LEN];
                f.read(f.cAdr, dat, FDAT_LEN);
                //memset(dat, 0xAB, FDAT_LEN);
                //dat[0] = 0xFF;
                //Serial << dat;
                Serial.println(dat[0], OCT);

                f.cAdr += FDAT_LEN;
            }
        }else{
            f.cAdr = 0;
        }
        delay(15);
    }
}

void send(uint8_t adr, byte dat[]) {
    if (auto&& pkt = the_twelite.network.use<NWK_SIMPLE>().prepare_tx_packet()) {
        pkt << tx_addr(adr)  // 0..0xFF (LID 0:parent, FE:child w/ no id, FF:LID broad cast), 0x8XXXXXXX (long address)
            << tx_retry(0x0) // set retry (0x3 send four times in total)
            << tx_packet_delay(0,5,20); // send packet w/ delay (send first packet with randomized delay from 100 to 200ms, repeat every 20ms)
        pack_bytes(pkt.get_payload() // set payload data objects.
            , make_pair(dat, DAT_LEN) // string should be paired with length explicitly.
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
        flags |= (1<<0);
        Serial << "a" << mwx::crlf;

    }else if(com&(1<< 1)){
        flags |= (1<<4);

    }else if(com&(1<< 2)){
        flags |= (1<<5);

    }else if(com&(1<< 3)){
        flags &= ~(1<<5);

    }else if(com&(1<< 4)){
        

    }else if(com&(1<<5)){
        if(flags&(1<<7)){
            flags &= ~(1<<7);
        }else{
            flags |= (1<<7);
        }

    }
    /*
       0 記録開始(打ち上げ待機)
       1 目標姿勢指示
       2 姿勢制御強制始動
       3 姿勢制御強制停止
       4
       5 debug you
       */
    handled = true; //処理完了とする
}
    /* flags:
        0 doWriteData
       1 isbnoOk
       2 isflashOk
       3 isbno_calOk
        4 dosettargetQuat
        5 doposControl
       6 isReadOnlyMode
       */
/* -------------   memo  -------------


|   
|
|

flash capacity 500kB  
Quat 8
Acce 6
Gyro 6
Magn 6
inde 2
mp   3
flag 1
yobi 1
sola 1
Temp 1
sum  35 byte
14285 frame
14285 / 30(fps) = 476 sec 
8+6+6+6+2+3+1+1+1+1 = 

memory Dataset
inde    1+
Gyro    6+
Quat    8+
Acce    6+
Magn    6+
Temp    1+
mp      6+
sola    1+
flags   1+
flags2  1+

37 byte!



*/
