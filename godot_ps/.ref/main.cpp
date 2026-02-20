#include <GL/glut.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <thread>
#include <cmath>
#include <unistd.h>
#include <ftdi.h>

#define WINDOW_W 1200.0
#define VIEW_W 800.0
#define WINDOW_H 800.0
#define VIEW_W_RAT VIEW_W/WINDOW_W
#define ARRAY_MAX 10000000
#define BAUD_RATE 115200
#define MVENDOR_ID 0x0403
#define MPRODUCT_ID 0x6001
#define RVENDOR_ID 0x0403
#define RPRODUCT_ID 0x6015
#define INTERFACE INTERFACE_A
#define DAT_LEN 25
#define FDAT_LEN 37
#define SCALE 0.01
#define REALTIME false

using namespace std;

class Quat{
    public:
        float x;
        float y;
        float z;
        float w;
        void setFromByte(unsigned char dat[8]){
            int16_t x1, y1, z1, w1;
            x1 = y1 = z1 = w1 = 0;
            w1 = (((uint16_t)dat[1]) << 8) | ((uint16_t)dat[0]);
            x1 = (((uint16_t)dat[3]) << 8) | ((uint16_t)dat[2]);
            y1 = (((uint16_t)dat[5]) << 8) | ((uint16_t)dat[4]);
            z1 = (((uint16_t)dat[7]) << 8) | ((uint16_t)dat[6]);
            const float scale = (1.0 / (1 << 14));
            w = w1 *  scale;
            x = x1 *  scale;
            y = y1 *  scale;
            z = z1 *  scale;
        }
        void print(){
            cout << w << " "  << x << " " << y << " " << z << endl;
        }
};
class Vector{
    public:
        float x = 0;
        float y = 0;
        float z = 0;
        void setFromByte(unsigned char buffer[], bool isAcce){
            float scale;
            if(isAcce){
                scale = 16.0;
            }else{
                scale = 100.0;
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
            cout << x << " " << y << " " << z << endl;
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
float vertex[ARRAY_MAX][3];
float normal[ARRAY_MAX][3];
int lines[ARRAY_MAX];
int vertLen=0, normLen=0, lineLen=0;
int deb;
bool whichDat = false;
int datLenSum;

Quat q[2];
Vector gyro;
Vector acc;
Vector mag;
int temp;
int mp[3];
int datindex = 0;
int solar;
unsigned char flags;
unsigned char flags2;
char charIn = 0x00;
ofstream rawD;
ofstream fixedD;

//void loop(ftdi_context *ftdic);
 //受信した値を実用的な値にする
void byte2value(Quat q[2], int mp[3], int& datindex, unsigned char& flags, int &solar, unsigned char dat[DAT_LEN]){
    // 8*2  1*3*2  1 = 23
    unsigned char qByte[2][8];
    for(int i=0; i<2; i++){
        for(int j=0; j<8; j++){
            qByte[i][j] = dat[8*i + j];
        }
    } // 0 ~ 8+7(15)

    mp[0] = (int)(dat[16] << 8) | (int)(dat[17]);
    mp[1] = (int)(dat[18] << 8) | (int)(dat[19]);
    mp[2] = (int)(dat[20] << 8) | (int)(dat[21]);

    int ddatindex = (int)dat[22] - datindex%256;
    if(ddatindex < 0){
        ddatindex += 256;
    }
    datindex += ddatindex; //22

    flags = dat[23]; //22

    solar = (int)dat[24]; //22

    for(int i=0; i<2; i++){
        q[i].setFromByte(qByte[i]);
    }
    for(int i=0; i<2; i++){
        if(q[i].w < 0){
            q[i].w = -q[i].w;
            q[i].x = -q[i].x;
            q[i].y = -q[i].y;
            q[i].z = -q[i].z;
        }
    }

}
void byte2value2(Quat &q,int mp[3],Vector &gyro,Vector &acc,Vector &mag, int temp, int& datindex,unsigned char &flags,unsigned char &flags2, int &solar,unsigned char dat[FDAT_LEN]){
    int ddatindex = (int)dat[0] - datindex%256;
    if(ddatindex < 0){
        ddatindex += 256;
    }
    datindex += ddatindex; //0

    unsigned char qByte[8];

    for(int i=0; i<6; i++){
        qByte[i] = dat[1+i];
    } // 1 - 6
    gyro.setFromByte(qByte, false);

    for(int i=0; i<8; i++){
        qByte[i] = dat[7+i];
    }// 7 -14
    q.setFromByte(qByte);

    for(int i=0; i<6; i++){
        qByte[i] = dat[15+i];
    } // 15 - 20
    acc.setFromByte(qByte, true);

    for(int i=0; i<6; i++){
        qByte[i] = dat[21+i];
    } // 21 - 26
    mag.setFromByte(qByte, false);

    temp = (int)dat[27];

    mp[0] = (int)(dat[28] << 8) | (int)(dat[29]);
    mp[1] = (int)(dat[30] << 8) | (int)(dat[31]);
    mp[2] = (int)(dat[32] << 8) | (int)(dat[33]);

    solar = (int)dat[34]; //22

    flags = dat[35]; //22

    flags2 = dat[36]; //22
}
void initMatrix(){
    glLoadIdentity();
    glScalef(SCALE,SCALE,SCALE);
}
void showStr(string const &str,float x,float y,float z,float r,float g,float b){
    const char* cstr = str.c_str();
    unsigned short stat;
    glColor3f(r,g,b);
    glLoadIdentity();
    glRasterPos3f(x, y, z);
    for(int i=0; i<str.length(); i++){
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, cstr[i]);
    }
}
int initTwelite(){
    struct ftdi_context *ftdic;
    unsigned char c1,c2;
    unsigned short stat;
    ftdic = ftdi_new();
    cout << ftdi_set_interface(ftdic, INTERFACE) << endl;
    if(REALTIME){
        cout << ftdi_usb_open(ftdic, MVENDOR_ID, MPRODUCT_ID) << endl;
    }else{
        cout << ftdi_usb_open(ftdic, RVENDOR_ID, RPRODUCT_ID) << endl;
    }

    cout << ftdi_usb_purge_buffers(ftdic) << endl; //これしないと modemStatusでrecieve overflow になる
    for(int i=0; i<8; i++){
        cout << ftdi_usb_reset(ftdic);
    }
    cout << endl;
    cout << ftdi_get_latency_timer(ftdic, &c1) << endl;
    c1 = 0x02;
    cout << ftdi_set_latency_timer(ftdic, c1) << endl;
    cout << ftdi_set_baudrate(ftdic, BAUD_RATE) << endl;
    cout << ftdi_setflowctrl(ftdic, 0) << endl;
    c1 = 0x08;
    c2 = 0x00;
    cout << ftdi_set_bitmode(ftdic, c1, c2);
    usleep(100000);
    cout << ftdi_poll_modem_status(ftdic, &stat) << endl;
    printf("status : 0x%x\n", stat);
    for(int i=0; i<7; i++){
        cout << ftdi_usb_reset(ftdic);
    }
    cout << endl;
    cout << ftdi_read_data(ftdic, &c1, 4) << endl;

    if(REALTIME){
        rawD.open("./realtimeDatRaw.txt");
        fixedD.open("./realtimeDatFixed.txt");
    }else{
        rawD.open("./flashDatRaw.txt");
        fixedD.open("./flashDatFixed.txt");
    }

    while(1){
        unsigned char dat[FDAT_LEN];
        memset(dat, 0, FDAT_LEN);
        int datLen = 0;
        if(REALTIME){
            datLen = ftdi_read_data(ftdic, dat, DAT_LEN);
        }else{
            datLen = ftdi_read_data(ftdic, dat, FDAT_LEN);
        }
        datLenSum += datLen;
        if(0 < datLen){
            rawD << dat;
            //cout << dat << endl;
            if(REALTIME){
                byte2value(q, mp, datindex,flags, solar, dat);
                fixedD << datindex << " ";
                for(int i=0; i<8;i++){
                    fixedD << (1&(flags>>i));
                }
                fixedD << " " << solar << "  " 
                    <<mp[0]<<" "<<mp[1]<<" "<<mp[2]
                    <<"  "<<q[0].w<<"  "<<q[0].x<<"  "<<q[0].y<<"  "<<q[0].z
                    <<"  "<<q[1].w<<"  "<<q[1].x<<"  "<<q[1].y<<"  "<<q[1].z
                    <<endl;
            }else{
                rawD << endl;
                cout << sizeof(dat) << endl;
                if(sizeof(dat) == FDAT_LEN){
                    byte2value2(q[1], mp, gyro, acc, mag, temp, datindex,flags,flags2, solar, dat);
                    fixedD << datindex << " " ;
                    for(int i=0; i<8;i++){
                        fixedD << (1&(flags>>i));
                    }
                    fixedD << " ";
                    for(int i=0; i<8;i++){
                        fixedD << (1&(flags2>>i));
                    }
                    fixedD<< " " << solar << "  " 
                        <<mp[0]<<" "<<mp[1]<<" "<<mp[2]
                        <<"  "<<q[1].w<<"  "<<q[1].x<<"  "<<q[1].y<<"  "<<q[1].z
                        <<"  "<<gyro.x<<"  "<<gyro.y<<"  "<<gyro.z
                        <<"  "<<acc.x <<"  "<<acc.y <<"  "<<acc.z
                        <<"  "<<mag.x <<"  "<<mag.y <<"  "<<mag.z
                        <<"  " << temp
                        <<endl;
                }else{
                    //fixedD << dat
                }
                // writing !!!
                if( 499990 <= datLenSum){
                    cout << "all data loaded" << endl;
                }
            }
            whichDat = true;
            if(REALTIME){
                usleep(66000);
                whichDat = false;
            }
        }
        if(charIn != 0x00){
            unsigned char c[1] = {charIn};
            cout << ftdi_write_data(ftdic, c, 1) << endl;
            charIn = 0x00;
        }
        usleep(1000);
    }
    return 0;
}

void resize(int w, int h){
}

void display(){
    //memset(dat, 0, 25);
    glClear(GL_COLOR_BUFFER_BIT); 
    glViewport(0, 0, VIEW_W, WINDOW_H);
    initMatrix();

    glRotatef(45,1.0,0,0);
    if(whichDat){
        glRotatef(acos(q[0].w)*360/3.14,q[0].x,q[0].y,q[0].z);
    }else{
        glRotatef(acos(q[1].w)*360/3.14,q[1].x,q[1].y,q[1].z); //old data
    }

    //glEnableClientState(GL_VERTEX_ARRAY);
    //glVertexPointer(3,GL_FLOAT,0,vertex);
    // 頂点の描画
    glPointSize(3);
    glBegin(GL_POINTS);
    {
        for(int i=0;i<vertLen;i++)
            glVertex3fv(vertex[i]);
    }
    glEnd();

    glBegin(GL_TRIANGLES);
        for(int i=0;i<lineLen;i++){
            glColor3f(0,(normal[i][1]+normal[i][2]*1.5)/2,0);
            glVertex3fv(vertex[lines[i*3]]);
            glVertex3fv(vertex[lines[i*3+1]]);
            glVertex3fv(vertex[lines[i*3+2]]);
        }
    glEnd();

    /*
    glColor3f(1.0,0.75,0);
    glBegin(GL_LINES);
    {
        for(int i=0;i<lineLen;i++){
            glArrayElement(lines[i*3]);
            glArrayElement(lines[i*3+1]);

            glArrayElement(lines[i*3+1]);      
            glArrayElement(lines[i*3+2]);

            glArrayElement(lines[i*3+2]);
            glArrayElement(lines[i*3]);
        }
    }
    glEnd();
    */
    deb++;
    const float step = 0.08;
    float yPos = 0.6;
    showStr("datindex:" + to_string(datindex), VIEW_W_RAT,yPos,0,1,1,1);
    yPos -= step;
    if(flags & (1<<0)){
        showStr("go/nogo: go ", VIEW_W_RAT,yPos,0,1,1,0);
    }else{
        //cout << flags << endl;
        showStr("go/nogo: nogo ", VIEW_W_RAT,yPos,0,1,0,0);
    }
    yPos -= step;
    if(flags & (1<<1)){
        showStr("sensor: ok ", VIEW_W_RAT,yPos,0,0,1,0);
    }else{
        showStr("sensor: failed! ", VIEW_W_RAT,yPos,0,1,0,0);
    }
    yPos -= step;
    if(flags & (1<<2)){
        showStr("memory: ok ", VIEW_W_RAT,yPos,0,0,1,0);
    }else{
        showStr("memory: failed! ", VIEW_W_RAT,yPos,0,1,0,0);
    }
    yPos -= step;
    if(flags & (1<<3)){
        showStr("bno_calibration: ok ", VIEW_W_RAT,yPos,0,0,1,0);
    }else{
        showStr("bno_calibration: ng ", VIEW_W_RAT,yPos,0,1,0,0);
    }
    yPos -= step;
    if(flags & (1<<5)){
        showStr("posControl: running ", VIEW_W_RAT,yPos,0,1,1,0);
    }else{
        showStr("posControl: stopping ", VIEW_W_RAT,yPos,0,1,0,0);
    }
    yPos -= step;
    showStr("solar: " + to_string(solar), VIEW_W_RAT,yPos,0,1,1,1);
    yPos -= step;
    showStr("quaternion1: " + to_string(q[1].w) + " " + to_string(q[1].x) + " " + to_string(q[1].y) + " " + to_string(q[1].z) + " ", VIEW_W_RAT,yPos,0,1,1,1);
    yPos -= step;
    showStr("mp: " + to_string(mp[0]) + " " + to_string(mp[1]) + " " + to_string(mp[2]), VIEW_W_RAT,yPos,0,1,1,1);
    glFlush();
}

void init3dData(){
    FILE *mDat;
    mDat = fopen("./FM.txt","r");

    if(mDat==NULL){
        cout << "file open failed" << endl;
        return;
    }
    char c[2];
    int tmp[6];
    vertLen = normLen = lineLen = 0;
    while(true){
        if((fscanf(mDat, "%c%c", &c[0],&c[1])==EOF)){
            break;
        }else if(c[0] == 'v' && c[1] == ' '){
            fscanf(mDat, "%f %f %f\n", &vertex[vertLen][0],&vertex[vertLen][1],&vertex[vertLen][2]);
            vertLen++;
        }else if(c[0] == 'v' && c[1] == 'n'){
            fscanf(mDat, " %f %f %f\n", &normal[normLen][0],&normal[normLen][1],&normal[normLen][2]);
            normLen++;
        }else if(c[0] == 'f' && c[1] == ' '){
            fscanf(mDat, "%d//%d %d//%d %d//%d\n", &tmp[0], &tmp[3], &tmp[1],&tmp[4], &tmp[2], &tmp[5]);
            lines[lineLen*3]   = tmp[0] - 1;
            lines[lineLen*3+1] = tmp[1] - 1;
            lines[lineLen*3+2] = tmp[2] - 1;
            lineLen++;
        }else{
            cout << "オカシ " << c << endl;
        }
    }
}

void keyType(unsigned char key, int x, int y){
    charIn = key;
    switch(key){
        case '\033': //escのascii code
            //ftdi_usb_close(ftdic);
            //ftdi_free(ftdic);
            exit(0);
            rawD.close();
            fixedD.close();
            break;
        default:
            break;
    }
}

void initUI(){
    glClearColor(0.1, 0.1, 0.2, 1.0);
}
void timer(int x)
{
    glutPostRedisplay();
    glutTimerFunc(66,timer,0);
}


int main(int argc, char *argv[]){
    
    init3dData();

    cout << vertex[2] << endl;

    glutInit(&argc, argv);
    glutInitWindowPosition(0,0);
    glutInitWindowSize(WINDOW_W, WINDOW_H);
    glutInitDisplayMode(GLUT_SINGLE|GLUT_RGBA);
    glutCreateWindow("fksB test");

    glutDisplayFunc(display);
    glutReshapeFunc(resize);
    glutKeyboardFunc(keyType);
    glutTimerFunc(66,timer,0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0);

    initUI();


    //ここから,ここから
    //マルチスレッドだとォ
    thread th1(initTwelite);



    glutMainLoop();
    th1.join();
    //ftdi_usb_close(ftdic);
    //ftdi_free(ftdic);

    return 0;
}


