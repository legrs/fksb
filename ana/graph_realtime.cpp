// inherited ~/tools/graph.cpp

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include <simple_svg_1.0.0.hpp>

#define T_LIMIT 1000
#define X_LIMIT 100
#define MAIN_LINE_WIDTH 0.3
#define AXIS_LINE_WIDTH1 0.1
#define AXIS_LINE_WIDTH2 0.3
#define AXIS_LINE_WIDTH3 0.5
#define AXIS_COLOR 150
#define INPUT_FILE_PATH "../record/realtime/260222-2058.csv"

//index,flag1,flag2,orimp ( Big-Endian )

using namespace std;
using namespace svg;

void split(string str, char separator, string out[]){
    int j=0;
    int size = str.size();
    for(int i=0; i<size; i++){
        if(str[i]==separator){
            j++;
        }else{
            out[j].push_back(str[i]);
        }
    }
    return;
}

unsigned char str2bin(string str, bool is_bigendian){
    unsigned char uc = 0x00;
    int size = str.size();
    for(int i=0; i<size; i++){
        if(is_bigendian){
            if(str[i]=='1'){
                uc |= (1<<(7-i));
            }
        }else{
            if(str[i]=='1'){
                uc |= (1<<i);
            }
        }
    }
    return uc;
}

int main(){
    Document doc("fg.svg", Layout(Dimensions(T_LIMIT,X_LIMIT), Layout::BottomLeft));

    Color grayColor = Color(AXIS_COLOR,AXIS_COLOR,AXIS_COLOR);

    doc << Rectangle("back",Point(0, 0), T_LIMIT, X_LIMIT, Fill(Color::White),Stroke());
    Path p_w = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Black));
    Path p_k = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Black));
    Path p_b = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Blue));
    Path p_g = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Green));
    Path p_r = Path(Fill(), Stroke(MAIN_LINE_WIDTH,Color::Red));

    // t axis
    for(float i=0; i<=T_LIMIT; i+=10){
        float width;
        if((int)i%50 == 0){
            if((int)1%100 == 0){
                width = AXIS_LINE_WIDTH3;
            }else{
                width = AXIS_LINE_WIDTH2;
            }
        }else{
            width = AXIS_LINE_WIDTH1;
        }
        doc << Line("l",Point(i,0),Point(i,X_LIMIT),Stroke(width,grayColor));
    }
    // x axis
    for(float i=0; i<=X_LIMIT; i+=10){
        float width;
        if((int)i%50 == 0){
            if((int)1%100 == 0){
                width = AXIS_LINE_WIDTH3;
            }else{
                width = AXIS_LINE_WIDTH2;
            }
        }else{
            width = AXIS_LINE_WIDTH1;
        }
        doc << Line("l",Point(0,i),Point(T_LIMIT,i),Stroke(width,grayColor));
    }

    vector<float> time;
    string tmp;
    vector<float> ori[4];
    vector<float> avel[3];
    vector<float> acce[3];
    vector<float> mp[3];
    vector<int> index;
    vector<unsigned char> flag[2];
    vector<float> temp;

    int dat_index = 0;

    // substitue data to vector
    fstream csv(INPUT_FILE_PATH, ios::in);
    //dump
    getline(csv, tmp);
    getline(csv, tmp);
    while(getline(csv, tmp)){
        string dat_string[8];
        split(tmp, ',', dat_string);
        
        index.push_back(stoi(dat_string[0]));
        flag[0].push_back(str2bin(dat_string[1], 1));
        flag[1].push_back(str2bin(dat_string[2], 1));

        cout << dat_index << endl;
        cout << tmp << endl;


        string dat_string_tmp[4];
        split(dat_string[3], ' ', dat_string_tmp);
        for(int i=0; i<4; i++){
            ori[i].push_back(stof(dat_string_tmp[i]));
        }
        dat_string_tmp[0] = "";
        dat_string_tmp[1] = "";
        dat_string_tmp[2] = "";
        dat_string_tmp[3] = "";
        split(dat_string[4], ' ', dat_string_tmp);
        for(int i=0; i<3; i++){
            mp[i].push_back(stof(dat_string_tmp[i]));
        }

        dat_index++;
    }
    csv.close();


    // plot & cal
    int size = index.size();
    for(int i=0; i<size; i++){
        int t,x;
        t = i;

        x = mp[0][i]*X_LIMIT/200;
        p_r << Point((float)t, (float)x);
        x = mp[1][i]*X_LIMIT/200;
        p_g << Point((float)t, (float)x);
        x = mp[2][i]*X_LIMIT/200;
        p_b << Point((float)t, (float)x);

        x = index[i]*X_LIMIT/255;
        p_k << Point((float)t, (float)x);


    }
    
    
    doc << p_w;
    doc << p_k;
    doc << p_b;
    doc << p_g;
    doc << p_r;


    doc.save();
    return 0;
}
/*
NaD:213

*/
