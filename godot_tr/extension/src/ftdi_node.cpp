#include "ftdi_node.h"
#include <unistd.h>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#define print UtilityFunctions::print

using namespace godot;

FtdiNode::FtdiNode(){

}
FtdiNode::~FtdiNode(){

}



int FtdiNode::init(int INTERFACE, int VENDOR, int PRODUCT, int BAUDRATE){
    unsigned char c1,c2;
    unsigned short stat;
    ftdic = ftdi_new();
    switch(INTERFACE){
        case 0:
            print(ftdi_set_interface(ftdic, INTERFACE_ANY));
            break;
        case 1:
            print(ftdi_set_interface(ftdic, INTERFACE_A));
            break;
        case 2:
            print(ftdi_set_interface(ftdic, INTERFACE_B));
            break;
        case 3:
            print(ftdi_set_interface(ftdic, INTERFACE_C));
            break;
        case 4:
            print(ftdi_set_interface(ftdic, INTERFACE_D));
            break;
    }
    print(ftdi_usb_open(ftdic, VENDOR, PRODUCT));



    print(ftdi_usb_purge_buffers(ftdic)); //これしないと modemStatusでrecieve overflow になる



    for(int i=0; i<8; i++){
        ftdi_usb_reset(ftdic);
    }
    print("\n");
    print(ftdi_get_latency_timer(ftdic, &c1));
    c1 = 0x02;
    print( ftdi_set_latency_timer(ftdic, c1) );
    print( ftdi_set_baudrate(ftdic, BAUDRATE) );
    print( ftdi_set_line_property(ftdic, BITS_8 , STOP_BIT_1 , NONE) );
    print( ftdi_setflowctrl(ftdic, 0) );
    c1 = 0x08;
    c2 = 0x00;
    print( ftdi_set_bitmode(ftdic, c1, c2));
    usleep(10000);
    print( ftdi_poll_modem_status(ftdic, &stat) );
    print("status : 0x", String::num_int64(stat, 16));
    for(int i=0; i<7; i++){
        ftdi_usb_reset(ftdic);
    }
    c1 = 0x2b;
    status = ftdi_write_data(ftdic, &c1, 1);

    return 0;
}
int FtdiNode::write(int c){
    unsigned char uc[1] = {(unsigned char)c};
    int result = ftdi_write_data(ftdic, uc, 1);
    return result;
}

PackedByteArray FtdiNode::read(int size){
    PackedByteArray byteArray;
    byteArray.resize(size);
    // write pointer of PackedByteArray
    int len = ftdi_read_data(ftdic, byteArray.ptrw(), size);
    if(0 < len){
        byteArray.resize(len);
        return byteArray;
    }
    //byteArray[0] = 0xaa;

    // empty
    return PackedByteArray();
}

int FtdiNode::isOK(){
    return FtdiNode::status;
}


void FtdiNode::_bind_methods(){
    ClassDB::bind_method(D_METHOD("init", "INTERFACE" , "VENDOR" , "PRODUCT" , "BAUDRATE"), &FtdiNode::init);
    ClassDB::bind_method(D_METHOD("read", "size"), &FtdiNode::read);
    ClassDB::bind_method(D_METHOD("write", "c"), &FtdiNode::write);
    ClassDB::bind_method(D_METHOD("isOK"), &FtdiNode::isOK);

}
