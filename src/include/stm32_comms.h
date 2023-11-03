#pragma once

#include <libserial/SerialPort.h>
#include <iostream>

class Stm32Comms
{
public:

    void connect(const std::string &serial_device, int16_t timeout_ms);

    bool check_connected();

    void disconnect();

    void send_rad_velo(float velo_l, float velo_r);

    void read_rad_velo_pos(float &velo_l, float &velo_r,float &pos_l, float &pos_r);

    

private:

    static const int output_array_length_ = 8;
    static const int input_array_length_ = 24;
    int16_t timeout_ms_;
    LibSerial::SerialPort serial_port_;
    char output_raw_[output_array_length_];
    //char input_raw_[input_array_length_];
    std::string input_raw_;
    float velo_l_;
    float velo_r_;

    float data_stitch_2_(uint8_t Data_High,uint8_t Data_Low);
    unsigned char check_sum_(unsigned char Count_Number,bool recive_mode);
    

};