#include "include/stm32_comms.h"
#define PI           3.14159265358979323846


unsigned char Stm32Comms::check_sum_(unsigned char Count_Number,bool recive_mode)
{
  unsigned char check_sum=0,k;
  
  if(recive_mode) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^this->input_raw_[k]; //By bit or by bit //按位异或
     }
  }
  if(not recive_mode) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^this->output_raw_[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}


float Stm32Comms::data_stitch_2_(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
  transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}

void Stm32Comms::connect(const std::string &serial_device, int16_t timeout_ms)
{
    this->timeout_ms_ = timeout_ms;
    try
    {
        // Open the Serial Port at the desired hardware port.
        serial_port_.Open(serial_device) ;
    }
    catch (const LibSerial::OpenFailed&)
    {
        std::cerr << "The serial port did not open correctly." << std::endl ;
    }

    // Set the baud rate of the serial port.
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200) ;

    // Set the number of data bits.
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8) ;

    // Turn off hardware flow control.
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE) ;

    // Disable parity.
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE) ;
    
    // Set the number of stop bits.
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1) ;
    
}

bool Stm32Comms::check_connected()
{
    return this->serial_port_.IsOpen();
}

void Stm32Comms::disconnect()
{
    serial_port_.Close();
}


void Stm32Comms::send_rad_velo(float velo_l, float velo_r)
{
    int count_l,count_r;
    count_l = (int)(velo_l/(2*PI)*60000/100);
    count_r = (int)(velo_r/(2*PI)*60000/100);

    this->output_raw_[0]='{'; //frame head 0x7B //帧头0X7B
    this->output_raw_[1] = count_l/10000; 
    this->output_raw_[2] = (count_l-count_l/10000*10000)/100; 
    this->output_raw_[3] = count_l%100; 
    this->output_raw_[4] = count_r/10000; 
    this->output_raw_[5] = (count_r-count_r/10000*10000)/100; 
    this->output_raw_[6] = count_r%100; 

    this->output_raw_[7]='}'; //frame tail 0x7D //帧尾0X7D

    int i;
    for (i=0;i<this->output_array_length_;i++)
    {
        this->serial_port_.WriteByte(this->output_raw_[i]);
    }
}


void Stm32Comms::read_rad_velo_pos(float &velo_l, float &velo_r,float &pos_l, float &pos_r)
{
    
    try
    {   
        
        this->serial_port_.ReadLine(this->input_raw_, '}', this->timeout_ms_);
        if (this->input_raw_.size() ==15)
        {
 	    velo_l = input_raw_[2]*10000 + input_raw_[3]*100 + input_raw_[4];
	    velo_r = input_raw_[5]*10000 + input_raw_[6]*100 + input_raw_[7];
        pos_l = input_raw_[8]*10000 + input_raw_[9]*100 + input_raw_[10];
	    pos_r = input_raw_[11]*10000 + input_raw_[12]*100 + input_raw_[13];
        
        }
        else
        {
            velo_l = -999;
            velo_r = -999;

        }

    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The Read() call timed out waiting for additional data." << std::endl ;
    }
}
