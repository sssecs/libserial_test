#include "include/stm32_comms.h"


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
    short temp;
    this->output_raw_[0]=0x7B; //frame head 0x7B //帧头0X7B
    this->output_raw_[1] = 0; //set aside //预留位
    this->output_raw_[2] = 0; //set aside //预留位

    //The target velocity of the X-axis of the robot
    //机器人x轴的目标线速度
    temp = velo_l*1000; //将浮点数放大一千倍，简化传输
    this->output_raw_[4] = temp;     //取数据的低8位
    this->output_raw_[3] = temp>>8;  //取数据的高8位

    //The target velocity of the Y-axis of the robot
    //机器人y轴的目标线速度
    temp = velo_r*1000;
    this->output_raw_[6] = temp;
    this->output_raw_[5] = temp>>8;

    //The target angular velocity of the robot's Z axis
    //机器人z轴的目标角速度
    this->output_raw_[8] = 0;
    this->output_raw_[7] = 0;

    this->output_raw_[9]=this->check_sum_(9,false); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
    this->output_raw_[10]=0x7D; //frame tail 0x7D //帧尾0X7D

    int i;
    for (i=0;i<this->output_array_length_;i++)
    {
        this->serial_port_.WriteByte(this->output_raw_[i]);
    }
}


void Stm32Comms::read_rad_velo_pos(double &velo_l, double &velo_r)
{
    
    try
    {   
        //this->serial_port_.Read( this->input_raw_, this->input_array_length_, this->timeout_ms_ );
        this->serial_port_.ReadLine(this->input_raw_, '\n', this->timeout_ms_);
        if (this->input_raw_.size() ==13)//((this->input_raw_[22] == this->check_sum_(22,true))) 
        {
            std::string delimiter = " ";
            size_t del_pos = this->input_raw_.find(delimiter);
            std::string token_1 = this->input_raw_.substr(0, del_pos);
            std::string token_2 = this->input_raw_.substr(del_pos + delimiter.length(),input_raw_.size()-1);

            velo_l = std::atoi(token_1.c_str());
            velo_r = std::atoi(token_2.c_str());
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