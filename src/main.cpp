#include <iostream>
#include "include/stm32_comms.h"
int main()
{
    float velol, velor;
    float posl, posr;
    Stm32Comms comms;
    std::cout << "Runing" << std::endl;
    enum State {connect, read};
    State my_state = connect;
    while(true)
    {
        switch (my_state)
        {
        case connect:
            comms.connect("/dev/ttyACM0",200);
            std::cout << "Trying" << std::endl;
            if (comms.check_connected())
            {
                my_state = read;
                std::cout << "configed" << std::endl;
            }
            break;
        
        case read:
            comms.read_rad_velo_pos(velol,velor,posl,posr);
            std::cout << velol << ';' << velor << " "<< posl << ";" <<posr << std::endl;
            comms.send_rad_velo((float) -3.14,(float) 3.14);
            break;
        }
    }
}