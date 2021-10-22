#include <iostream>
#include <librealsense2/rs.hpp>

#include <iomanip>
#include <string.h>
#include <math.h>
#include <thread>
#include <chrono>
#include <mutex>

#include "onboard.h"

int main(int argc, char * argv[]) try
{
    std::cout << "Hello px4!" << std::endl;
    
    // check ip address
    if(!argv[1])
    {
        std::cout << "IP address required!" << std::endl;
        return 0;
    }

    //call onboard code
    Onboard onboard(argv[1]);

    onboard.update();

    std::cout << "code terminating..." << std::endl;

    return 0;
}

catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}