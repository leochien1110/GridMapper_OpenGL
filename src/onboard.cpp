#include "onboard.h"

Onboard::Onboard(std::string gs_ip)
{
    ip = gs_ip;
    std::cout << "ip: " << ip << std::endl;
    onboard_status = true;
}

Onboard::~Onboard()
{
    stop();
}
void Onboard::update()
{
    std::cout << "Onboard updating..." << std::endl;
    // run flight controller

    // run sock

    // run camera
    camera.init(width,height,fps);  // config devices and filter
    camera.start(); //start threading stream 

    // run mapper
    Mapper mapper(camera.pc_vertices, camera.points,            \
                  camera.camera_pose, camera.specific_point,  \
                  camera.inv_C, width,height);
    mapper.start();

    while(onboard_status)
    {
        // share data / data transfer
    }
}

void Onboard::stop()
{
    std::cout << "Onboard stop" << std::endl;
}