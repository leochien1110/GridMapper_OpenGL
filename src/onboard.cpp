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

    // run camera
    camera.init(width,height,fps);  // config devices and filter
    camera.start(data); //start threading stream 
    std::cout << camera.camera_pose[0] << " "   \
        << camera.camera_pose[1] << " "         \
        << camera.camera_pose[2] << " "         \
        << &camera.inv_C                  \
        << " " << std::endl;
    
    // run mapper
    Mapper mapper(camera.pc_vertices, camera.points,            \
                  camera.camera_pose, camera.specific_point,  \
                  camera.inv_C, width,height);
    mapper.start(data);

    // run sock
    data2GS.init(ip, port_num,   \
                100, 30, 100,  \
                mapper.voxelmap, camera.camera_pose);

    // send data to groundstation
    //std::thread t_send(&Connect::senddata, data2GS,     \
    //            mapper.mapper_status);

    Scene scene(mapper.unit_length, mapper.block_unit,  \
                camera.half_FOVxz, camera.half_FOVyz);
    
    scene.start(data);
    while(onboard_status)
    {
        // share data / data transfer
        
    }
}

void Onboard::stop()
{
    std::cout << "Onboard stop" << std::endl;
}