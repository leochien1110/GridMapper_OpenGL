#include "onboard.h"

Onboard::Onboard(std::string gs_ip)
{
    ip = gs_ip;
    std::cout << "ip: " << ip << std::endl;
    onboard_status = true;
    
    std::cout << "ob.dc.specific_row:" << specific_row << std::endl;
    specific_row = 14;
    std::cout << "ob.dc.a:" << Data::a << std::endl;
    Data::a = 444;
}

Onboard::~Onboard()
{
    std::cout << "Onboard deconstructing..." << std::endl;
    stop();
}

void Onboard::update()
{
    std::cout << "Onboard updating...\n" << std::endl;
    std::cout << "ob.updata.specific_row:" << specific_row << std::endl;
    specific_row = 13;
    std::cout << "ob.updata.a:" << Data::a << std::endl;
    Data::a = 555;
    // run flight controller

    // run camera
    camera.init();  // config devices and filter
    camera.start();
    //camera.start(data); //start threading stream 
    //std::cout << camera.camera_pose[0] << " "   \
        << camera.camera_pose[1] << " "         \
        << camera.camera_pose[2] << " "         \
        << &camera.inv_C                  \
        << " " << std::endl;
    
    // run mapper
    Mapper mapper(camera.pc_vertices, camera.points,            \
                  camera.camera_pose, camera.specific_point,  \
                  camera.inv_C, width,height);
    //mapper.start(data);
    mapper.start();

    // run sock
    data2GS.init(ip, port_num,   \
                100, 30, 100,  \
                mapper.voxelmap, camera.camera_pose);
    //data2GS.start(data);
    // send data to groundstation
    //std::thread t_send(&Connect::senddata, data2GS,     \
    //            mapper.mapper_status);

    Scene scene(mapper.unit_length, mapper.block_unit,  \
                camera.half_FOVxz, camera.half_FOVyz);
    //scene.update(camera.camera_pose,    \
                100, 30, 100, mapper.voxelmap,  \
                mapper.camera_scaled_pose, mapper.unit_length_x);
    //scene.start(data);
    while(onboard_status)
    {
        // share data / data transfer
        
    }
}

void Onboard::stop()
{
    std::cout << "Onboard stop" << std::endl;
}