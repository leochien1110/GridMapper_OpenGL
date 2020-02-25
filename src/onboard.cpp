#include <sys/socket.h>
#include <sys/types.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <thread>
#include <mutex>

#include "connection.hpp"
#include "scene.hpp"
#include "mapper.hpp"
#include "sensor.hpp"

char voxel_map[100][30][100];

int main(int argc, char * argv[]) try
{
    Connection px2gs(1,argv[1]);
    px2gs.init(6666);

    const int width = 848;	//1280,848,640,480, 424
    const int height = 480;	//720,480,360,270,240
    const int desired_fps = 60;
    Mapper voxel_mapper(width, height, desired_fps, voxel_map);

    px2gs.end();

}

catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
