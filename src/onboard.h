#ifndef ONBOARD_H
#define ONBOARD_H

#include <string>
#include <iostream>

#include "rs_camera.h"
#include "mapper.h"

class Onboard
{
public:
    Onboard(std::string gs_ip);
    ~Onboard();
    void update();
    void stop();

private:
    // int
    int width = 640, height = 480, fps = 30;

    // string
    std::string ip;

    // char

    // bool
    bool onboard_status;

    // Class
    RS_Camera camera;
};

#endif //ONBOARD_H