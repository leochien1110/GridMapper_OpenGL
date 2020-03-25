#ifndef ONBOARD_H
#define ONBOARD_H

#include <string>
#include <iostream>

#include "rs_camera.h"
#include "mapper.h"
#include "connect.h"
#include "scene.h"
#include "data.h"

class Onboard
{
public:
    Onboard(std::string);
    ~Onboard();
    void update();
    void stop();

private:
    // int
    int width = 848, height = 480, fps = 30;

    // string
    std::string ip;
    uint16_t port_num;

    // char

    // bool
    bool onboard_status;

    // Class
    RS_Camera camera;
    Connect data2GS;
    //Scene scene;
};

#endif  //ONBOARD_H