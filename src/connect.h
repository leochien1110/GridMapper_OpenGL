#ifndef CONNECT_H
#define CONNECT_H

#include <sys/socket.h>
#include <sys/types.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <iostream> 
#include <thread>
#include <mutex>
#include "data.h"

class Connect
{
public:
    Connect();

    ~Connect();
    
    void init(std::string,uint16_t,int,int,int,unsigned char (*_map)[30][100],float *);
    void start();
    void end();
    void senddata(bool);
    void recvdata();
    void sendmap();
    void sendcam();
    int process_sending(void * sendbuf, int SIZE, bool flag);
    int process_receiving(void * recvbuf, int SIZE);

    int sockfd = 0, Clientsockfd = 0;
    struct sockaddr_in serverInfo, clientInfo;
    unsigned int addrlen = sizeof(clientInfo);
    int ret = -1, sen = -1, err = -1, port_num = 0;
    std::string ip;

    char server_msg[30] = {"Hi, this is server.\n"};
    char client_msg[30] = {"Hi, this is client.\n"};
    
private:
    bool socket_connect = false;
    unsigned char (*map)[30][100];
    float *camera_pose;
    //int grid_x = 100, grid_y = 30, grid_z = 100;
    char voxel_map[100][30][100];
    std::mutex mutex;
};


#endif  //CONNECT_H