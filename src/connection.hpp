#ifndef CONNECTION_H
#define CONNECTION_H

#include <sys/socket.h>	//unix socket
#include <sys/types.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h> //close()
#include <thread>
#include <mutex>

class Connection
{
public:
    Connection(bool type, std::string addr);
    // parameters
    int sockfd = 0, Clientsockfd = 0;
    struct sockaddr_in serverInfo, clientInfo;
    unsigned int addrlen;
    int ret = -1, sen = -1, err = -1, port_num_c = 0;
    char message[30] = {"Hi, this is onboard.\n"};
    
    bool socket_connect = false;
    
    //functions
    void init(uint16_t port_num);
    void senddata(bool & mapper_status,std::mutex &mutex_t_m2g,
                unsigned char map[][30][100], int x, int y, int z,
                std::mutex &mutex_t_c2g, float camera_pose[]);
    void recvdata();
    void sendmap(std::mutex &mutex_t_m2g, unsigned char map[][30][100], int x, int y, int z);
    void sendcam(std::mutex &mutex, float camera_pose[]);
    int process_sending(void * sendbuf, int SIZE, bool flag);
    int process_receiving();
    void end();

private:
    bool socket_type;
    std::string sock_addr;
};

#endif // CONNECTION_H