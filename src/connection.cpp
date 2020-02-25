#include "connection.hpp"
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <iostream>

Connection::Connection(bool type, std::string addr)
{
    printf("Socket called!\n");
    //std::cout << "address:" << addr << std::endl; 
    sock_addr = addr;

    if(type == 0)
    {
        printf("UDP enable\n");
        socket_type = type;
    }
    else
    {
        printf("TCP enable\n");
        socket_type = type;
    }
}

void Connection::init(uint16_t port_num)
{
    if(socket_type == 0)
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    else if(socket_type == 1)
        sockfd = socket(AF_INET, SOCK_STREAM, 0); 
    if(sockfd == -1)
        printf("Fail to create a socket!\n");
    
    port_num_c = port_num;
    char char_addr[sizeof(sock_addr)+1];
    strcpy(char_addr, sock_addr.c_str());
    printf("sock_addr:%s\n",char_addr);

    int nSendBuf=512*1024;//512K
    setsockopt(sockfd,SOL_SOCKET,SO_SNDBUF,(const char*)&nSendBuf,sizeof(int));
    //int set = 1;
    //setsockopt(sockfd, SOL_SOCKET, MSG_NOSIGNAL, (void *)&set, sizeof(int));
    signal(SIGPIPE, SIG_IGN);
    //************************
    bzero(&serverInfo,sizeof(serverInfo));
    serverInfo.sin_family = PF_INET;
    // local host test
    serverInfo.sin_addr.s_addr = inet_addr(char_addr);	//127.0.0.1		104.39.160.46	104.39.89.30
    serverInfo.sin_port = htons(port_num);
    err = connect(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
    if(err==-1){
        printf("Connection error\n");
        socket_connect = false;
    }
    else{
        printf("Connected!\n");
        socket_connect = true;
    }
}

void Connection::senddata(bool & mapper_status,std::mutex &mutex_t_m2g,
                        unsigned char map[][30][100], int x, int y, int z,
                        std::mutex &mutex_t_c2g, float camera_pose[])
{
    while(mapper_status){
        if(!socket_connect){
            printf("\033[1A"); //go back to previous row
            printf("\033[K");  //flush
            printf("\033[1A");
            printf("\033[K");
            printf("\033[1A");
            printf("\033[K");
            end();
            printf("Reinitializing...\n");
            init(port_num_c);
            std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(100));
        }
        else{
            clock_t t01;
            sendmap(mutex_t_m2g, map, x, y, z);
            sendcam(mutex_t_c2g,camera_pose);
            clock_t t04;
            //double duration = t04 - t01;
            //std::cout<< "\r" << "time spend: " << duration << std::flush;
        }
    }
}

void Connection::recvdata()
{

}

void Connection::sendmap(std::mutex &mutex, unsigned char map[][30][100], int x, int y, int z)
{
    unsigned char buf[x][y][z];
    int buf_size = sizeof(buf);
    //std::lock_guard<std::mutex> mlcok(mutex_t_s2g);
    mutex.lock();
    for (int i = 0; i < x; i++) {
        for (int j = 0; j < y; j++) {
            for (int k = 0; k < z; k++) {
                buf[i][j][k] = map[i][j][k];
            }
        }
    }
    mutex.unlock();
    if(err != -1){
        process_sending(buf, buf_size, 0);
    }
    //printf("sendmap sent: %d\n", sen);
    //std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(50));
}

void Connection::sendcam(std::mutex &mutex, float camera_pose[])
{
    float buf[9];
    int buf_size = sizeof(buf);
    //std::lock_guard<std::mutex> mlcok(mutex_t_c2g);
    mutex.lock();
    for (int i = 0; i < 9; i++) {
        buf[i]=camera_pose[i];
    }
    mutex.unlock();
    if(err != -1){
        process_sending(buf, buf_size, 1);
    }		
    //printf("sendcam sent: %d\n", sen);
}

int Connection::process_sending(void * sendbuf, int SIZE, bool flag)
{
    if(flag == 0){
        sen = send(sockfd, sendbuf, SIZE, 0);
    }
    else if(flag == 1){
        sen = send(sockfd, sendbuf, SIZE, MSG_DONTWAIT);
    }
    //mutex_t.unlock();
    //std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(1));
    //printf("sen:%i\n",sen);
    if(sen == -1){
        //std::cout << "\r" << "Sending error!!\n" << std::flush;
        printf("Sending error!\n");
        socket_connect = false;
        end();
    }
    else if(sen == 0){
        //std::cout << "\r" << "Sending disconnected!\n" << std::flush;
        printf("Sending disconnected!\n");
        socket_connect = false;
        end();
    }
}

int Connection::process_receiving()
{

}

void Connection::end()
{
    close(sockfd);
    printf("Socket Closed!\n");
}