#include "connect.h"

Connect::Connect()
{
    printf("Connect called!\n");
}
Connect::~Connect()
{
    end();
}

void Connect::init(std::string _ip, uint16_t _port_num)
{
    ip.assign(_ip);
    port_num = _port_num;
    
    //map = new char**[grid_x];
    /*for(int i = 0; i < grid_x; ++i){
        //map[i] = new char*[grid_y];
        for(int j = 0; j < grid_y; ++j){
            //map[i][j] = new char[grid_z];
            for(int k = 0; k < grid_z; ++k){
                //map[i][j][k] = 0;
                voxel_map[i][j][k] = map[i][j][k];
            }
        }
    }*/
    //map = _map;

    //std::cout << "_map: " << map[20][10][20] << std::endl;
    //camera_pose = _camera_pose;

    // convert ip type (string->char[])
    char char_ip[ip.length()+1];
    std::strcpy(char_ip, ip.c_str()); 
    //std::copy(ip.begin(), ip.end(), char_ip);
    std::cout << "char_ip: " << char_ip << \
    "    port:" << port_num << std::endl;


    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if(sockfd == -1){
        printf("Fail to create a socket!\n");
    }
    int nSendBuf=512*1024;//512K
    setsockopt(sockfd,SOL_SOCKET,SO_SNDBUF,(const char*)&nSendBuf, sizeof(int));
    //int set = 1;
    //setsockopt(sockfd, SOL_SOCKET, MSG_NOSIGNAL, (void *)&set, sizeof(int));
    signal(SIGPIPE, SIG_IGN);
    //************************
    bzero(&serverInfo,sizeof(serverInfo));
    serverInfo.sin_family = PF_INET;
    // local host test
    serverInfo.sin_addr.s_addr = inet_addr(char_ip);	//127.0.0.1		104.39.160.46	104.39.89.30
    serverInfo.sin_port = htons(port_num);

    err = connect(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
    if(err==-1){
        printf("Connection error\n");
        connect_stream = false;
    }
    else{
        printf("Connected!\n");
        connect_stream = true;
    }
}

void Connect::start()
{
    if(connect_stream == false){
        std::cout << "Stop to send data!" << std::endl;
        return;
    }
    std::cout << "Start to send data" << std::endl;
    streamThread = std::thread(&Connect::senddata, this);
}

void Connect::end()
{
    close(sockfd);
    printf("Socket Closed!\n");
     if(streamThread.joinable()){
        streamThread.join();
        std::cout << "Connect streamThread released" << std::endl;
    }
}

void Connect::senddata()
{
    while(mapper_stream){
        if(!connect_stream){
            printf("\033[1A"); //go back to previous row
            printf("\033[K");  //flush
            printf("\033[1A");
            printf("\033[K");
            printf("\033[1A");
            printf("\033[K");
            end();
            printf("Reinitializing...\n");
            //init();
            std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(100));
        }
        else{
            clock_t t01;
            sendmap();
            sendcam();
            sendshift();
            clock_t t04;
            //double duration = t04 - t01;
            //std::cout<< "\r" << "time spend: " << duration << std::flush;
        }
        //std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(10));
    }
}

void Connect::recvdata()
{
    
}

void Connect::sendmap()
{
    unsigned char buf[grid_x][grid_y][grid_z];
    int buf_size = sizeof(buf);
    //std::lock_guard<std::mutex> mlcok(mutex_t_s2g);
    mutex.lock();
    for (int i = 0; i < grid_x; i++) {
        for (int j = 0; j < grid_y; j++) {
            for (int k = 0; k < grid_z; k++) {
                buf[i][j][k] = voxel_map_logodd[i][j][k];
            }
        }
    }
    mutex.unlock();
    if(err != -1){
        process_sending(buf, buf_size, 0);
    }
}

void Connect::sendcam()
{
    float buf[12];
    int buf_size = sizeof(buf);

    //std::lock_guard<std::mutex> mlcok(mutex_t_c2g);
    mutex.lock();
    for (int i = 0; i < 12; i++) {
        buf[i]=camera_global_pose[i];
    }
    mutex.unlock();
    if(err != -1){
        process_sending(buf, buf_size, 1);
    }	
}

void Connect::sendshift()
{
    int buf[3];
		int buf_size = sizeof(buf);
		//std::lock_guard<std::mutex> mlcok(mutex_t_s2g);
		mutex.lock();
		for (int i = 0; i < 3; i++) {
			buf[i]=map_shift[i];
		}
		if(err != -1){
			process_sending(buf, buf_size, 0);
		}
        mutex.unlock();		
		//printf("sendshift sent: %d\n", sen);
		std::cout << map_shift[0] << " " << map_shift[1] << " " << map_shift[2] << " " << std::endl;
}

int Connect::process_sending(void * sendbuf, int SIZE, bool flag)
{
    if(flag == 0){
        sen = send(sockfd, sendbuf, SIZE, 0);
    }
    else if(flag == 1){
        sen = send(sockfd, sendbuf, SIZE, MSG_DONTWAIT);//MSG_DONTWAIT);
    }
    //mutex_t.unlock();
    //std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(1));
    //printf("sen:%i\n",sen);
    if(sen == -1){
        //std::cout << "\r" << "Sending error!!\n" << std::flush;
        printf("Sending error!\n");
        connect_stream = false;
        end();
    }
    else if(sen == 0){
        //std::cout << "\r" << "Sending disconnected!\n" << std::flush;
        printf("Sending disconnected!\n");
        connect_stream = false;
        end();
    }
    return sen;
}

int Connect::process_receiving(void * recvbuf, int SIZE)
{
    ret = recv(Clientsockfd, recvbuf, SIZE, 0);
    if(ret == -1){
        printf("Receiving error!\n");
    }
    
    return ret;
}