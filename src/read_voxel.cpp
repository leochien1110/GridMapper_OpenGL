#ifdef __WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WINSOCK2.H>
#pragma comment(lib,"ws2_32.lib")
#elif __linux__
#include <stdlib.h>
#include <sys/socket.h>	//unix socket
#include <unistd.h>
#include <netdb.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>
#endif
//#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <math.h>  //tan cos
#include <functional>
#include <vector>
#include <opencv2/opencv.hpp>   // Include OpenCV API

//imgui
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

// opengl
 
//#define GLFW_INCLUDE_GLU
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION  
#include "imageutil.h"
#include "stb_image.h"
#include "shader_m.h"

#define READFILE	// comment this line to connect to onboard mapper
#define GUI

// Map file name/directory
#ifdef READFILE
	std::string filename = "../res/A2_10030100_06292020.log";
#endif //READFILE

using namespace std;
using namespace cv;

// Mutex
std::mutex mutex_map;
std::mutex mutex_cam;
std::mutex mutex_shift;

/* Building Dimension */
const int LENGTH = 20;
const int HEIGHT = 6;	//celling = 0
const int WIDTH = 20;
float CAM[3] = { 7.5, 5, 5 };	//camera actual position

/* Voxel Map Dimension */
//Voxel map scale
const float block_unit_m = 0.2;	//(0.5 meter/cell)
const int unit_length_x = 5;	//(20  pixel/cell)
const int unit_length_y = 5;	//(20  pixel/cell)
const int unit_length_z = 5;	//(20  pixel/cell)

//Map scale in real world (pixel per meter), can use to transfer map from real world to voxel world
const float mapscale_x = unit_length_x / block_unit_m;	//75
const float mapscale_y = unit_length_y / block_unit_m;
const float mapscale_z = unit_length_z / block_unit_m;	//40

/* Voxel Map Resolution */
const int v_x = LENGTH * mapscale_x;	//1500
const int v_y = HEIGHT * mapscale_y;	//900
const int v_z = WIDTH * mapscale_z;	//1500

const int grid_x = 100;	//LENGTH / block_unit_m = resolution / unit_length
const int grid_y = 30;	// unit_length_y;	
const int grid_z = 100;	// WIDTH / block_unit_m
int specific_row = 15;
unsigned char voxel_map_logodd[grid_x][grid_y][grid_z] = {127};
bool initial_voxel_map[grid_x][grid_y][grid_z] = {1};	//make sure cell has been visited
int map_shift[3] = { 0 };	//(0,0,0), once the camera is close to boundary, shift the whole cells

// Field of View

float FOVxz = 1.487021;
float FOVyz = 1.01229;
float half_FOVxz = FOVxz / 2;
float half_FOVyz = FOVyz / 2;
float FOV[2];
//float f1_2 = mapscale_x * sqrt(pow(1, 2) / (pow(tan(half_FOVxz), 2) + pow(tan(half_FOVyz), 2) + 1));
//float f1_0 = tan(half_FOVxz)*f1_2;
//float f1_1 = tan(half_FOVyz)*f1_2;
float f1_2 = 1/block_unit_m;
float f1_0 = tan(half_FOVxz)*f1_2;
float f1_1 = tan(half_FOVyz)*f1_2;
float f1[3] = { f1_0, f1_1, f1_2 };
float max_distance = 10;
float init_camera_global_pos[3] = { 10 , 3 , 10 };	//meter
float camera_global_pose[12] = { 0 };		//camera position(x,y,z) + rotation angle(phi,theta,psi)
float camera_scaled_pose[3] = { 0 };
float camera_state[6] = { 0 };

// Rotation angle
float phi = 0;
float theta = 0;
float psi = 0;

// Connect Info widgets
bool show_connect_map = true;
bool show_connect_camera = true;
bool show_connect_map_shift = true;
bool show_connect_planned_path = true;
bool show_connect_trajectory = true;
bool socket_connect = false;

// Socket Data Sending
class DataSend {
public:
#ifdef __WIN32
	SOCKET sockfd;
#elif __linux__
	int sockfd = 0, Clientsockfd = 0;
	struct sockaddr_in serverInfo,clientInfo;
#endif
	unsigned int addrlen = sizeof(clientInfo);
	int ret = 0;
	char message[30] = {"Hi,this is server.\n"};
	// Socket declared
	DataSend() {
		printf("Socket called!\n");
#ifdef __WIN32
#elif __linux__		
#endif
	}
	//check if there some error
	void init(uint16_t port_num) {
#ifdef __WIN32
		int err;
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA data;
		if (WSAStartup(sockVersion, &data) != 0)
		{map_sock
			std::cout << LPSTR("Client Socket Readymap_sock!\n");
		}map_sock
		sockfd = socket(AF_INET, SOCK_STREAM, IPPROmap_sockTO_TCP);
		if (sockfd == INVALID_SOCKET)
		{
			printf("invalid socket !\n");
		}
#elif __linux__

		std::cout << "port:" << port_num << std::endl;

		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd == -1){
			printf("Fail to create a socket!\n");
		}
		int nRecvBuf=512*1024;//512K
		setsockopt(sockfd,SOL_SOCKET,SO_RCVBUF,(const char*)&nRecvBuf,sizeof(int));
		//int nNetTimeout=10;//1sec
		//RECV Timeout
		//setsockopt(sockfd, SOL_S0CKET,SO_RCVTIMEO, (char *)&nNetTimeout, sizeof(int));
		//************************
		bzero(&serverInfo,sizeof(serverInfo));
		serverInfo.sin_family = PF_INET;
		// local host test
		serverInfo.sin_addr.s_addr = INADDR_ANY;	//"127.0.0.1" = INADDR_ANY
		serverInfo.sin_port = htons(port_num);
		bind(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
		listen(sockfd,5);
		
//		printf("set_bind and listen called!\n");
		
#endif
	}
	void set_accept(){
//		printf("wait for connection....\n");
		Clientsockfd = accept(sockfd,(struct sockaddr*)&clientInfo, &addrlen);
		/*if((Clientsockfd = accept(sockfd,(struct sockaddr*)&clientInfo, &addrlen)) == -1){
			perror("accept");
		}*/
//		printf("Connection Accepted!\n");
		socket_connect = true;
	}
	void set_nonblocking(){
		//fcntl(Clientsockfd , F_SETFL, O_NONBLOCK);
		int flag = fcntl(sockfd, F_GETFL, 0);
		if (flag < 0) {
			perror("fcntl1 F_GETFL fail");
			//return 0;
		}
		if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
			perror("fcntl1 F_SETFL fail");
			//return 0;
		}
		printf("Non-blocking Activated!\n");
	}

	void recvdata(){
		while(true){
			if(ret == -1 || ret == 0){
				show_connect_map = false;
				show_connect_camera = false;
				show_connect_map_shift = false;
				set_accept();
			}
			//clock_t t01;
			recvmap();
			//clock_t t02;
			recvcam();
			//clock_t t03;
			recvshift();
			//clock_t t04;
			//double duration = t04 - t01; 
			//std::cout << "\r" << "time spend: " << duration << std::flush;
			//std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(10));
		}
	}
	void recvmap(){
		//while(true){
			unsigned char buf[grid_x][grid_y][grid_z] = {0};
			int buf_size = sizeof(buf);
			process_receiving(&buf, sizeof(buf));
			//printf("map_sock recv: %i\n",ret);
			if(ret == buf_size){
				show_connect_map = true;
				mutex_map.lock();
				for(int i = 0; i < grid_x; i++){
					for(int j = 0; j < grid_y; j++){
						for(int k = 0; k < grid_z; k++){
							voxel_map_logodd[i][j][k] = buf[i][j][k];
						}
					}
				}
				mutex_map.unlock();
			}
			else if (ret == 0){
				printf("map is disconnected\n");
				show_connect_map = false;
				for (int i = 0; i < grid_z; i++) {
					for (int j = 0; j < grid_y; j++) {
						for (int k = 0; k < grid_x; k++) {
							voxel_map_logodd[i][j][k] = 0;		//map robustness
						}
					}
				}
			}
			else if (ret == -1){
				printf("map is not connected!\n");
				show_connect_map = false;
				socket_connect = false;
			}
			else {
				printf("Mapper Receive error!\n");
				printf("map_sock.ret:%i\n",ret);
//				continue;
			}
			//this_thread::sleep_for( chrono::duration<int, std::milli>( 100 ) );
			
		//}
	}

	void recvcam(){
		//while(true){
			float buf[12] = {0};
			int buf_size = sizeof(buf);
			process_receiving(&buf, sizeof(buf));
			//printf("cam_sock recv: %i\n",ret);
			if(ret == buf_size){
				//printf("cam_sock recv successful!:%i\n",ret);
				show_connect_camera = true;
				std::lock_guard<std::mutex> mlock(mutex_cam);
				for(int i = 0; i < 12; i++){
					camera_global_pose[i] = buf[i];
				}
				//if(camera_global_pose[0]==0)
				//	std::cout << camera_global_pose[0] << " " << camera_global_pose[1] << " " << camera_global_pose[2] << std::endl;
			}
			else if (ret == 0){
				printf("Camera is disconnected\n");
				show_connect_camera = false;
				for (int i = 0; i < 12; i++) {
					camera_global_pose[i] = 0;
				}
			}
			else if (ret == -1){
				//printf("Camera is not connected!\n");
				show_connect_camera = false;
				socket_connect = false;
			}
			else {
				printf("Camera Receive error!\n");
				printf("cam_sock.ret:%i\n",ret);
//				continue;
			}
			//std::cout << camera_global_pose[0] << " " << camera_global_pose[1] << " " << camera_global_pose[2] << " " << std::endl;
			//this_thread::sleep_for( chrono::duration<int, std::milli>( 200 ) );

		//}
	}

	void recvshift(){
		//while(true){
			int buf[3] = {0};
			int buf_size = sizeof(buf);
			process_receiving(&buf, sizeof(buf));
			//printf("shift_sock recv: %i\n",ret);
			if(ret == buf_size){
				//printf("shift_sock recv successful!:%i\n",ret);
				show_connect_map_shift = true;
				mutex_shift.lock();
				for(int i = 0; i < 3; i++){
					map_shift[i] = buf[i];
				}
				mutex_shift.unlock();
			}
			else if (ret == 0){
				printf("Shift is disconnected\n");
				show_connect_map_shift = false;
				for (int i = 0; i < 3; i++) {
					map_shift[i] = 0;
				}
			}
			else if (ret == -1){
				//printf("Shift is not connected!\n");
				show_connect_map_shift = false;
				socket_connect = false;
			}
			else {
				printf("Shift Receive error!\n");
				printf("shiftsock.ret:%i\n",ret);
//				continue;
			}
			std::cout << map_shift[0] << " " << map_shift[1] << " " << map_shift[2] << " " << std::endl;
			//this_thread::sleep_for( chrono::duration<int, std::milli>( 200 ) );
		//}
	}

	void process_sending() {
#ifdef __WIN32
#elif __linux__
		//std::cout << "char size:" << sizeof(data) << std::endl;
		send(Clientsockfd, message, sizeof(message), 0);
		// force cast
		//float(&pArray)[n_update][4] = *reinterpret_cast<float(*)[n_update][4]>(data);
#endif
	}
	// data receive from server
	void process_receiving(void * recvbuf, int SIZE) {
		//printf("processing received data...\n");
		ret = recv(Clientsockfd, recvbuf, SIZE, MSG_WAITALL);
		//printf("SIZE:%i\n",SIZE);
		//if(camera_global_pose[0]==0)
		//			std::cout << camera_global_pose[0] << " " << camera_global_pose[1] << " " << camera_global_pose[2] \
		//					<< " " << ret << std::endl;
		//printf("ret:%i\n",ret);
		//while(1){
		//}
		
	}
	void End() {
#ifdef __WIN32
		closesocket(sockfd);
#elif __linux__
		close(sockfd);
#endif		
		printf("Socket Closed!\n");
	}
};

struct float3 {
	float x, y, z;
	float3 operator*(float t);

	float3 operator-(float t);

	void operator*=(float t);

	void operator=(const float3 &other);

	void add(float t1, float t2, float t3);
};

struct float4 {
	float w, x, y, z;
	float4 operator*(float t)
	{
		return { w * t, x * t, y * t, z * t };
	}

	float4 operator-(float t)
	{
		return { w - t, x - t, y - t, z - t };
	}

	void operator*=(float t)
	{
		w = w * t;
		x = x * t;
		y = y * t;
		z = z * t;
	}

	void operator=(float4 other)
	{
		w = other.w;
		x = other.x;
		y = other.y;
		z = other.z;
	}

	void add(float t1, float t2, float t3, float t4)
	{
		w += t1;
		x += t2;
		y += t3;
		z += t4;
	}
};

float4 q;

float4 Quat2Euler(float4 q) {
	float4 angle;
	//float Cq[3][3];
	//Cq[0][0] = pow(q.w, 2) + pow(q.x, 2) - pow(q.y, 2) - pow(q.z, 2);	Cq[0][1] = 2 * (q.x* q.y + q.w * q.z);								Cq[0][2] = 2 * (q.x* q.z - q.w * q.y);
	//Cq[1][0] = 2 * (q.w * q.z - q.x* q.y);								Cq[1][1] = -pow(q.w, 2) + pow(q.x, 2) - pow(q.y, 2) + pow(q.z, 2);	Cq[1][2] = 2 * (q.y* q.z + q.w * q.x);
	//Cq[2][0] = 2 * (q.x* q.z + q.w * q.y);								Cq[2][1] = 2 * (q.y* q.z - q.w * q.x);								Cq[2][2] = 1 - 2 * (pow(q.y, 2) + pow(q.z, 2))/*pow(q.w, 2) - pow(q.x, 2) - pow(q.y, 2) + pow(q.z, 2)*/;
	//2-3-1
	//angle.x = atan2(Cq[1][0], Cq[1][1]);	//camera_global_pose[3] attitude pitch
	//angle.y = asin(Cq[1][2]);				//camera_global_pose[4] heading yaw
	//angle.y = atan2(Cq[0][2], Cq[2][2]);	//camera_global_pose[5] bank roll
	//2-3-1
	//angle.x = asin(Cq[0][1]);	//camera_global_pose[3] attitude pitch
	//angle.y = atan2(-Cq[0][2], Cq[2][2]);	//camera_global_pose[4] heading yaw
	//angle.z = atan2(-Cq[1][0], 1 - 2 * (pow(q.x, 2) - pow(q.z, 2)));	//camera_global_pose[5] bank roll
	//3-2-1
	double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	angle.x = atan2(sinr_cosp, cosr_cosp);	//yaw
	double sinp = 2.0 * (q.w * q.y - q.z * q.x);
	angle.y = asin(sinp);	//pitch
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	angle.z = atan2(siny_cosp, cosy_cosp);	//roll

	//--------Reference----------
	//---------------------------
	//3-2-1
	//float camera_global_pose[3] = atan2(2 * (q.w*q.z + q.y * q.x), q.z ^ 2 - q.w ^ 2 - q.x ^ 2 + q.y ^ 2);
	//float camera_global_pose[4] = asin(2 * q.x*q.z - q.w * q.y);
	//float camera_global_pose[5] = atan2(2.*(q.w*q.x + q.y*q.z), (q.z^2+q.w^2-q.x^2-q.y^2));
	//1-2-3
	//float camera_global_pose[3] = atan2(2 * (q.y*q.z - q.w * q.x), q.z ^ 2 + q.w ^ 2 - q.x ^ 2 - q.y ^ 2);	//atan2(Cq[2][1],Cq[2][2])
	//float camera_global_pose[4] = asin(2 * (q.x*q.z + q.w * q.y));										//asin(Cq[2][0])
	//float camera_global_pose[5] = atan2(2.*(q.w*q.z - q.x * q.y), (q.z ^ 2 - q.w ^ 2 - q.x ^ 2 + q.y ^ 2));	//atan2(-Cq[1][0],-Cq[0][0])
	//2-3-1
	//float camera_global_pose[3] = atan2(2 * (q.w * q.z - q.y * q.x), q.z ^ 2 - q.w ^ 2 + q.x ^ 2 - q.y ^ 2);	//atan2(-Cq[1][0],-Cq[1][1])
	//float camera_global_pose[4] = asin(2 * (q.w * q.x + q.y * q.z));									//asin(Cq[1][2])
	//float camera_global_pose[5] = atan2(2*(q.x * q.z - q.w * q.y), (q.z^2 + q.w ^ 2 - q.x ^ 2 - q.y ^ 2));	//atan2(Cq[0][2],Cq[2][2])

	return angle;
}

//---------------------
// OpenGL Visualization
//---------------------
void glInit(GLFWwindow * window);
void glSetup(unsigned int VAO, unsigned int VBO, float Vertices[], unsigned int AttribSize1,
	unsigned int AttribSize2, unsigned int Stride, unsigned int VertexOffset);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
unsigned int loadTexture(char const * path);
unsigned int matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);

//Input Setup
void processInput(GLFWwindow *window);
void mouse_button_callback(GLFWwindow * window, int button, int action, int mode);
void mouse_position_callback(GLFWwindow * window, double xpos, double ypos);
void scroll_callback(GLFWwindow * window, double xoffset, double yoffset);
void mouseLeftPressEvent(GLFWwindow * window, double xpos, double ypos);
void mouseLeftReleaseEvent();
void mouseLeftDragEvent(GLFWwindow * window, double xpos, double ypos);

// Window settings
const unsigned int SCR_WIDTH = 1600;
const unsigned int SCR_HEIGHT = 900;

// Camera movement
glm::vec3 cameraPos = glm::vec3(0.0f, -5.0f, 10.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, -0.5f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

// Source light
glm::vec3 lightPos(0.0f, 5.0f, 10.0f);

// Mouse setting
bool firstMouse = true;
float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch = -30.0f;
float lastX = SCR_WIDTH / 2.0;
float lastY = SCR_HEIGHT / 2.0;
float fov = 45.0f;
bool rotation_started = false;
bool viewpoint = true;
int start_x = 0;
int start_y = 0;

// load textures (we now use a utility function to keep the code more organized)
// -----------------------------------------------------------------------------
unsigned int texture_index = 0;


// set up vertex data
float vertices[] = {
	// positions          // normals           // texture coords
	-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,
	 0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 0.0f,
	 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
	-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,

	-0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   0.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   1.0f, 1.0f,
	-0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   0.0f, 0.0f,

	-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
	-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
	-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
	-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 0.0f,

	0.5f,   0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
	0.5f,   0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
	0.5f,  -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	0.5f,  -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
	0.5f,  -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
	0.5f,   0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,

	-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 1.0f,
	 0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 1.0f,
	 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 0.0f,
	 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 0.0f,
	-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 0.0f,
	-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 1.0f,

	-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 1.0f,
	 0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 1.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 0.0f,
	 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 0.0f,
	-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 0.0f,
	-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 1.0f
};

float axisVertices[] = {
	//x-axis
	0.0f, 0.0f,  0.0f, 1.0f, 0.0f, 0.0f,
	2.0f, 0.0f,  0.0f, 1.0f, 0.0f, 0.0f,
	//y-axis
	0.0f,  0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
	0.0f, -2.0f, 0.0f, 0.0f, 1.0f, 0.0f,
	//z-axis
	0.0f, 0.0f,  0.0f, 0.0f, 0.0f, 1.0f,
	0.0f, 0.0f, -2.0f, 0.0f, 0.0f, 1.0f,
};

float fovVertices[] = {
	//top-right
	  0.0f,    0.0f,  0.0f,  0.5f, 0.1f, 0.3f,
	 f1[0],   f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	//top-left
	  0.0f,    0.0f,   0.0f, 0.5f, 0.1f, 0.3f,
	-f1[0],   f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	//down-left
	  0.0f,    0.0f,   0.0f, 0.5f, 0.1f, 0.3f,
	-f1[0],  -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	//down-right
	  0.0f,    0.0f,   0.0f, 0.5f, 0.1f, 0.3f,
	 f1[0],  -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	//top
	  f1[0],  f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	 -f1[0],  f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	//left
	 -f1[0],  f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	 -f1[0], -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	//down
	 -f1[0], -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	  f1[0], -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	//right
	  f1[0], -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
	  f1[0],  f1[1], -f1[2], 0.5f, 0.1f, 0.3f
};

int main() {
	//clock_t t1 = clock();
	struct timeval start, end;
	long mtime, seconds, useconds;	
	int here = 0;
	
	// Trajectory
	std::vector<float3> trajectory;
	std::vector<float3> scale_trajectory;

	// get ip address
	struct ifaddrs * ifAddrStruct=NULL;
    struct ifaddrs * ifa=NULL;
    void * tmpAddrPtr=NULL;
	//char addressBuffer[INET_ADDRSTRLEN];
	char IPaddressBuffer1[INET_ADDRSTRLEN];
	char IPaddressBuffer2[INET_ADDRSTRLEN];
	char * ifaname1;
	char * ifaname2;
	int ipcount = 0;
    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) {
			continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET) { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            printf("%s IP4 Address %s\n", ifa->ifa_name, addressBuffer);
			if(ipcount == 0){
				memcpy(IPaddressBuffer1, addressBuffer, sizeof(IPaddressBuffer1));
				ifaname1 = ifa->ifa_name;
				ipcount++;
			}
			else if(ipcount == 1){
				memcpy(IPaddressBuffer2, addressBuffer, sizeof(IPaddressBuffer2));
				ifaname2 = ifa->ifa_name;
			}
        } 
		/*else if (ifa->ifa_addr->sa_family == AF_INET6) { // check it is IP6
            // is a valid IP6 Address
            tmpAddrPtr=&((struct sockaddr_in6 *)ifa->ifa_addr)->sin6_addr;
            char addressBuffer[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, tmpAddrPtr, addressBuffer, INET6_ADDRSTRLEN);
            printf("%s IP6 Address %s\n", ifa->ifa_name, addressBuffer); 
        } */
    }
	if (ifAddrStruct!=NULL) freeifaddrs(ifAddrStruct);

#ifdef READFILE
	
	std::ifstream infile(filename);
	std::string temp = "";
	if (!infile) {
		return 1;
	}
	std::cout << "Reading... " << std::endl;
	// get length of file:
	infile.seekg(0, infile.end);
	int length = infile.tellg();
	infile.seekg(0, infile.beg);
	std::cout << "Reading... " << filename << std::endl;

	char * buffer = new char[length];
	infile.read(buffer, length);
	std::string str2 = buffer;
	std::stringstream ss(str2);
	//ss.ignore(str2.length(), 'V');

	std::cout<<"read_voxel start!" << std::endl;
	
	for (int k = 0; k < grid_z; k++) {
		for (int j = 0; j < grid_y; j++) {
			for (int i = 0; i < grid_x; i++) {
				getline(ss, temp, ',');
				voxel_map_logodd[i][j][k] = atof(temp.c_str());
				//std::cout << voxel_map_logodd[i][j][k] << std::endl;
				if(voxel_map_logodd[i][j][k] != 127)
					initial_voxel_map[i][j][k] = 1;
			}
		}
	}
	//3.488579,0.075603,-0.578689,0.916609,0.003368,0.015034,0.399488
	q.w = 0.999860; q.x = -0.002322; q.y = -0.016577; q.z = -0.000075;
	float4 angle = Quat2Euler(q);

	camera_state[0] = 0;
	camera_state[1] = 0;
	camera_state[2] = 0;
	map_shift[0] = 0;
	map_shift[1] = 0;
	map_shift[2] = 0;
	camera_global_pose[0] = init_camera_global_pos[0] + camera_state[0] + map_shift[0];
	camera_global_pose[1] = init_camera_global_pos[1] + camera_state[1] + map_shift[1];
	camera_global_pose[2] = init_camera_global_pos[2] + camera_state[2] + map_shift[2];
	camera_global_pose[3] = angle.y;
	camera_global_pose[4] = angle.z;
	camera_global_pose[5] = angle.x;

#endif

#ifndef READFILE
	//socket construct
    char inputBuffer[400000] = {};
    
    char message[] = {"Hi,this is server.\n"};
    
	DataSend map_sock, MPC_sock, cam_sock, shift_sock, data_sock;
	// A*
/*	MPC_sock.init(7777);
	printf("Waiting for A*...\n");
	MPC_sock.set_accept();
	MPC_sock.set_nonblocking();
*/	

	// voxel map
	/*
	map_sock.init(6666);
	printf("Waiting for Mapper...\n");
	map_sock.set_accept();
	map_sock.set_nonblocking();

	cam_sock.init(6667);
	printf("Waiting for camera states...\n");
	cam_sock.set_accept();
	cam_sock.set_nonblocking();

	shift_sock.init(6668);
	printf("Waiting for shift_map...\n");
	shift_sock.set_accept();
	shift_sock.set_nonblocking();
	*/

	data_sock.init(7777);
	printf("Waiting for data...\n");
	//data_sock.set_accept();
	//data_sock.set_nonblocking();
	//printf("Here1\n");
#endif

	// OpenCV
	//-------
	//const auto window_name2 = "GS Voxel Map_xz";
	//namedWindow(window_name2, WINDOW_AUTOSIZE);
	
    //if ((client_fd = accept(server_fd, (struct sockaddr *)&clientAddr, &addrlen)) == -1) {
    //   		perror("accept");
    //}
	//fcntl(last_fd , F_SETFL, O_NONBLOCK); /* Change the socket into non-blocking state	*/
	//fcntl(client_fd, F_SETFL, O_NONBLOCK); /* Change the socket into non-blocking state	*/

	//------------//
	//OpenGL Setup//
	//------------//

	// glfw: initialize and configure(version 3.3)
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// OpenGL Window
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "GS Voxel 3D", NULL, NULL);
	// Create a simple OpenGL window for rendering:
	//GLFWwindow* window2 = glfwCreateWindow(width, height, "RealSense Measure Example", NULL, NULL);

	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glInit(window);
	
	
	// Dear Imgui
#ifdef GUI
	ImVec4 clear_color = ImVec4(0.1f, 0.1f, 0.1f, 1.00f);
	const char* glsl_version = "#version 130";
	// Setup Dear ImGui context
	ImGui::CreateContext();
    
    ImGui::StyleColorsDark();

	 // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

	bool show_2d_map = true;
    bool show_demo_window = true;
	bool show_another_window = false;
	//ImVec4 clear_color = ImVec4(0.3f, 0.5f, 0.60f, 1.00f);
	
	ImVec2 window_size_2d = ImVec2(500,500); //2d map size


#endif
	// build and compile shader program
	Shader lightingShader("../res/texture02.vs", "../res/texture02.fs");
	Shader planeshader("../res/grid.vs", "../res/grid.fs");


	// Vertex attributes
	// Voxel Cells
	// first, configure the cube's VAO (and VBO)
	unsigned int VBO, cubeVAO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glBindVertexArray(cubeVAO);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	// texture attribute
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);

	// second, configure the light's VAO (VBO stays the same; the vertices are the same for the light object which is also a 3D cube)
	unsigned int lightVAO;
	glGenVertexArrays(1, &lightVAO);
	glBindVertexArray(lightVAO);
	// we only need to bind to the VBO (to link it with glVertexAttribPointer), no need to fill it; the VBO's data already contains all we need (it's already bound, but we do it again for educational purposes)
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	//Texture
	unsigned int wallTexture = loadTexture("../res/wall_b.jpg");			//index = 0
	unsigned int creeperTexture  = loadTexture("../res/creeper.jpg");	//index = 1

	//Plane
	const int gridlines = 202;
	float planeVertices[(gridlines + 2) * 2 * 6];
	int gridlinecount = 0;
	for (int k = 0; k < gridlines; k += 2) {
		//-----------------
		planeVertices[6 * k + 0] = 0.0f;
		planeVertices[6 * k + 1] = 0.0f;
		planeVertices[6 * k + 2] = -0.5*k;
		planeVertices[6 * k + 3] = 1.0f;
		planeVertices[6 * k + 4] = 1.0f;
		planeVertices[6 * k + 5] = 1.0f;

		planeVertices[6 * (k + 1) + 0] = 100.0;
		planeVertices[6 * (k + 1) + 1] = 0.0f;
		planeVertices[6 * (k + 1) + 2] = -0.5*k;
		planeVertices[6 * (k + 1) + 3] = 1.0f;
		planeVertices[6 * (k + 1) + 4] = 1.0f;
		planeVertices[6 * (k + 1) + 5] = 1.0f;	//last 131
		//std::cout << planeVertices[6 * k + 0] << "," << planeVertices[6 * k + 2] << ";" << planeVertices[6 * (k + 1) + 0] << "," << planeVertices[6 * (k + 1) + 2] << std::endl;
		gridlinecount++;
		//||||||||||||||||||||
		planeVertices[6 * k + 0 + 6 * (gridlines)] = 0.5*k;	//1st 132
		planeVertices[6 * k + 1 + 6 * (gridlines)] = 0.0f;
		planeVertices[6 * k + 2 + 6 * (gridlines)] = 0.0f;
		planeVertices[6 * k + 3 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * k + 4 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * k + 5 + 6 * (gridlines)] = 1.0f;

		planeVertices[6 * (k + 1) + 0 + 6 * (gridlines)] = 0.5*k;
		planeVertices[6 * (k + 1) + 1 + 6 * (gridlines)] = 0.0f;
		planeVertices[6 * (k + 1) + 2 + 6 * (gridlines)] = -100.0;
		planeVertices[6 * (k + 1) + 3 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * (k + 1) + 4 + 6 * (gridlines)] = 1.0f;
		planeVertices[6 * (k + 1) + 5 + 6 * (gridlines)] = 1.0f;	//263
		//std::cout << planeVertices[6 * k + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * k + 2 + 6 * (gridlines + 2)] << ";" << planeVertices[6 * (k + 1) + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * (k + 1) + 2 + 6 * (gridlines + 2)] << std::endl;
		gridlinecount++;
	}
	unsigned int planeVAO, planeVBO;
	glGenVertexArrays(1, &planeVAO);
	glGenBuffers(1, &planeVBO);
	glBindVertexArray(planeVAO);
	glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);	//(x,y,z,R,G,B),each float occupy 4 byte
	glEnableVertexAttribArray(0);
	// color attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));	//last variable is the offset of the color data
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);
	
	// Axis	
	unsigned int axisVAO, axisVBO;
	glGenVertexArrays(1, &axisVAO);
	glGenBuffers(1, &axisVBO);
	glBindVertexArray(axisVAO);
	glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(axisVertices), axisVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glBindVertexArray(0);

	// Field of View
	unsigned int fovVAO, fovVBO;
	glGenVertexArrays(1, &fovVAO);
	glGenBuffers(1, &fovVBO);
	glBindVertexArray(fovVAO);
	glBindBuffer(GL_ARRAY_BUFFER, fovVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(fovVertices), fovVertices, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glBindVertexArray(0);
	
	int packagelost = 0;
	const int trajLength = 52;
	float trajArr[trajLength * 6] = { 0 };
	unsigned char voxelBuf[grid_x][grid_y][grid_z] = { 0 };	//voxel_logodd for socket buffer
	int map_shift_Buf[3] = { 0 };							//map_shift for socket buffer
	float camera_pose_Buf[6] = { 0 };
	float waypt[trajLength][3] = { 0 };

	
	//std::thread t_map(&DataSend::process_receiving, map_sock, &voxel_map_logodd, sizeof(voxel_map_logodd));
	//std::thread t_cam(&DataSend::process_receiving, cam_sock, &camera_global_pose, sizeof(camera_global_pose));
	//std::thread t_shift(&DataSend::process_receiving, shift_sock, &map_shift, sizeof(map_shift));

	//std::thread t_map(&DataSend::recvmap, map_sock);
	//std::thread t_cam(&DataSend::recvcam, cam_sock);
	//std::thread t_shift(&DataSend::recvshift, shift_sock);
#ifndef READFILE
	std::thread t_shift(&DataSend::recvdata, data_sock);
#endif //READFILE

	while (!glfwWindowShouldClose(window)) {
		//std::cout << "socket_connect" << socket_connect << std::endl;
		//socket_connect
		//t1 = clock();
		gettimeofday(&start, NULL);
		// input
		// -----
		processInput(window);
		//int update_data[25000][4] = { 0 };	//(x,y,z,log-odd)

		//Receive//
		//-------
#ifndef READFILE
		//receive voxel map		
//		printf("sizeof(voxelBuf):%i\n",sizeof(voxelBuf));
//		printf("Here!\n");
		
		//map_sock.process_receiving(&voxelBuf,sizeof(voxelBuf));
		/*
		printf("map_sock recv: %i\n",map_sock.ret);
		if(map_sock.ret == -1 || map_sock.ret == 0){
			printf("waiting for Mapper...\n");
			map_sock.set_accept();
			map_sock.set_nonblocking();
		}
		if(map_sock.ret == sizeof(voxelBuf)){
			printf("map_sock recv successful!:%i\n",map_sock.ret);
			show_connect_map = true;
			for(int k = 0; k < grid_z; k++){
				for(int j = 0; j < grid_y; j++){
					for(int i = 0; i < grid_x; i++){
//						std::cout << "sizeof(voxelBuf):" << sizeof(voxelBuf) << std::endl;
						voxel_map_logodd[i][j][k] = voxelBuf[i][j][k];
						//printf("Here!\n");
//						printf("voxel[%i][%i][%i] = %i\n",i,j,k,voxel_map_logodd[i][j][k]);
//						std::cout << "voxel[" << i << "][" << j << "][" << k << "] = " << voxel_map_logodd[i][j][k] << std::endl;
					}
				}
			}
		}
		else if (map_sock.ret == 0){
			printf("map is disconnected\n");
			show_connect_map = false;
			for (int i = 0; i < grid_z; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_x; k++) {
						voxel_map_logodd[i][j][k] = 0;
					}
				}
			}
		}
		else if (map_sock.ret == -1){
			printf("map is not connected!\n");
			show_connect_map = false;
		}
		else {
			printf("Mapper Receive error!\n");
			printf("map_sock.ret:%i\n",map_sock.ret);
//			continue;
		}
//		if(map_sock.ret != sizeof(voxelBuf))
//			continue;
		

		//receive camera states
		//cam_sock.process_receiving(&camera_pose_Buf,sizeof(camera_pose_Buf));
		printf("cam_sock recv: %i\n",cam_sock.ret);
		if(cam_sock.ret == -1 || cam_sock.ret == 0){
			printf("waiting for Camera States...\n");
			cam_sock.set_accept();
			cam_sock.set_nonblocking();
		}
		if(cam_sock.ret == sizeof(camera_pose_Buf)){
//			printf("cam_sock recv successful!:%i\n",cam_sock.ret);
			show_connect_camera = true;
//			std::cout << "camera_pose_Buf:";
			for(int i = 0; i < 6; i++){
				camera_global_pose[i] = camera_pose_Buf[i];
//				std::cout << camera_pose_Buf[i];
			}
//			std::cout << " " << std::endl;
//			std::cout << "\r" << "Device Pose(X,Y, Z, camera_global_pose[0], camera_global_pose[1], camera_global_pose[2]): " << std::setprecision(5) << std::fixed <<
//				camera_global_pose[0] << " " << camera_global_pose[1] << " " << camera_global_pose[2] << " " <<
//				(camera_global_pose[3] * 180 / 3.14159) << " " << (camera_global_pose[4] * 180/3.14158) << " " << (camera_global_pose[5] * 180 / 3.14158)<< " ";	//angle
		}
		else if (cam_sock.ret == 0){
			printf("cam_sock is disconnected\n");
			show_connect_camera = false;
			for(int i = 0; i < 6; i++){
				camera_global_pose[i] = 0;
			}
		}
		else if (cam_sock.ret == -1){
			printf("cam_sock is not connected!\n");
			show_connect_camera = false;
		}
		else {
			printf("cam_sock Receive error!\n");
			printf("cam_sock.ret:%i\n",cam_sock.ret);
			show_connect_camera = false;
		}

		//receive map shift
		//shift_sock.process_receiving(&map_shift_Buf,sizeof(map_shift_Buf));
		printf("shift_sock recv: %i\n",shift_sock.ret);
		if(shift_sock.ret == -1 || shift_sock.ret == 0){
			printf("waiting for shift_map...\n");
			shift_sock.set_accept();
			shift_sock.set_nonblocking();
		}
		
		if(shift_sock.ret == sizeof(map_shift_Buf)){
//			printf("shift_sock recv successful!:%i\n",shift_sock.ret);
			show_connect_map_shift = true;
			for(int i = 0; i < 3; i++){
				map_shift[i] = map_shift_Buf[i];
			}
		}
		else if (shift_sock.ret == 0){
			printf("shift_sock is disconnected\n");
			show_connect_map_shift = false;
			for(int i = 0; i < 3; i++){
				map_shift[i] = 0;
			}
		}
		else if (shift_sock.ret == -1){
			printf("shift_sock is not connected!\n");
			show_connect_map_shift = false; 
		}
		else {
			printf("shift_sock Receive error!\n");
			printf("shift_sock.ret:%i\n",shift_sock.ret);
			show_connect_map_shift = false;
		}
		*/
		//std::cout << "Shifted Postion(X,Y,Z): " << std::setprecision(5) << std::fixed <<
		//camera_global_pose[0] << " " << camera_global_pose[1] << " " << camera_global_pose[2] << " " << map_shift[0] << " " << map_shift[1] << " " << map_shift[2] << std::endl;
		
#endif
		
/*		
		//recevice trajectory
		printf("sizeof(waypt):%i\n",sizeof(waypt));
		MPC_sock.process_receiving(&waypt, sizeof(waypt));
		//MPCret = recv(MPC_sock.Clientsockfd,&waypt,sizeof(waypt)*8,0);
		//printf("MPC_sock.ret:%i",MPC_sock.ret);
		if(MPC_sock.ret == -1 || MPC_sock.ret == 0)
            MPC_sock.set_accept();

		if(MPC_sock.ret == sizeof(waypt)){
			for(int i = 0; i < trajLength; i++){
				trajArr[6*i  ] =  waypt[i][0] / block_unit_m;
				trajArr[6*i+1] = -waypt[i][1] / block_unit_m;
				trajArr[6*i+2] = -waypt[i][2] / block_unit_m;
				trajArr[6*i+3] = 1.0f;
				trajArr[6*i+4] = 1.0f;
				trajArr[6*i+5] = 0.0f;
				std::cout << waypt[i][0] << "," << waypt[i][1] << "," <<  waypt[i][2] << std::endl;
				//std::cout << trajArr[6*i  ] << "," << trajArr[6*i+1] << "," <<  trajArr[6*i+2] 
				//<< std::endl;
			}
			//printf("MPC_sock.ret:%i\n",MPC_sock.ret);
			show_connect_trajectory = true;
		}
		else if (MPC_sock.ret == 0){
			printf("A* is disconnected\n");
			show_connect_trajectory = false;
		}
		else if (MPC_sock.ret == -1){
			printf("A* is not connected!\n");
			show_connect_trajectory = false;
		}
		else {
			printf("A* Receive error!\n");
			printf("A*.ret:%i\n",MPC_sock.ret);
			show_connect_trajectory = false;
			continue;
		}
*/		
		
		// OpenCV 2D Map
		//--------------
		Mat grid2dmap(v_x, v_z, CV_8UC3, Scalar(255, 255, 255));	//xz
		// Draw grid
		for (int i = 1; i < grid_z; i++) {
			line(grid2dmap, Point(i * unit_length_x, 0), Point(i * unit_length_x, v_x), Scalar(0, 0, 0), 1);
		}
		for (int i = 1; i < grid_x; i++) {
			line(grid2dmap, Point(0, i * unit_length_z), Point(v_z, i * unit_length_z), Scalar(0, 0, 0), 1);
		}

		for (int k = 0; k < grid_z; k++) {
			for (int i = 0; i < grid_x; i++) {
				//std::cout << left;
				//std::cout << setprecision(3) << std::setw(5) << voxel_map_logodd[i][specific_row][k];		//show log_odd map in cmd window
				if (voxel_map_logodd[i][specific_row][k] != 127) {
					rectangle(grid2dmap, Point(i * unit_length_x, k * unit_length_z), Point((i + 1) * unit_length_x, (k + 1) * unit_length_z),
						Scalar(voxel_map_logodd[i][specific_row][k], 255 - voxel_map_logodd[i][specific_row][k], 255 - voxel_map_logodd[i][specific_row][k]), -1);	//BGR
					//new statement
					//initial_voxel_map[i][specific_row][k] = 0;					
				}
				//std::cout << omp_get_num_threads() << std::endl;
			}
			//std::cout << std::endl;
		}


		/* Roatation */
		// Rotation matrix & Inverse rotation matrix
		float determinant = 0;
		float C[3][3] = { {cos(camera_global_pose[4])*cos(camera_global_pose[5]), cos(camera_global_pose[4])*sin(camera_global_pose[5]), -sin(camera_global_pose[4])},
		{(-cos(camera_global_pose[3])*sin(camera_global_pose[5]) + sin(camera_global_pose[3])*sin(camera_global_pose[4])*cos(camera_global_pose[5])), cos(camera_global_pose[3])*cos(camera_global_pose[5]) + sin(camera_global_pose[3])*sin(camera_global_pose[4])*sin(camera_global_pose[5]), sin(camera_global_pose[3])*cos(camera_global_pose[4])},
		{sin(camera_global_pose[3])*sin(camera_global_pose[5]) + cos(camera_global_pose[3])*sin(camera_global_pose[4])*cos(camera_global_pose[5]), (-sin(camera_global_pose[3])*cos(camera_global_pose[5]) + cos(camera_global_pose[3])*sin(camera_global_pose[4])*sin(camera_global_pose[5])), cos(camera_global_pose[3])*cos(camera_global_pose[4])} };

		// Inverse Rotation matrix
		float inv_C[3][3];
		for (int i = 0; i < 3; i++)
			determinant = determinant + (C[0][i] * (C[1][(i + 1) % 3] * C[2][(i + 2) % 3] - C[1][(i + 2) % 3] * C[2][(i + 1) % 3]));
		for (int mi = 0; mi < 3; mi++) {
			for (int mj = 0; mj < 3; mj++)
				inv_C[mi][mj] = ((C[(mj + 1) % 3][(mi + 1) % 3] * C[(mj + 2) % 3][(mi + 2) % 3]) - (C[(mj + 1) % 3][(mi + 2) % 3] * C[(mj + 2) % 3][(mi + 1) % 3])) / determinant;
		}


		// FOV conversion
		float f1L[3], f1R[3], f2L[3], f2R[3];
		float f1Rc[3] = { 0,0,0 };
		float f1Lc[3] = { 0,0,0 };
		float f2Rc[3] = { 0,0,0 };
		float f2Lc[3] = { 0,0,0 };

		//camera unit conversion
		camera_scaled_pose[0] = camera_global_pose[0] * mapscale_x;
		camera_scaled_pose[1] = camera_global_pose[1] * mapscale_y;
		camera_scaled_pose[2] = camera_global_pose[2] * mapscale_z;


		//f1[2] = (unit_length_z / block_unit_m) * sqrt(pow(max_distance, 2) / (pow(tan(half_FOVxz), 2) + pow(tan(half_FOVyz), 2) +1 ));
		//f1[0] = tan(half_FOVxz)*f1[2];
		//f1[1] = tan(half_FOVyz)*f1[2];
		//fov position (voxel frame)
		//Right(+-+)
		f1R[0] = f1[0];
		f1R[1] = -f1[1];
		f1R[2] = f1[2];
		//std::cout << "(x,z)=" << f1R[0] << "," << f1R[2] << std::endl;
		//Left(--+)
		f1L[0] = -f1[0];
		f1L[1] = -f1[1];
		f1L[2] = f1[2];

		//f1' = Cc/c' * f1
		for (int l = 0; l < 3; l++) {
			for (int m = 0; m < 3; m++) {
				f1Rc[l] += inv_C[l][m] * f1R[m];
				f1Lc[l] += inv_C[l][m] * f1L[m];
			}
		}

		for (int i = 0; i < 3; i++) {
			f1Rc[i] += camera_scaled_pose[i];
			f1Lc[i] += camera_scaled_pose[i];
		}

		/* Draw the FOV(x-z plane) */

		Point FOV_R(f1Rc[0], f1Rc[2]);	//12m is the max range
		Point FOV_L(f1Lc[0], f1Lc[2]);	//12m is the max range

		line(grid2dmap, Point(camera_scaled_pose[0], camera_scaled_pose[2]), FOV_R, Scalar(0, 255, 0), 2);
		line(grid2dmap, Point(camera_scaled_pose[0], camera_scaled_pose[2]), FOV_L, Scalar(0, 0, 255), 2);
		circle(grid2dmap, Point(camera_scaled_pose[0], camera_scaled_pose[2]), 3, Scalar(0, 0, 0), 2);
		// Create OpenCV matrix of size (w,h) from the colorized depth data

		//flip img
		flip(grid2dmap, grid2dmap, 0);
		//flip(grid2dmap, grid2dmap, 1);

		//cvInputProcess();
		int key = waitKeyEx(1);
		//std::cout << "Y=" << key << std::endl;
		if (key != -1) {
			std::cout << "Key: " << key << std::endl;
			switch (key)
			{
			case 1113938: specific_row--; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//up
			case 2490368: specific_row--; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//up_win
			case 1113940: specific_row++; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//down
			case 2621440: specific_row++; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//down_win
			case 1179731:
				FILE *fp;
				fp = fopen("../res/readmap_10030100_06092020.txt", "w");
				//fprintf(fp, "");
				for (int k = 0; k < grid_z; k++) {
					for (int j = 0; j < grid_y; j++) {
						for (int i = 0; i < grid_x; i++) {
							fprintf(fp, "%i,", voxel_map_logodd[i][j][k]);
							
						}
					}
				}
				printf("m pressed, Map saved!\n");
				break;
			case 115:																					//S_win
				fp = fopen("map.txt", "w");
				//fprintf(fp, "");
				for (int k = 0; k < grid_z; k++) {
					for (int j = 0; j < grid_y; j++) {
						for (int i = 0; i < grid_x; i++) {
							fprintf(fp, "%i,", voxel_map_logodd[i][j][k]);
							printf("115 Map saved!\n");
						}
					}
				}
				break;
			}
		}

		// traj VAO
		unsigned int trajVAO, trajVBO;
		glGenVertexArrays(1, &trajVAO);
		glGenBuffers(1, &trajVBO);
		glBindVertexArray(trajVAO);
		glBindBuffer(GL_ARRAY_BUFFER, trajVBO);
		glBufferData(GL_ARRAY_BUFFER, scale_trajectory.size() * sizeof(float3), &scale_trajectory[0], GL_DYNAMIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glBindVertexArray(0);

		//opengl camera view
		if (viewpoint == true) {
			fov = 75;
			glm::vec3 front;
			front.x = -cos(camera_global_pose[3]) * sin(-camera_global_pose[4]);
			front.y = sin(camera_global_pose[3]);
			front.z = -cos(camera_global_pose[3]) * cos(-camera_global_pose[4]);
			cameraFront = glm::normalize(front);
			cameraPos = glm::vec3(camera_scaled_pose[0] / unit_length_x - map_shift[0] / block_unit_m,
				-camera_scaled_pose[1] / unit_length_y + map_shift[1] / block_unit_m,
				-camera_scaled_pose[2] / unit_length_x  + map_shift[2] / block_unit_m + 0.1);
			cameraPos -= front;
		}

		//Plane
		const int gridlines = 202;
		float planeVertices[(gridlines + 2) * 2 * 6];
		int gridlinecount = 0;
		for (int k = 0; k < gridlines; k += 2) {
			//-----------------
			planeVertices[6 * k + 0] = 0.0f;
			planeVertices[6 * k + 1] = -specific_row;
			planeVertices[6 * k + 2] = -0.5*k;
			planeVertices[6 * k + 3] = 1.0f;
			planeVertices[6 * k + 4] = 1.0f;
			planeVertices[6 * k + 5] = 1.0f;

			planeVertices[6 * (k + 1) + 0] = 100.0;
			planeVertices[6 * (k + 1) + 1] = -specific_row;
			planeVertices[6 * (k + 1) + 2] = -0.5*k;
			planeVertices[6 * (k + 1) + 3] = 1.0f;
			planeVertices[6 * (k + 1) + 4] = 1.0f;
			planeVertices[6 * (k + 1) + 5] = 1.0f;	//last 131
			//std::cout << planeVertices[6 * k + 0] << "," << planeVertices[6 * k + 2] << ";" << planeVertices[6 * (k + 1) + 0] << "," << planeVertices[6 * (k + 1) + 2] << std::endl;
			gridlinecount++;
			//||||||||||||||||||||
			planeVertices[6 * k + 0 + 6 * (gridlines)] = 0.5*k;	//1st 132
			planeVertices[6 * k + 1 + 6 * (gridlines)] = -specific_row;
			planeVertices[6 * k + 2 + 6 * (gridlines)] = 0.0f;
			planeVertices[6 * k + 3 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * k + 4 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * k + 5 + 6 * (gridlines)] = 1.0f;

			planeVertices[6 * (k + 1) + 0 + 6 * (gridlines)] = 0.5*k;
			planeVertices[6 * (k + 1) + 1 + 6 * (gridlines)] = -specific_row;
			planeVertices[6 * (k + 1) + 2 + 6 * (gridlines)] = -100.0;
			planeVertices[6 * (k + 1) + 3 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * (k + 1) + 4 + 6 * (gridlines)] = 1.0f;
			planeVertices[6 * (k + 1) + 5 + 6 * (gridlines)] = 1.0f;	//263
			//std::cout << planeVertices[6 * k + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * k + 2 + 6 * (gridlines + 2)] << ";" << planeVertices[6 * (k + 1) + 0 + 6 * (gridlines + 2)] << "," << planeVertices[6 * (k + 1) + 2 + 6 * (gridlines + 2)] << std::endl;
			gridlinecount++;
		}
		unsigned int planeVAO, planeVBO;
		glGenVertexArrays(1, &planeVAO);
		glGenBuffers(1, &planeVBO);
		glBindVertexArray(planeVAO);
		glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);	//(x,y,z,R,G,B),each float occupy 4 byte
		glEnableVertexAttribArray(0);
		// color attribute
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));	//last variable is the offset of the color data
		glEnableVertexAttribArray(1);
		glBindVertexArray(0);

		// render
		// ------glbgRender(color);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Start the Dear ImGui frame
		
		

		// activate shader
		planeshader.use();

		// view/projection transformations
		glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 300.0f);
		glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		planeshader.setMat4("projection", projection);
		planeshader.setMat4("view", view);
		
		// render floor--------------floor_render();
		glLineWidth(2.0f);
		glm::mat4 plane = glm::mat4(1.0f);
		glBindVertexArray(planeVAO);
		plane = glm::translate(plane, glm::vec3(- map_shift[0] / block_unit_m, map_shift[1] / block_unit_m, map_shift[2] / block_unit_m));
		planeshader.setMat4("model", plane);
		glDrawArrays(GL_LINES, 0, gridlinecount * 2);
		glBindVertexArray(0);

		// render axes-------------axis_render();
		glLineWidth(6.0f);
		glm::mat4 axis = glm::mat4(1.0f);
		glBindVertexArray(axisVAO);
		planeshader.setMat4("model", axis);
		glDrawArrays(GL_LINES, 0, 6);
		glBindVertexArray(0);

		// render fov-------------fov_render();
		glLineWidth(4.0f);
		glm::mat4 glfov = glm::mat4(1.0f);
		glBindVertexArray(fovVAO);
		glfov = glm::translate(glfov, glm::vec3(camera_scaled_pose[0] / unit_length_x - map_shift[0] / block_unit_m, 
			-camera_scaled_pose[1]/ unit_length_x  + map_shift[1] / block_unit_m, -camera_scaled_pose[2] / unit_length_x + map_shift[2] / block_unit_m));
		//312
		glfov = glm::rotate(glfov, camera_global_pose[4], glm::vec3(0, -1, 0));	//2
		glfov = glm::rotate(glfov, camera_global_pose[5], glm::vec3(0, 0, -1));	//3
		glfov = glm::rotate(glfov, camera_global_pose[3], glm::vec3(1, 0, 0));	//1

		planeshader.setMat4("model", glfov);
		glDrawArrays(GL_LINES, 0, 16);
		glBindVertexArray(0);

		// activate shader for textured boxes
		// be sure to activate shader when setting uniforms/drawing objects
		// shader configuration
		// --------------------
		lightingShader.use();
		lightingShader.setInt("material.diffuse", 0);
		lightingShader.setVec3("light.position", lightPos);
		lightingShader.setVec3("viewPos", cameraPos);

		// light properties
		lightingShader.setVec3("light.ambient", 0.4f, 0.4f, 0.4f);
		lightingShader.setVec3("light.diffuse", 0.5f, 0.5f, 0.5f);
		lightingShader.setVec3("light.specular", 0.2f, 0.2f, 0.2f);

		// material properties
		lightingShader.setVec3("material.specular", 0.5f, 0.5f, 0.5f);
		lightingShader.setFloat("material.shininess", 8.0f);

		lightingShader.setMat4("projection", projection);
		lightingShader.setMat4("view", view);

		// setup texture index
		if(texture_index == 0){
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, wallTexture);
		}
		else if(texture_index == 1){
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, creeperTexture);
		}

		// render boxes
		glBindVertexArray(cubeVAO);

		for (int j = 0; j < grid_y; j++) {
			for (int k = 0; k < grid_z; k++) {
				for (int i = 0; i < grid_x; i++) {
					if (voxel_map_logodd[i][j][k] != 127) {
						if (voxel_map_logodd[i][j][k] >= 180) {
							// calculate the model matrix for each object and pass it to shader bedore drawing
							glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
							model = glm::translate(model,
								glm::vec3((i + 0.5) - map_shift[0] / block_unit_m, -(j + 0.5)+ map_shift[1] / block_unit_m, -(k + 0.5)+ map_shift[2] / block_unit_m));
							//glm::vec3 scale = { probability, probability, probability};
							//model = glm::scale(model, scale);
							lightingShader.setMat4("model", model);
							glDrawArrays(GL_TRIANGLES, 0, 36);
						}
					}
				}
			}
		}

		glBindVertexArray(0);
		glBindTexture(GL_TEXTURE_2D, 0);
		//time elapse
		//clock_t t2 = clock();
		//double duration = t2 - t1;

		gettimeofday(&end, NULL);

		seconds = end.tv_sec - start.tv_sec;
		useconds = end.tv_usec - start.tv_usec;
		mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
		//printf("Elapsed time: %ld milliseconds\n", mtime);

		//t1 = t2;
		//double duration = t_get_depth2 - t_get_depth1;
		//std::cout << "\r" << "time spend: " << std::setprecision(3) << std::fixed << t1 << " " << t2 << " " << duration << std::flush;
		//std::string fps = std::to_string(1 / (duration / CLOCKS_PER_SEC));
		//std::string fps = std::to_string((double)1000/mtime);
		//std::cout << "fps: " << fps << std::endl;
		//std::string layer_row = std::to_string(specific_row);
		//putText(grid2dmap, std::string("fps: " + fps), Point(10, 20), 0, 0.6, Scalar(0, 0, 255), 2);
		//putText(grid2dmap, std::string("Layer: " + layer_row), Point(5, 20), 0, 0.6, Scalar(0, 0, 255), 2);
		//imshow(window_name2, grid2dmap);

		glActiveTexture(GL_TEXTURE2);
		
#ifdef GUI
		ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		unsigned int texture_2d = matToTexture(grid2dmap,GL_LINEAR,GL_LINEAR,GL_CLAMP_TO_EDGE);
		// 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        //if (show_demo_window)
        //   ImGui::ShowDemoWindow(&show_demo_window);
		// 1. Show Connect Infomation
//#ifndef READFILE
		{
			ImGui::Begin("Connect Info");
			ImGui::Text("VOXEL MAP		");	ImGui::SameLine();
			if (show_connect_map)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			
			ImGui::Text("CAMERA STATE	 ");	ImGui::SameLine();
			if (show_connect_camera)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			
			ImGui::Text("MAP SHIFT		");	ImGui::SameLine();
			if (show_connect_map_shift)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			
			ImGui::Text("TRAJECTORY 	  ");	ImGui::SameLine();
			if (show_connect_planned_path)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			
			ImGui::Text("PLANNED PATH	 ");	ImGui::SameLine();
			if (show_connect_trajectory)
				ImGui::TextColored(ImVec4(0.f, 1.f, 0.24f, 1.f), "Connected");
			else
				ImGui::TextColored(ImVec4(1.f, 0.f, 0.f, 1.f), "Disconnected");
			ImGui::Text("%s IP4 Address:		%s\n", ifaname1, IPaddressBuffer1);
			ImGui::Text("%s IP4 Address:	%s\n", ifaname2, IPaddressBuffer2);
			ImGui::End();
		}
		
//#endif
        // 2. Show Map Infomation
        {
            static float f = 0.0f;
            static int counter = 0;
			
            ImGui::Begin("Map Info");                          // Create a window called "Hello, world!" and append into it.
			ImGui::Text("Refresh rate: average %.2f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Checkbox("2D Map", &show_2d_map);
			if(show_2d_map){
				ImGui::Text("2D Map Size (%i, %i)", (int)window_size_2d.x, (int)window_size_2d.y);
				ImGui::SliderInt("Layer",&specific_row,0,grid_y);
				ImGui::Image((void*)(intptr_t)texture_2d, window_size_2d);
			}
            ImGui::Text("This is some useless text.");               // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);
            ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            if (ImGui::Button("Click me plz~"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::End();
        }
		
        // 3. Show another simple window.
        if (show_another_window)
        {
            ImGui::Begin("Another Window", &show_another_window);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window!");
            if (ImGui::Button("Close Me"))
                show_another_window = false;
            ImGui::End();
        }
		
		ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        //glViewport(0, 0, display_w, display_h);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glDeleteTextures(1, &texture_2d);
#endif
		
		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();

		/* Sending UDP Data */
		//ds.process_sending((char*)&x);

		//system("pause");
	}
#ifndef READFILE
	//map_sock.End();
	//cam_sock.End();
	//shift_sock.End();
	data_sock.End();
//	MPC_sock.End();
#endif
	// optional: de-allocate all resources once they've outlived their purpose:
	// ------------------------------------------------------------------------gl_clean();
	glDeleteVertexArrays(1, &cubeVAO);
	glDeleteBuffers(1, &VBO);

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();

	// Cleanup ImGui
#ifdef GUI
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
#endif

#ifdef __WIN32
	infile.close();
#elif __linux

#endif
	//system("pause");
	//std::cout << "Press any key to exit......" << endl;
	return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	// moving mode
	float cameraSpeed = 0.5f; // can adjust
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS){
		FILE *fp;
		fp = fopen("../res/readmap_10030100_06092020.log", "w");
		fprintf(fp, "");
		for (int k = 0; k < grid_z; k++) {
			for (int j = 0; j < grid_y; j++) {
				for (int i = 0; i < grid_x; i++) {
					fprintf(fp, "%i,", voxel_map_logodd[i][j][k]);
					
				}
			}
		}
		printf("m pressed, Map saved!\n");
	}
	
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS){
		texture_index = 0;
	}
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS){
		texture_index = 1;
	}
	lightPos = cameraPos + glm::vec3(2.0f, 0.0f, -0.5f);
}

void mouse_button_callback(GLFWwindow * window, int button, int action, int mode)
{
#ifdef GUI
	ImGuiIO &io = ImGui::GetIO();
    (void)io;
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !io.WantCaptureMouse) {
		//std::cout << "Left Key Pressed!" << std::endl;
		rotation_started = true;
	}
#else
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		//std::cout << "Left Key Pressed!" << std::endl;
		rotation_started = true;
	}
#endif //GUI

	else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		//std::cout << "Right Key Pressed!" << std::endl;
		//move viewpoint to realsense position
		if (viewpoint == false) {
			viewpoint = true;
		}
		else if (viewpoint == true) {
			viewpoint = false;
			cameraPos = cameraPos + glm::vec3(0.0f, 0.0f, 2.0f);
			fov = 60;
		}
	}
	else if (button == GLFW_RELEASE) {
		//std::cout << "Left Key Released!" << std::endl;
		rotation_started = false;
		firstMouse = true;
	}
}

void mouse_position_callback(GLFWwindow * window, double xpos, double ypos)
{
	if (rotation_started) {
		if (firstMouse)	// this bool variable is initially set to true
		{
			start_x = xpos;
			start_y = ypos;
			firstMouse = false;
		}

		if (rotation_started) {
			float xoffset = start_x - xpos;
			float yoffset = ypos - start_y;	// reversed since y-coordinates range from bottom to top
			start_x = xpos;
			start_y = ypos;

			float sensitivity = 0.1;
			xoffset *= sensitivity;
			yoffset *= sensitivity;

			yaw += xoffset;
			pitch += yoffset;
		}
		if (pitch > 89.0f)
			pitch = 89.0f;
		if (pitch < -89.0f)
			pitch = -89.0f;
	}

	glm::vec3 front;
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraFront = glm::normalize(front);
}

void scroll_callback(GLFWwindow * window, double xoffset, double yoffset)
{
	if (fov >= 1.0f && fov <= 70.0f)
		fov -= yoffset;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 70.0f)
		fov = 70.0f;
}

void mouseLeftPressEvent(GLFWwindow * window, double xpos, double ypos)
{
	rotation_started = true;
	start_x = xpos;
	start_y = ypos;
}

void mouseLeftReleaseEvent()
{
	rotation_started = false;

}

void mouseLeftDragEvent(GLFWwindow * window, double xpos, double ypos)
{
	if (rotation_started)
	{
		float xoffset = (ypos - start_y);
		float yoffset = (xpos - start_x);

		float sensitivity = 0.05;
		xoffset *= sensitivity;
		yoffset *= sensitivity;

		yaw += xoffset;
		pitch += yoffset;

		if (pitch > 89.0f)
			pitch = 89.0f;
		if (pitch < -89.0f)
			pitch = -89.0f;

		glm::vec3 front;
		front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
		front.y = sin(glm::radians(pitch));
		front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
		cameraFront = glm::normalize(front);
	}
}

void glInit(GLFWwindow * window)
{
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// glfw mouse input
	// ---------------gl_Mouse();
	glfwSetCursorPosCallback(window, mouse_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// tell GLFW to capture our mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	// glad: load all OpenGL function pointers
	// ---------------------------------------glad_Init();
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
	}

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_FRONT);
	//glFrontFace(GL_CW);
}

void glSetup(unsigned int VAO, unsigned int VBO, float Vertices[], unsigned int AttribSize1, unsigned int AttribSize2, unsigned int Stride, unsigned int VertexOffset)
{
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
	// position attribute
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, AttribSize1, GL_FLOAT, GL_FALSE, Stride * sizeof(float), (void*)0);
	// texture coord attribute
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, AttribSize2, GL_FLOAT, GL_FALSE, Stride * sizeof(float), (void*)(VertexOffset * sizeof(float)));
	glBindVertexArray(0);	// unbind
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const * path)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);

	int width, height, nrComponents;
	unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
	stbi_set_flip_vertically_on_load(true);
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;

		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}
	glBindTexture(GL_TEXTURE_2D, 0);
	return textureID;
}

//Mat to GL texture
unsigned int matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
    // Generate a number for our textureID's unique handle
    unsigned int textureID;
    glGenTextures(1, &textureID);

    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Catch silly-mistake texture interpolation method for magnification
    if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
        magFilter == GL_LINEAR_MIPMAP_NEAREST ||
        magFilter == GL_NEAREST_MIPMAP_LINEAR ||
        magFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
        magFilter = GL_LINEAR;
    }

    // Set texture interpolation methods for minification and magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

    // Set incoming texture format to:
    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
    // Work out other mappings as required ( there's a list in comments in main() )
    GLenum inputColourFormat = GL_BGR;
    if (mat.channels() == 1)
    {
        inputColourFormat = GL_RG;
    }

    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
        0,                 // Pyramid level (for mip-mapping) - 0 is the top level
        GL_RGB,            // Internal colour format to convert to
        mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
        mat.rows,          // Image height i.e. 480 for Kinect in standard mode
        0,                 // Border width in pixels (can either be 1 or 0)
        inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
        GL_UNSIGNED_BYTE,  // Image data type
        mat.ptr());        // The actual image data itself

    // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
    if (minFilter == GL_LINEAR_MIPMAP_LINEAR ||
        minFilter == GL_LINEAR_MIPMAP_NEAREST ||
        minFilter == GL_NEAREST_MIPMAP_LINEAR ||
        minFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        glGenerateMipmap(GL_TEXTURE_2D);
    }
	glBindTexture(GL_TEXTURE_2D, 0);
    return textureID;
}

//float 3
float3 float3::operator*(float t)
{
	return { x * t, y * t, z * t };
}

float3 float3::operator-(float t)
{
	return { x - t, y - t, z - t };
}

void float3::operator*=(float t)
{
	x = x * t;
	y = y * t;
	z = z * t;
}

void float3::operator=(const float3 &other)
{
	x = other.x;
	y = other.y;
	z = other.z;

}

void float3::add(float t1, float t2, float t3)
{
	x += t1;
	y += t2;
	z += t3;
}