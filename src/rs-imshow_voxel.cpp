/* ---------------------------------
 * Wen-Yu Chien,  PSU UAS Research Laboratory (PURL), June 2020
 * Research Assistant, Penn State University
 * email : wuc188@psu.edu, leochien1110@gmail.com
 * Advicer: Eric Johnson
 * --------------------------------*/

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <math.h>  //tan cos
#include <time.h>
//#include <omp.h>	// multiple process
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <signal.h>
#include <thread>
#include <mutex>
#include <functional>
#include <vector>

// opengl
//#define GLFW_INCLUDE_GLU
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION    
#include "stb_image.h"
#include "shader_m.h"

// Linux Only
#include <sys/socket.h>	//unix socket
#include <sys/types.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <termios.h>	//unix m1 key event
#include <unistd.h>

using namespace cv;
using namespace std::chrono;
#define PI 3.14159265358979323846

//***************//
//FUNCTION SWITCH//
//***************//
//#define RT	// Ray Tracing, 0:Line Drawing / 1:Voxel Ray Traversa
#define GL		// 0:disable, 1:enable
#define CV		// 0:disable, 1:enable
//#define TimeCost	// Calculate the time cost of the code
//#define Map_Expand 	// map shift function
//#define SOCK
//#define VICON
const char * map_dir = "../../logger/map/10030100_06292020.log";

#ifdef VICON
	#include "DataStreamClient.h"	// VICON Stream Client
	#include <time.h>
	#include <sys/time.h>
	#include <iostream>
	#include <fstream>
	#include <cassert>
	#include <cstdlib>
	#include <ctime>
	#include <vector>
	#include <string.h>
	#include <stdio.h>
	#include <string>
	#include <time.h>
	#include <chrono>  
	#define output_stream std::cout 
#endif //VICON

//key event for linux
#ifdef __linux__
static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard();
void close_keyboard();
int _kbhit();
int readch();
#endif // __linux__

// Pthread mutex initialize
std::mutex mutex_t_m2g;
std::mutex mutex_t_c2g;
std::mutex mutex_t_s2g;
std::mutex mutex_t;

struct float4 {
	float w, x, y, z;
	float4 operator*(float t);
	float4 operator-(float t);
	void operator*=(float t);
	void operator=(const float4 &other);
	void add(float t1, float t2, float t3, float t4);
};

struct float3 {
	float x, y, z;
	float3 operator*(float t);
	float3 operator-(float t);
	void operator*=(float t);
	void operator=(const float3 &other);
	void add(float t1, float t2, float t3);
};

double exp_fast(double x)
{
  if(x<-500){x = 0;    return x;}
  x = 1.0 + x/256;  x *= x;  x *= x;  x *= x;
  x *= x;  x *= x;  x *= x;  x *= x;  x *= x;
  return x;
}

float4 Quat2Euler(float4 q);
void calc_transform(rs2_pose& pose_data, float mat[16]);

//--------------------
// Voxel Map Dimension
//--------------------
//Voxel map size
const float block_unit_m = 0.2;	//(meter/cell): change this then change grid
const int unit_length_x = 5;	//(pixel/cell)
const int unit_length_y = 5;	//(pixel/cell)
const int unit_length_z = 5;	//(pixel/cell)

//Map scale in real world (pixel per meter), can use to transfer map from real world to voxel world
const float mapscale_x = unit_length_x / block_unit_m;	//
const float mapscale_y = unit_length_y / block_unit_m;
const float mapscale_z = unit_length_z / block_unit_m;	//

// Map Dimension(meter)
const int LENGTH = 20;
const int HEIGHT = 6;	//celling = 0
const int WIDTH  = 20;

// Voxel Map Resolution
const int v_x = LENGTH * mapscale_x;
const int v_y = HEIGHT * mapscale_y;
const int v_z =  WIDTH * mapscale_z;	

const int grid_x = 100;	//v_x / unit_length_x	
const int grid_y = 30;	//v_y / unit_length_y
const int grid_z = 100;	//v_z / unit_length_z

//-----------------
// Camera Parameter
//-----------------
// Camera resolution
const int width = 848;	//1280,848,640,480, 424
const int height = 480;	//720,480,360,270,240
const int resolution = width * height;
const int desired_fps = 30;

// Reduce resolution
const int reduced_ratio = 4;	//1,2,4,8,10,16,20
const int reduced_w = width / reduced_ratio;
const int reduced_h = height / reduced_ratio;
const int reduced_res = reduced_w * reduced_h;
const int reduced_ratio_square = reduced_ratio * reduced_ratio;
int si, sj, sn, ri, rj;

// Field of View
float FOVxz = 1.487021;
float FOVyz = 1.01229;
float half_FOVxz = FOVxz / 2;
float half_FOVyz = FOVyz / 2;
float FOV[2];
float f1_2 = 1/block_unit_m;
float f1_0 = tan(half_FOVxz)*f1_2;
float f1_1 = tan(half_FOVyz)*f1_2;
float f1[3] = { f1_0, f1_1, f1_2 };

// D435 parameters
float baseline = 0.05;
float focallen = 0.5*width/tan(half_FOVxz);
float subpixel = 0.08;
float sensor_para = subpixel/baseline/focallen;
float p_min = 0.1;
// d^2*reduced_ratio^2/86^2; 86px within 0.2m square @1m
float pixel_density = 86*86/reduced_ratio/reduced_ratio;

float max_distance = 16;
float min_distance = 0.3;
float init_camera_global_pos[3] = { LENGTH/2 , HEIGHT/2 , WIDTH/2 };	//meter
//camera position in memter:
float camera_global_pose[12] = { 0 };	//(x,y,z,phi,theta,psi)
float camera_pixel_pose[3] = { 0 };	//camera position in pixel
float camera_state[6] = { 0 };

// Rotation angle
float4 q;

// Voxel Map Data array
unsigned char voxel_map_logodd[grid_x][grid_y][grid_z];
unsigned char voxel_map_logodd_old[grid_x][grid_y][grid_z];
bool initial_voxel_map[grid_x][grid_y][grid_z];	//make sure cell has been visited

int map_shift[3] = { 0 };	// (0,0,0), once the camera is close to boundary, shift the whole cells
int mps = 0;				// map shift step

// Connection
bool socket_connect = false;
bool mapper_status = false;

// Socket Data Sending
class DataSend {
public:
	int sockfd = 0, Clientsockfd = 0;
	struct sockaddr_in serverInfo,clientInfo;
	unsigned int addrlen = sizeof(clientInfo);
	int ret = -1, sen = -1, err = -1, port_num_c = 0;
	char message[30] = {"Hi,this is server.\n"};

	// Socket declared
	DataSend() {
		printf("Socket called!\n");
	}

	//check errors
	void init(uint16_t port_num) {
		port_num_c = port_num;
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if(sockfd == -1){
			printf("Fail to create a socket!\n");
		}
		int nSendBuf=512*1024;//512K
		setsockopt(sockfd,SOL_SOCKET,SO_SNDBUF,(const char*)&nSendBuf,sizeof(int));
		signal(SIGPIPE, SIG_IGN);
		//************************
		bzero(&serverInfo,sizeof(serverInfo));
		serverInfo.sin_family = PF_INET;
		// local host test
		serverInfo.sin_addr.s_addr = inet_addr("192.168.0.150");	//127.127.0.0.1		104.39.160.46	104.39.89.30 192.168.0.150
		serverInfo.sin_port = htons(port_num);
		err = connect(sockfd,(struct sockaddr *)&serverInfo,sizeof(serverInfo));
		if(err==-1){
			//printf("Connection error\n");
			socket_connect = false;
		}
		else{
			printf("Connected!\n");
			socket_connect = true;
		}
	}

	void senddata(){
		while(mapper_status){
			if(!socket_connect){
				/*printf("\033[1A"); //go back to previous row
				printf("\033[K");  //flush
				printf("\033[1A");
				printf("\033[K");
				printf("\033[1A");
        		printf("\033[K");*/
				End();
				//printf("Reinitializing...\n");
				init(port_num_c);
				std::this_thread::sleep_for(std::chrono::duration<int, std::milli>(100));
			}
			else{
				clock_t t01;
				sendmap();
				sendcam();
				sendshift();
				clock_t t04;
			}
		}
	}
	void sendmap(){
		unsigned char buf[grid_x][grid_y][grid_z];
		int buf_size = sizeof(buf);
		//std::lock_guard<std::mutex> mlcok(mutex_t_s2g);
		mutex_t_s2g.lock();
		for (int i = 0; i < grid_x; i++) {
			for (int j = 0; j < grid_y; j++) {
				for (int k = 0; k < grid_z; k++) {
					buf[i][j][k] = voxel_map_logodd[i][j][k];
				}
			}
		}
		mutex_t_s2g.unlock();
		if(err != -1){
			process_sending(buf, buf_size, 0);
		}
		//printf("sendmap sent: %d\n", sen);
	}

	void sendcam(){
		float buf[12];
		int buf_size = sizeof(buf);
		//std::lock_guard<std::mutex> mlcok(mutex_t_c2g);
		mutex_t_c2g.lock();
		for (int i = 0; i < 12; i++) {
			buf[i]=camera_global_pose[i];
		}
		mutex_t_c2g.unlock();
		if(err != -1){
			process_sending(buf, buf_size, 0);
		}		
		//printf("sendcam sent: %d\n", sen);
	}

	void sendshift(){
		int buf[3];
		int buf_size = sizeof(buf);
		//std::lock_guard<std::mutex> mlcok(mutex_t_s2g);
		mutex_t_s2g.lock();
		for (int i = 0; i < 3; i++) {
			buf[i]=map_shift[i];
		}
		mutex_t_s2g.unlock();
		if(err != -1){
			process_sending(buf, buf_size, 0);
		}		
		//printf("sendshift sent: %d\n", sen);
		std::cout << map_shift[0] << " " << map_shift[1] << " " << map_shift[2] << " " << std::endl;
	}

	void process_sending (void * sendbuf, int SIZE, bool flag) {		//void * sendbuf, int SIZE, bool flag
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
			printf("Sending error!\n");
			socket_connect = false;
			End();
		}
		else if(sen == 0){
			printf("Sending disconnected!\n");
			socket_connect = false;
			End();
		}
	}

	// data receive from server
	int process_receiving(void * recvbuf, int SIZE) {
		//printf("processing received data...\n");
		ret = recv(Clientsockfd, recvbuf, SIZE, 0);
		if(ret == -1){
			printf("Receiving error!\n");
		}
        return ret;
	}

	void End() {
		close(sockfd);
		printf("Socket Closed!\n");
	}
};

// Connect Info
bool show_connect_map = true;
bool show_connect_camera = true;
bool show_connect_map_shift = true;
bool show_connect_planned_path = true;
bool show_connect_trajectory = true;

void ray_traversal(int raydir[], float ray_m[], int dist[],float z_m, float obj_distance);
void line_drawing(float position[], float ray_length, int dist[], float z_m);
void render_vmap(bool type, int cube[], float r_distance, float z_m, float obj_distance);
void shift_map(int axis);

#ifdef CV
static void CVonMouse(int event,int x,int y,int,void*);
#endif //CV

#ifdef VICON
void startViconThread();
void quat_mult( double a[4], double b[4], double c[4]);
#endif //VICON

#ifdef GL
//---------------------
// OpenGL Visualization
//---------------------
void glInit(GLFWwindow * window);
void glSetup(unsigned int VAO, unsigned int VBO, float Vertices[], unsigned int AttribSize1,
	unsigned int AttribSize2, unsigned int Stride, unsigned int VertexOffset);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
unsigned int loadTexture(char const * path);

//Input Setup
void processInput(GLFWwindow *window);
void mouse_button_callback(GLFWwindow * window, int button, int action, int mode);
void mouse_position_callback(GLFWwindow * window, double xpos, double ypos);
void scroll_callback(GLFWwindow * window, double xoffset, double yoffset);
void mouseLeftPressEvent(GLFWwindow * window, double xpos, double ypos);
void mouseLeftReleaseEvent();
void mouseLeftDragEvent(GLFWwindow * window, double xpos, double ypos);

// 3D Map Window settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

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
float lastX = 800.0f / 2.0;
float lastY = 600.0 / 2.0;
float fov = 75.0f;
bool rotation_started = false;
bool viewpoint = true;
int start_x = 0;
int start_y = 0;

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
#endif

int main(int argc, char * argv[]) try
{
	// Initial time for frame rate
	clock_t t1 = clock();

	//----------------//
	// Initialize map //
	//----------------//
	
	// Voxel map
	for (int i = 0; i < grid_x; i++) {
		for (int j = 0; j < grid_y; j++) {
			for (int k = 0; k < grid_z; k++) {
				voxel_map_logodd[i][j][k] = 127;
				voxel_map_logodd_old[i][j][k] = 127;
				initial_voxel_map[i][j][k] = 0;
			}
		}
	}

	//---------------//
	/* Cameras Setup */
	//---------------//
	rs2::sensor depth_sensor;
	rs2::sensor color_sensor;
	rs2::sensor pose_sensor;

	// Create realsense context for managing devices
	rs2::context	ctx;
	rs2::pointcloud pc;
	rs2::points points;

	std::vector<float> scale_trajectory;
	std::vector<float> trajectory;
	
	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	
	// Declare filters
	rs2::decimation_filter deci_filter;
	rs2::threshold_filter thhd_filter;
	rs2::spatial_filter spat_filter;
	rs2::temporal_filter temp_filter;
	rs2::hole_filling_filter hole_filter;
	
	// Configure filter parameters
	deci_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	thhd_filter.set_option(RS2_OPTION_MAX_DISTANCE, max_distance); //max depth range
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 8);
	hole_filter.set_option(RS2_OPTION_HOLES_FILL, 0);

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	std::vector<rs2::pipeline>  pipelines;

	// Check avaible cameras
	rs2::device_list devices = ctx.query_devices();
	std::cout << "Found " << devices.size() << " devices " << std::endl;
	if (devices.size() != 2) {
		std::cout << "[WARNING]There's no/only one camera connected.\n\
		Please unplug USB and try again." << std::endl;
		return 0;
	}

	// Setup Pipeline for each device
	for (auto&& dev : devices)
	{
		const char* dev_name = dev.get_info(RS2_CAMERA_INFO_NAME);
		printf("dev_name:%s\n",dev_name);
		const char* dev_num = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		rs2::pipeline pipe(ctx);

		// Setup configuration
		rs2::config cfg;
		cfg.enable_device(dev_num);
		auto advanced_sensors = dev.query_sensors();

		for (auto&& sensor : advanced_sensors) {
			std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
			std::cout << module_name << std::endl;

			if (module_name == "Stereo Module") {
				depth_sensor = sensor;
			} else if (module_name == "RGB Camera") {
				color_sensor = sensor;
			} else if (module_name == "Tracking Module") {
				pose_sensor = sensor;
			}
		}
		
		// D435 & D435i
		if (strcmp("Intel RealSense D435", dev_name) == 0 || strcmp("Intel RealSense D435I", dev_name) == 0) {
			std::cout << "Camera " << dev.get_info(RS2_CAMERA_INFO_NAME) << " Connected" << std::endl;
			cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, desired_fps);
			cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, desired_fps);
			
			rs2::pipeline_profile selection = pipe.start(cfg);
			rs2::device selected_device = selection.get_device();
			auto d_sensor = selected_device.first<rs2::depth_sensor>();
			// Functions
			d_sensor.set_option(RS2_OPTION_VISUAL_PRESET,rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
			if(d_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
				d_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
			if (d_sensor.supports(RS2_OPTION_LASER_POWER))
				d_sensor.set_option(RS2_OPTION_LASER_POWER, 150);
			pipelines.emplace_back(pipe);
		}
		// T265
		else if (strcmp("Intel RealSense T265", dev_name) == 0) {
            cfg.enable_stream(RS2_STREAM_POSE,RS2_FORMAT_6DOF);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 1);
            cfg.disable_stream(RS2_STREAM_FISHEYE, 2);
			std::cout << "Camera: " << dev.get_info(RS2_CAMERA_INFO_NAME) << " Connected" << std::endl;
			
			// Functions have to be set before streaming
			pose_sensor.set_option(RS2_OPTION_ENABLE_MAPPING, 1.f);
			pose_sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 1.f);
			pose_sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 1.f);
			
			pipe.start(cfg);
			pipelines.emplace_back(pipe);
		}
		sleep(1);	// wait 1 sec to setup next device
	}
	
	// Create Windows for 2D map
#ifdef CV
	const auto window_name = "Depth Image";
	//namedWindow(window_name, WINDOW_AUTOSIZE);
	const auto window_name2 = "2D Map_xz";
	namedWindow(window_name2, WINDOW_AUTOSIZE);
#endif
	// Print Infomation
	std::string b_unit = std::to_string(block_unit_m);
	std::cout << "Cell Unit:" << block_unit_m << "(m/square)" << std::endl;
	std::cout << "Reduced Map Width:" << reduced_w << std::endl;
	std::cout << "\nIn 2D Map, press S to save map\n↑ & ↓ to move layer of 2D map\nHold ESC to quit!" << std::endl;

	//show specific 2D map
	int specific_row = init_camera_global_pos[1] / block_unit_m;		//ex: specific row-> y = 15	, height/2

#ifdef SOCK
	//------//
	//Socket//
	//------//
	DataSend map_2GS, cam_2GS, shift_2GS, data2GS;	//send data to ground station	
	data2GS.init(7777);
#endif

#ifdef GL
	//------------//
	//OpenGL Setup//
	//------------//
	// glfw: initialize and configure(version 3.3)
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// OpenGL Window
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Voxel 3D", NULL, NULL);
	// Create a simple OpenGL window for rendering:
	// GLFWwindow* window2 = glfwCreateWindow(width, height, "RealSense Measure Example", NULL, NULL);

	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glInit(window);

	// build and compile shader program
	Shader lightingShader("../res/texture02.vs", "../res/texture02.fs");
	Shader planeshader("../res/grid.vs", "../res/grid.fs");
	
	// -----------
	// Voxel Cells
	// -----------
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

	// load textures (we now use a utility function to keep the code more organized)
	// -----------------------------------------------------------------------------
	unsigned int diffuseMap = loadTexture("../res/wall.jpg");

	// -----
	// Plane
	// -----
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

		planeVertices[6 * (k + 1) + 0] = grid_x;
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
		planeVertices[6 * (k + 1) + 2 + 6 * (gridlines)] = -grid_x;
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
	
	// ----
	// Axis
	// ----	
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

	// -------------
	// Field of View
	// -------------
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
#endif //GL

#ifdef SOCK
	// --------------
	// Pthread socket
	// --------------
	mapper_status = true;
	std::thread t_send(&DataSend::senddata, data2GS);
#endif

#ifdef VICON	
	std::thread t_vicon(&startViconThread);
#endif	//VICON
	
	// Save log
	std::string parentdir = "../../logger/t265"; //03112020_boxPos_01.txt
	std::string filepath,filepath2;
	time_t now = time(NULL);
	struct tm *info;
	info = localtime(&now);
	char date_time[30];
	strftime(date_time,30,"%y-%m-%d_%H-%M",info);
	std::string filetime(date_time);
	filepath +=  parentdir + "/pose_" + filetime + ".log";
	std::string filename = filetime;
	std::cout << filename << std::endl;

	std::ofstream fileID;
	fileID.open(filepath, std::ofstream::out | std::ofstream::app);
	if(!fileID.is_open()){
		printf("\n\nCould not open file");
		return 0;
	}

#ifdef TimeCost
	std::ofstream fileID2;
	filepath2 +=  parentdir + "/mapping time_" + filetime + ".csv";
	fileID2.open(filepath2, std::ofstream::out | std::ofstream::app);
	if(!fileID2.is_open()){
		printf("\n\nCould not open file2");
		return 0;
	}
	time_point<system_clock,microseconds> tt1 = time_point_cast<microseconds>(system_clock::now());
#endif //TimeCost

	// Start Steaming
#ifdef GL
	while (waitKey(1) != 27 && !glfwWindowShouldClose(window)){
#else
	while (waitKey(1) != 27){
#endif //GL

#ifdef GL
		//------------
		//OpenGL input
		//------------
		processInput(window);	//mouse and keyboard input
#endif //GL
		
		//--------------
		// OpenCV 2D Map
		//--------------
#ifdef CV		
		Mat grid2dmap(v_x, v_z, CV_8UC3, Scalar(255, 255, 255));	//xz
		// Draw grid
		for (int i = 1; i < grid_z; i++) {
			line(grid2dmap, Point(i * unit_length_x, 0), Point(i * unit_length_x, v_x), Scalar(0, 0, 0), 1);
		}
		for (int i = 1; i < grid_x; i++) {
			line(grid2dmap, Point(0, i * unit_length_z), Point(v_z, i * unit_length_z), Scalar(0, 0, 0), 1);
		}
#endif

		/* D435 Stream */
        rs2::frameset data = pipelines[0].wait_for_frames();	//Wait for next set of frames from the camera
		//rs2::depth_frame d = data.get_depth_frame();			//Get depth data
		rs2::frame color_frame = data.get_color_frame();		//Get color frame
		//d = d.apply_filter(spat_filter);
		//d = d.apply_filter(thhd_filter);
		rs2::frame depth = data.get_depth_frame();	//Visualize 2D depth image
		//depth = thhd_filter.process(depth);
		//depth = spat_filter.process(depth);
		//depth = depth.apply_filter(color_map);	// colorize depth

		//point cloud
		pc.map_to(color_frame);
		points = pc.calculate(depth);						// heavy function :(
		auto vertices = points.get_vertices();              // get vertices

		/* T265 Stream */
    	auto frames = pipelines[1].wait_for_frames();
        if( rs2::pose_frame f = frames.first_or_default(RS2_STREAM_POSE) ) {
    		auto pose_data = f.get_pose_data();	// Get states data(x,y,z,qw,qx,qy,qz)
			unsigned int tracker_confidence = pose_data.tracker_confidence;
			//printf("tracker_confidence: %i\n",tracker_confidence);
    		
			// Quaternion(w,x,y,z)
    		q.w =  pose_data.rotation.w;
    		q.x = -pose_data.rotation.z;
    		q.y =  pose_data.rotation.x;
    		q.z = -pose_data.rotation.y;

			// Conver quaternion to euler angle
			float4 angle = Quat2Euler(q);
			// Position
			camera_state[0] = pose_data.translation.x;	//X
			camera_state[1] = -pose_data.translation.y;	//Y
			camera_state[2] = -pose_data.translation.z;	//Z
			//convert to y-z-x order
			camera_state[3] = angle.y;					//camera_global_pose[3]
			camera_state[4] = angle.z;					//camera_global_pose[4]
			camera_state[5] = angle.x;					//camera_global_pose[5]
			//x-z-y
			//camera_state[3] = angle.x;					//camera_global_pose[3]
			//camera_state[4] = angle.z;					//camera_global_pose[4]
			//camera_state[5] = angle.y;					//camera_global_pose[5]
			//y-x-z
			//camera_state[3] = angle.y;					//camera_global_pose[3]
			//camera_state[4] = angle.x;					//camera_global_pose[4]
			//camera_state[5] = angle.z;					//camera_global_pose[5]
        }

#ifdef Map_Expand
		mutex_t_s2g.lock();
		for (int i  = 0; i < 3; i++){
			if((camera_state[i] + map_shift[i]) < -0.5){	//hit lower boundary, ++
				mps = 1;
				map_shift[i] = map_shift[i] + mps;
				shift_map(i);
			}
			else if((camera_state[i] + map_shift[i]) > 0.5){	//hit upper boundary, --
				mps = -1;
				map_shift[i] = map_shift[i] + mps;
				shift_map(i);
			}
			mps = 0;
		}
		mutex_t_s2g.unlock();
#endif
		
		time_point<system_clock,microseconds> ts = time_point_cast<microseconds>(system_clock::now());
		uint64_t time_stmp = ts.time_since_epoch().count();
		{
			mutex_t_c2g.lock();
			//std::lock_guard<std::mutex> mlcok(mutex_t_c2g);
			camera_global_pose[0] = init_camera_global_pos[0] + camera_state[0] + map_shift[0];
			camera_global_pose[1] = init_camera_global_pos[1] + camera_state[1] + map_shift[1];
			camera_global_pose[2] = init_camera_global_pos[2] + camera_state[2] + map_shift[2];
			camera_global_pose[3] = 0 + camera_state[3];
			camera_global_pose[4] = 0 + camera_state[4];
			camera_global_pose[5] = 0 + camera_state[5];
			mutex_t_c2g.unlock();
		}
		
		camera_pixel_pose[0] = camera_global_pose[0] * mapscale_x;	//(pixel)
		camera_pixel_pose[1] = camera_global_pose[1] * mapscale_y;
		camera_pixel_pose[2] = camera_global_pose[2] * mapscale_z;
		
		// add new point in the trajectory (if motion large enough to reduce size of traj. vector)
		float3 traj;
		float3 traj_color;
		traj.x =  camera_pixel_pose[0] / unit_length_x - map_shift[0] / block_unit_m; //X(cell)
		traj.y = -camera_pixel_pose[1] / unit_length_y + map_shift[1] / block_unit_m; //Y
		traj.z = -camera_pixel_pose[2] / unit_length_z + map_shift[2] / block_unit_m; //Z
		traj_color.x = 0.0f;
		traj_color.y = 1.0f;
		traj_color.z = 0.0f;
		//std::cout << "trajectory.z:" << trajectory.z << std::endl;
		
		if (scale_trajectory.size() == 0){
			//scale_trajectory.push_back(trajectory);
			//scale_trajectory.push_back(traj_color);
			scale_trajectory.push_back(traj.x);
			scale_trajectory.push_back(traj.y);
			scale_trajectory.push_back(traj.z);
			scale_trajectory.push_back(0.0f);
			scale_trajectory.push_back(1.0f);
			scale_trajectory.push_back(0.0f);
		}
		else {
			float3 prev;
			prev.x = scale_trajectory[scale_trajectory.size() - 6];
			prev.y = scale_trajectory[scale_trajectory.size() - 5];
			prev.z = scale_trajectory[scale_trajectory.size() - 4];
			float3 curr = traj;
			//std::cout << "scale_trajectory.back().z:" << scale_trajectory.back().z << std::endl;
			//std::cout << "prev.z:" << prev.z << std::endl;
			if (sqrt(pow((curr.x - prev.x), 2) + pow((curr.y - prev.y), 2) + pow((curr.z - prev.z), 2)) > 0.002){
				//scale_trajectory.push_back(trajectory);
				//scale_trajectory.push_back(traj_color);
				scale_trajectory.push_back(traj.x);
				scale_trajectory.push_back(traj.y);
				scale_trajectory.push_back(traj.z);
				scale_trajectory.push_back(traj_color.x);
				scale_trajectory.push_back(traj_color.y);
				scale_trajectory.push_back(traj_color.z);
			}
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

		float pc_localpos[3];
		rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
		rs2_fov(&intr, FOV);
		half_FOVxz = FOV[0] * 0.00872664625997;	// (pi/180)/2 =  0.00872664625997
		half_FOVyz = FOV[1] * 0.00872664625997;
		float cm_fovxz = 1 / tan(half_FOVxz);

		// Get coordinate and reduce the resolution
		float3 reduced_vertices[reduced_res];
		for (int i = 0; i < reduced_res; i++) {
			reduced_vertices[i] = {10,10,10};
		}

		for (int i = 0; i < points.size(); i++)
		{
			if(vertices[i].z && vertices[i].z > min_distance){
				int ri = 0;
				si = i / reduced_ratio % reduced_w;
				sj = i / reduced_ratio / width;
				ri = si + sj * reduced_w; 

				if(vertices[i].z < reduced_vertices[ri].z){
					reduced_vertices[ri].x = vertices[i].x;
					reduced_vertices[ri].y = vertices[i].y;
					reduced_vertices[ri].z = vertices[i].z;
				}
			}
		}
		
		//----------------------------
		// Update a 3D Grid Map Cells
		//----------------------------
		for (int i = 0; i < reduced_res; i++)
		{
			if (vertices[i].z)
			{
				pc_localpos[0] = reduced_vertices[i].x;
				pc_localpos[1] = reduced_vertices[i].y;
				pc_localpos[2] = reduced_vertices[i].z;

				//pointcloud position relative to camera, real world (meter)
				float pc_position[3];
				pc_position[0] = 0;
				pc_position[1] = 0;
				pc_position[2] = 0;

				/* Rotation */
				//rotation correction [camera frame] (meter), pt1 = invC * pt1', for loop: matrix multiply
				for (int l = 0; l < 3; l++) {
					for (int m = 0; m < 3; m++) {
						pc_position[l] += inv_C[l][m] * pc_localpos[m];
					}
				}

				/* Unit Conversion */
				//convert pc position to [local frame] (meter → pixel)
				pc_position[0] *= mapscale_x;
				pc_position[1] *= mapscale_y;
				pc_position[2] *= mapscale_z;

				//convert pc position to [global frame] (pixel)
				float v[3];	//point cloud map coordinate
				v[0] = camera_pixel_pose[0] + pc_position[0];
				v[1] = camera_pixel_pose[1] + pc_position[1];
				v[2] = camera_pixel_pose[2] + pc_position[2];

				int r[3];						//ray 
				float r_length;					//ray length
				float r_unit_length;			//unit ray length
				int length_factor;				//use to extend r_length
				float dx, dy, dz;				//unit ray x - vector
				float obj_distance = sqrt(pow(pc_position[0], 2) + pow(pc_position[1], 2) + pow(pc_position[2], 2));	//ray_distance

				/* Occupied Cell */
				int obj_cube[3];	// pointcloud/object position
				int raydir[3];		// ray direction vector
				float ray_m[3];		// ray slope
				float ray_m_hypo = sqrt(raydir[0]*raydir[0]+raydir[1]*raydir[1]+raydir[2]*raydir[2]);	// ray hypotenuse

				obj_cube[0] = v[0] / unit_length_x;
				obj_cube[1] = v[1] / unit_length_y;
				obj_cube[2] = v[2] / unit_length_z;
				
				raydir[0] = v[0]-camera_pixel_pose[0];
				raydir[1] = v[1]-camera_pixel_pose[1];
				raydir[2] = v[2]-camera_pixel_pose[2];
				
				ray_m[0] = ray_m_hypo/raydir[0];
				ray_m[1] = ray_m_hypo/raydir[1];
				ray_m[2] = ray_m_hypo/raydir[2];
				
				/**************************/
				/***Occupied Cell Assign***/
				/**************************/
				if (obj_cube[0] >= grid_x || obj_cube[1] >= grid_y || obj_cube[2] >= grid_z || obj_cube[0] < 0 || obj_cube[1] < 0 || obj_cube[2] < 0
					|| obj_cube[0] - 1 < 0 || obj_cube[1] - 1 < 0 || obj_cube[2] - 1 < 0)
					continue;

				render_vmap(true, obj_cube, obj_distance, pc_localpos[2], obj_distance);

				/*************************************/
				/***Ray Traversal (Ray Cell Assign)***/
				/*************************************/
#ifdef RT
				// Fast Voxel Ray Traversal Algorithm (Method 1)
				ray_traversal(raydir,ray_m,obj_cube,pc_localpos[2],obj_distance);
#endif //RT
#ifndef RT
				// Line algorithm (Method 2)
				line_drawing(pc_position,obj_distance,obj_cube,pc_localpos[2]);
#endif //RT
			}
		}

#ifdef GL
		//---------------------------
		//****3D OpenGL Visualization
		//---------------------------
		//opengl camera view
		if (viewpoint == true) {
			fov = 75;
			glm::vec3 front;
			front.x = -cos(camera_global_pose[3]) * sin(-camera_global_pose[4]);
			front.y = sin(camera_global_pose[3]);
			front.z = -cos(camera_global_pose[3]) * cos(-camera_global_pose[4]);
			cameraFront = glm::normalize(front);
			cameraPos = glm::vec3(camera_pixel_pose[0] / unit_length_x - map_shift[0] / block_unit_m,
				-camera_pixel_pose[1] / unit_length_y + map_shift[1] / block_unit_m,
				-camera_pixel_pose[2] / unit_length_x + map_shift[2] / block_unit_m + 0.1);
			cameraPos -= front;
		}

		// traj VAO
		unsigned int trajVAO, trajVBO;
		glGenVertexArrays(1, &trajVAO);
		glGenBuffers(1, &trajVBO);
		glBindVertexArray(trajVAO);
		glBindBuffer(GL_ARRAY_BUFFER, trajVBO);
		glBufferData(GL_ARRAY_BUFFER, scale_trajectory.size() * sizeof(float), &scale_trajectory[0], GL_DYNAMIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		glBindVertexArray(0);

		//Plane
		const int gridlines = 2*grid_x+2;
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

			planeVertices[6 * (k + 1) + 0] = grid_x;
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
			planeVertices[6 * (k + 1) + 2 + 6 * (gridlines)] = -grid_x;
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
		glClearColor(0.3f, 0.5f, 0.6f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
		glLineWidth(5.0f);
		glm::mat4 axis = glm::mat4(1.0f);
		glBindVertexArray(axisVAO);
		planeshader.setMat4("model", axis);
		glDrawArrays(GL_LINES, 0, 6);
		glBindVertexArray(0);

		// render traj------------traj_render();
		glLineWidth(2.0f);
		glm::mat4 gltraj = glm::mat4(1.0f);
		glBindVertexArray(trajVAO);
		planeshader.setMat4("model",gltraj);
		glDrawArrays(GL_LINE_STRIP,0,scale_trajectory.size()/6);

		// render fov-------------fov_render();
		glLineWidth(2.0f);
		glm::mat4 glfov = glm::mat4(1.0f);
		glBindVertexArray(fovVAO);
		glfov = glm::translate(glfov, glm::vec3(camera_pixel_pose[0] / unit_length_x - map_shift[0] / block_unit_m, 
			-camera_pixel_pose[1]/ unit_length_x  + map_shift[1] / block_unit_m, -camera_pixel_pose[2] / unit_length_x + map_shift[2] / block_unit_m));
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

		// material propertiesspecific_row
		lightingShader.setVec3("material.specular", 0.5f, 0.5f, 0.5f);
		lightingShader.setFloat("material.shininess", 8.0f);

		lightingShader.setMat4("projection", projection);
		lightingShader.setMat4("view", view);

		// render boxes
		glBindVertexArray(cubeVAO);
		for (int j = 0; j < grid_y; j++) {
			for (int k = 0; k < grid_z; k++) {
				for (int i = 0; i < grid_x; i++) {
					if (initial_voxel_map[i][j][k] == 1) {
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
#if 0	// show free cells as well
						else if (voxel_map_logodd[i][j][k] <= 30) {
							lightingShader.setVec4("Color", 1.0f - voxel_map_logodd[i][j][k] / 255,
								1.0f - voxel_map_logodd[i][j][k] / 255, voxel_map_logodd[i][j][k] / 255,
								(voxel_map_logodd[i][j][k] / 255 + 0.2) / 1.2);
							//lightingShader.setVec4("Color", 0.0f, 0.0f, 1.0f, 1.0f);
							glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
							model = glm::translate(model, glm::vec3((i + 0.5), -(j + 0.5), -(k + 0.5)));
							//glm::vec3 scale = { probability, probability, probability};
							//model = glm::scale(model, scale);
							lightingShader.setMat4("model", model);
							glDrawArrays(GL_LINE_STRIP_ADJACENCY, 0, 36);
						}
#endif
					}
				}
			}
		}
		glBindVertexArray(0);

		// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
		// -------------------------------------------------------------------------------
		glfwSwapBuffers(window);
		glfwPollEvents();
		
#endif //GL

#ifdef CV
		//---------------------------
		//****2D OpenCV Visualization
		//---------------------------

		for (int k = 0; k < grid_z; k++) {
			for (int i = 0; i < grid_x; i++) {
				if (initial_voxel_map[i][specific_row][k] == 1) {
					rectangle(grid2dmap, Point(i * unit_length_x, k * unit_length_z), Point((i + 1) * unit_length_x, (k + 1) * unit_length_z),
						Scalar(voxel_map_logodd[i][specific_row][k], 255 - voxel_map_logodd[i][specific_row][k], 255 - voxel_map_logodd[i][specific_row][k]), -1);	//BGR
				}
			}
		}

		f1[2] = mapscale_x;
		f1[0] = tan(half_FOVxz)*mapscale_x;
		f1[1] = tan(half_FOVyz)*mapscale_x;
		// FOV conversion
		float f1L[3], f1R[3], f2L[3], f2R[3];
		float f1Rc[3] = { 0,0,0 };
		float f1Lc[3] = { 0,0,0 };
		float f2Rc[3] = { 0,0,0 };
		float f2Lc[3] = { 0,0,0 };
		//f1[2] = (unit_length_z / block_unit_m) * sqrt(pow(max_distance, 2) / (pow(tan(half_FOVxz), 2) + pow(tan(half_FOVyz), 2) +1 ));
		//f1[0] = tan(half_FOVxz)*f1[2];
		//f1[1] = tan(half_FOVyz)*f1[2];

		//fov position (voxel frame)
		//Right(+-+)
		f1R[0] = f1[0];
		f1R[1] = -f1[1];
		f1R[2] = f1[2];
		//Left(--+)
		f1L[0] = -f1[0];
		f1L[1] = -f1[1];
		f1L[2] = f1[2];

		// f1' = Cc/c' * f1
		for (int l = 0; l < 3; l++) {
			for (int m = 0; m < 3; m++) {
				f1Rc[l] += inv_C[l][m] * f1R[m];
				f1Lc[l] += inv_C[l][m] * f1L[m];
			}
		}

		for (int i = 0; i < 3; i++) {
			f1Rc[i] += camera_pixel_pose[i];
			f1Lc[i] += camera_pixel_pose[i];
		}

		/* Draw the FOV(x-z plane) */
		Point FOV_R(f1Rc[0], f1Rc[2]);	//12m is the max range
		Point FOV_L(f1Lc[0], f1Lc[2]);	//12m is the max range
		line(grid2dmap, Point(camera_pixel_pose[0], camera_pixel_pose[2]), FOV_R, Scalar(0, 255, 0), 2);
		line(grid2dmap, Point(camera_pixel_pose[0], camera_pixel_pose[2]), FOV_L, Scalar(0, 0, 255), 2);
		circle(grid2dmap, Point(camera_pixel_pose[0], camera_pixel_pose[2]), 3, Scalar(0, 0, 0), 2);

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat image(Size(width, height), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);	//Depth image
		//Mat image(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);	//RGB Image

		//flip img
		flip(grid2dmap, grid2dmap, 0);

		//cvInputProcess();
		int key = waitKeyEx(1);
		//std::cout << "Y=" << key << std::endl;
		if (key != -1) {
			std::cout << "Key: " << key << std::endl;
			switch (key)
			{
			case 119: specific_row--; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//up
			case 2490368: specific_row--; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//up_win
			case 115: specific_row++; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//down
			case 2621440: specific_row++; /*std::cout << "Y=" << specific_row << std::endl;*/ break;	//down_win
			case 109:
				FILE *fp;
				fp = fopen(map_dir, "w");
				fprintf(fp, "");
				for (int k = 0; k < grid_z; k++) {
					for (int j = 0; j < grid_y; j++) {
						for (int i = 0; i < grid_x; i++) {
							fprintf(fp, "%i,", voxel_map_logodd[i][j][k]);
							
						}
					}
				}
				printf("m pressed, Map saved!\n");
				break;
			}
		}
		
		//time elapse
		clock_t t2 = clock();
		double duration = t2 - t1;
		t1 = t2;
		//double duration = t_get_depth2 - t_get_depth1;
		//std::cout << "\r" << "time spend: " << std::setprecision(3) << std::fixed << duration / CLOCKS_PER_SEC << "  " << key << std::flush;
		std::string fps = std::to_string(1 / (duration / CLOCKS_PER_SEC));
		std::string layer_row = std::to_string(specific_row);
		//std::cout << "\r" << "time spend: " << duration/ CLOCKS_PER_SEC << std::endl;
		putText(grid2dmap, std::string("fps: " + fps), Point(10, 20), 0, 0.6, Scalar(0, 0, 255), 2);
		putText(grid2dmap, std::string("Unit:" + b_unit + "(m/square)"), Point(10, 40), 0, 0.6, Scalar(0, 0, 255), 2);
		putText(grid2dmap, std::string("Layer: " + layer_row), Point(10, 60), 0, 0.6, Scalar(0, 0, 255), 2);

		// Update the stream window with new data
		//imshow(window_name, image);
		//show the voxel map image
		imshow(window_name2, grid2dmap);
		setMouseCallback(window_name2,CVonMouse);

    	imshow(window_name, image);
#endif	//CV

		// Save Current Position as Trajectory
		std::string msg = std::to_string(time_stmp) + ","\
						+ std::to_string(camera_global_pose[0]) + ","\
						+ std::to_string(camera_global_pose[1]) + ","\
						+ std::to_string(camera_global_pose[2]) + ","\
						+std::to_string(q.w) + "," + std::to_string(q.x) + ","\
						+ std::to_string(q.y) + "," + std::to_string(q.z);
		fileID << msg<<std::endl;
#ifdef TimeCost
		time_point<system_clock,microseconds> tt2 = time_point_cast<microseconds>(system_clock::now());
		auto mapping_duration = duration_cast<microseconds>(tt2 - tt1);
		tt1 = tt2;
		std::cout << "Time taken by function: "<< mapping_duration.count() << " microseconds" << std::endl;

		// Save mapping time cost
		std::string msg2 = std::to_string(mapping_duration.count());
		fileID2 << msg2 <<std::endl;
#endif //TimeCost
	}
	pipelines[0].stop();
	pipelines[1].stop();
	printf("RealSense pipelines stopped!\n");

	printf("Trajectory saved!\nProgram closing...\n");
	fileID.close();
	if(fileID.is_open())
        fileID.close();
#ifdef TimeCost
	fileID2.close();
	if(fileID2.is_open())
        fileID2.close();
#endif //TimeCost

#ifdef SOCK
	// close thread
	mapper_status = false;
	t_send.join();
	// end socket
	data2GS.End();
#endif

#ifdef GL
	// optional: de-allocate all resources once they've outlived their purpose:
	// ------------------------------------------------------------------------gl_clean();
	glDeleteVertexArrays(1, &cubeVAO);
	glDeleteBuffers(1, &VBO);

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();
#endif
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

#ifdef __linux__
void init_keyboard()
{
	tcgetattr(0, &initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag &= ~ICANON;
	new_settings.c_lflag &= ~ECHO;
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
	tcsetattr(0, TCSANOW, &initial_settings);
}

int _kbhit()
{
	unsigned char ch;
	int nread;

	if (peek_character != -1) return 1;
	new_settings.c_cc[VMIN] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
	nread = read(0, &ch, 1);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_settings);
	//printf("nread = %i\n",nread);
	if (nread == 1)
	{
		printf("linux kb hit, nread == %i\n", nread);
		peek_character = ch;
		return 1;
	}
	//printf("linux kb hit, nread != 1");
	return 0;
}

int readch()
{
	char ch;

	if (peek_character != -1)
	{
		ch = peek_character;
		peek_character = -1;
		return ch;
	}
	read(0, &ch, 1);
	return ch;
}
#endif


float4 float4::operator*(float t)
{
	return { w * t, x * t, y * t, z * t };
}

float4 float4::operator-(float t)
{
	return { w - t, x - t, y - t, z - t };
}

void float4::operator*=(float t)
{
	w = w * t;
	x = x * t;
	y = y * t;
	z = z * t;
}

void float4::operator=(const float4 &other)
{
	w = other.w;
	x = other.x;
	y = other.y;
	z = other.z;
}

void float4::add(float t1, float t2, float t3, float t4)
{
	w += t1;
	x += t2;
	y += t3;
	z += t4;
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
	//float camera_global_pose[3] = atan2(2 * (q.y*q.z - q.w * q.x),	\\
											q.z ^ 2 + q.w ^ 2 - q.x ^ 2 - q.y ^ 2);	//atan2(Cq[2][1],Cq[2][2])
	//float camera_global_pose[4] = asin(2 * (q.x*q.z + q.w * q.y));										//asin(Cq[2][0])
	//float camera_global_pose[5] = atan2(2.*(q.w*q.z - q.x * q.y),		\\
											(q.z ^ 2 - q.w ^ 2 - q.x ^ 2 + q.y ^ 2));	//atan2(-Cq[1][0],-Cq[0][0])
	//2-3-1
	//float camera_global_pose[3] = atan2(2 * (q.w * q.z - q.y * q.x),	\\
											q.z ^ 2 - q.w ^ 2 + q.x ^ 2 - q.y ^ 2);	//atan2(-Cq[1][0],-Cq[1][1])
	//float camera_global_pose[4] = asin(2 * (q.w * q.x + q.y * q.z));									//asin(Cq[1][2])
	//float camera_global_pose[5] = atan2(2*(q.x * q.z - q.w * q.y), 	\\
											(q.z^2 + q.w ^ 2 - q.x ^ 2 - q.y ^ 2));	//atan2(Cq[0][2],Cq[2][2])
	return angle;
}

void calc_transform(rs2_pose& pose_data, float mat[16])
{
	auto q = pose_data.rotation;
	auto t = pose_data.translation;
	// Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns)
	mat[0] = -(1 - 2 * q.y*q.y - 2 * q.z*q.z); mat[4] = 2 * q.x*q.y - 2 * q.z*q.w;     mat[8] = -(2 * q.x*q.z + 2 * q.y*q.w);      mat[12] = t.x;
	mat[1] = -(2 * q.x*q.y + 2 * q.z*q.w);     mat[5] = 1 - 2 * q.x*q.x - 2 * q.z*q.z; mat[9] = -(2 * q.y*q.z - 2 * q.x*q.w);      mat[13] = t.y;
	mat[2] = -(2 * q.x*q.z - 2 * q.y*q.w);     mat[6] = 2 * q.y*q.z + 2 * q.x*q.w;     mat[10] = -(1 - 2 * q.x*q.x - 2 * q.y*q.y); mat[14] = t.z;
	mat[3] = 0.0f;                             mat[7] = 0.0f;                          mat[11] = 0.0f;                             mat[15] = 1.0f;
}

void ray_traversal(int raydir[], float ray_m[], int dist[], float z_m, float obj_distance)
{
	int cam_cube[3];
	cam_cube[0] = camera_pixel_pose[0]/unit_length_x;
	cam_cube[1] = camera_pixel_pose[1]/unit_length_y;
	cam_cube[2] = camera_pixel_pose[2]/unit_length_z;

	// ray: start from camera position
	int ray_cube[3];
	ray_cube[0] = cam_cube[0];
	ray_cube[1] = cam_cube[1];
	ray_cube[2] = cam_cube[2];
	float r_length = 0;

	int cb[2];
	float tmax[3] = {0};
	float delta[3] = {0};
	int stepX, outX = 0;
	int stepY, outY = 0;
	int stepZ, outZ = 0;

	if (raydir[0] > 0)
	{
		stepX = 1;
		outX = grid_x;
		cb[0] = ray_cube[0] + 1;
	}
	else 
	{
		stepX = -1;
		outX = -1;
		cb[0] = ray_cube[0];
	}
	if (raydir[1] > 0.0f)
	{
		stepY = 1;
		outY = grid_y;
		cb[1] = ray_cube[1] + 1; 
	}
	else 
	{
		stepY = -1;
		outY = -1;
		cb[1] = ray_cube[1];
	}
	if (raydir[2] > 0.0f)
	{
		stepZ = 1;
		outZ = grid_z;
		cb[2] = ray_cube[2] + 1;
	}
	else 
	{
		stepZ = -1;
		outZ = -1;
		cb[2] = ray_cube[2];
	}

	if (raydir[0] != 0)
	{
		tmax[0] = (cb[0]*unit_length_x - camera_pixel_pose[0]) * ray_m[0]; 
		delta[0] = unit_length_x * stepX * ray_m[0];
	}
	else tmax[0] = 1000000;
	if (raydir[1] != 0)
	{
		tmax[1] = (cb[1]*unit_length_y - camera_pixel_pose[1]) * ray_m[1]; 
		delta[1] = unit_length_y * stepY * ray_m[1];
	}
	else tmax[1] = 1000000;
	if (raydir[2] != 0)
	{
		tmax[2] = (cb[2]*unit_length_z - camera_pixel_pose[2]) * ray_m[2];
		delta[2] = unit_length_z * stepZ * ray_m[2];
	}
	else tmax[2] = 1000000;
	// ray tracing loop
	while(ray_cube[0] != dist[0] && ray_cube[1] != dist[1] && ray_cube[2] != dist[2])
	{
		if (tmax[0] < tmax[1])
		{
			if (tmax[0] < tmax[2])
			{
				ray_cube[0] = ray_cube[0] + stepX;
				if (ray_cube[0] == outX) return;
				tmax[0] += delta[0];
				r_length = tmax[0];
			}
			else
			{
				ray_cube[2] = ray_cube[2] + stepZ;
				if (ray_cube[2] == outZ) return;
				tmax[2] += delta[2];
				r_length = tmax[2];
			}
		}
		else
		{
			if (tmax[1] < tmax[2])
			{
				ray_cube[1] = ray_cube[1] + stepY;
				if (ray_cube[1] == outY) return;
				tmax[1] += delta[1];
				r_length = tmax[1];
			}
			else
			{
				ray_cube[2] = ray_cube[2] + stepZ;
				if (ray_cube[2] == outZ) return;
				tmax[2] += delta[2];
				r_length = tmax[2];
			}
		}
		//float r_length = sqrt(abs(ray_cube[0] - cam_cube[0])*abs(ray_cube[0] - cam_cube[0])+
		//					  abs(ray_cube[1] - cam_cube[1])*abs(ray_cube[1] - cam_cube[1])+
		//					  abs(ray_cube[2] - cam_cube[2])*abs(ray_cube[2] - cam_cube[2]));
		if(r_length < obj_distance){render_vmap(false, ray_cube, r_length,z_m, obj_distance);}
		else{return;}
	}
}

void line_drawing(float pc_position[], float ray_length, int dist[], float z_m)
{
	double c_m_xz = 1;
	double c_m_zy = 1;
	double c_m_xy = 1;
	bool angle45_xz = true;
	bool angle45_zy = true;
	bool angle45_xy = true;
	int r[3];						//ray 
	float r_length;					//ray length
	float r_unit_length;			//unit ray length
	int length_factor;				//use to extend r_length
	float dx, dy, dz;				//unit ray x - vector

	//Find max pc_position axes
	double max = 0;
	int c_index = 0;
	for (int n = 0; n < 3; ++n) {
		if (abs(pc_position[n]) > max) {
			max = abs(pc_position[n]);
			c_index = n;
		}
	}

	//Make sure that the ray direction is correct
	int unit_length_dir = unit_length_x;
	if (pc_position[c_index] < 0)
		unit_length_dir *= -1;
	int ray_case = 0;
	if (pc_position[0] == 0 && pc_position[1] == 0 && pc_position[2] != 0) {			//case1 (0,0,1)
		dx = 0;
		dy = 0;
		dz = unit_length_dir;
		ray_case = 1;
	}
	else if (pc_position[0] != 0 && pc_position[1] == 0 && pc_position[2] == 0) {		//case2 (1,0,0)
		dx = unit_length_dir;
		dy = 0;
		dz = 0;
		ray_case = 2;
	}
	else if (pc_position[0] == 0 && pc_position[1] != 0 && pc_position[2] == 0) {		//case3 (0,1,0)
		dx = 0;
		dy = unit_length_dir;
		dz = 0;
		ray_case = 3;
	}
	else if (pc_position[0] == 0 && pc_position[1] != 0 && pc_position[2] != 0) {		//case4 (0,1,1)
		c_m_zy = pc_position[1] / pc_position[2];
		if (c_index == 1)
		{
			dx = 0;
			dy = unit_length_dir;
			dz = dy / c_m_zy;
		}
		else if (c_index == 2)
		{
			dz = unit_length_dir;
			dx = 0;
			dy = dz * c_m_zy;
		}
		ray_case = 4;
	}
	else if (pc_position[0] != 0 && pc_position[1] == 0 && pc_position[2] != 0) {		//case5 (1,0,1)
		c_m_xz = pc_position[2] / pc_position[0];
		if (c_index == 0) {
			dx = unit_length_dir;
			dz = dx * c_m_xz;
			dy = 0;
		}
		else if (c_index == 2)
		{
			dz = unit_length_dir;
			dx = dz / c_m_xz;
			dy = 0;
		}
		ray_case = 5;
	}
	else if (pc_position[0] != 0 && pc_position[1] != 0 && pc_position[2] == 0) {		//case6 (1,1,0)
		c_m_xy = pc_position[1] / pc_position[0];
		if (c_index == 0) {
			dx = unit_length_dir;
			dz = 0;
			dy = dx * c_m_xy;
		}
		else if (c_index == 1) {
			dy = unit_length_dir;
			dz = 0;
			dx = dy / c_m_xy;
		}
		ray_case = 6;
	}
	else																	//Normal Cases(1,1,1)
	{
		c_m_xz = pc_position[2] / pc_position[0];		//x-z plane, m = dz/dx;
		c_m_zy = pc_position[1] / pc_position[2];		//z-y plane,
		//c_m_xy = pc_position[1] / pc_position[0];		//x-z plane

		/* Check if angle > 45 */
		//x-z plane
		if (abs(c_m_xz) >= 1)	angle45_xz = true;
		else if (abs(c_m_xz) < 1)	angle45_xz = false;

		//z-y plane
		if (abs(c_m_zy) >= 1)	angle45_zy = true;
		if (abs(c_m_zy) < 1)	angle45_zy = false;

		//x-y plane
		//if (abs(c_m_xy) >= 1)	angle45_xy = true;
		//if (abs(c_m_xy) < 1)	angle45_xy = false;
		//if (abs(c_m_xy) < 1)	angle45_xy = false;

		//Assign Maximum axis as unit_length
		if (c_index == 0) {
			dx = unit_length_dir;
			dz = dx * c_m_xz;
			dy = dz * c_m_zy;
		}
		else if (c_index == 1) {
			dy = unit_length_dir;
			dz = dy / c_m_zy;
			dx = dz / c_m_xz;
		}
		else if (c_index == 2) {
			dz = unit_length_dir;
			dx = dz / c_m_xz;
			dy = dz * c_m_zy;
		}
	}

	//compare unit ray vector length and original obj distance
	r_unit_length = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
	length_factor = ray_length / r_unit_length;

	/* Free Cell */
	for (int k = 0; k < length_factor; k++) {
		r_length = r_unit_length * k;
		//voxel map origin(pixel)
		r[0] = camera_pixel_pose[0] + dx * k;
		r[1] = camera_pixel_pose[1] + dy * k;
		r[2] = camera_pixel_pose[2] + dz * k;
		//std::cout << "r[0]:" << r[0] << "  r[1]:" << r[1] << "  r[2]:" << r[2] << std::endl;
		//*3D* find parameter of cube
		int ray_cube[3];
		ray_cube[0] = r[0] / unit_length_x;
		ray_cube[1] = r[1] / unit_length_y;
		ray_cube[2] = r[2] / unit_length_z;

		//check if cell is beyond boundary (for fix boundary only)
		if (ray_cube[0] >= grid_x || ray_cube[1] >= grid_y || ray_cube[2] >= grid_z || ray_cube[0] < 0 || ray_cube[1] < 0 || ray_cube[2] < 0
			|| ray_cube[0] - 1 < 0 || ray_cube[1] - 1 < 0 || ray_cube[2] - 1 < 0)
			continue;

		// Keep ray away from obstacle cells
		if(ray_cube[0] == dist[0] && ray_cube[1] == dist[1] && ray_cube[2] == dist[2])
			continue;

		//free cell mapping!!
		render_vmap(false, ray_cube, r_length, z_m, ray_length);
	}
}

void render_vmap(bool type, int cube[], float r_distance, float z_m, float obj_distance)
{
	float DW;			// distance weighting
	float logodds;	// depend on pdf
	float P_occ;
	float k = 0.07;			// inverse sensor model weighting
	
	r_distance /= mapscale_x;	//pixel->cube->meter
	obj_distance /= mapscale_x;
	
	if(r_distance > max_distance)
		r_distance = max_distance;
	else if(r_distance < min_distance)
		r_distance = min_distance;

	DW = r_distance/pixel_density;	// Distance Weighting

	float delta_z = z_m * z_m * sensor_para;
	float sigma = delta_z * r_distance / z_m;
	float Ps;

	// Inverse sensor model
	if (r_distance <= obj_distance){
		P_occ = p_min;
		Ps = P_occ + (k/sigma/sqrt(2*PI)+0.5-P_occ)*exp_fast(-0.5*(r_distance-obj_distance)*(r_distance-obj_distance)/(sigma*sigma));
	}
	else{
		P_occ = 0.5;
		Ps = P_occ;
	}
	
	if (Ps >= 1){
		Ps = 0.99999;
	}
	logodds = log(Ps/(1-Ps));
	
	//std::lock_guard<std::mutex> mlock(mutex_t_m2g);
	mutex_t_m2g.lock();
	// true = occupied cell
	if (voxel_map_logodd[cube[0]][cube[1]][cube[2]] < 255 && type == true) {
		if(ceil(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds) >= 255){
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] = 255;
		}
		else{
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] =
				ceil(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds);
		}

		initial_voxel_map[cube[0]][cube[1]][cube[2]] = 1;
		voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] = voxel_map_logodd[cube[0]][cube[1]][cube[2]];	//set this value as old data in next loop 
	}

	//false = free cell
	else if (voxel_map_logodd[cube[0]][cube[1]][cube[2]] > 0 && type == false) {
		if(floor(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds) <= 0){
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] = 0;
		}
		else{
			voxel_map_logodd[cube[0]][cube[1]][cube[2]] =
				floor(voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] + DW*logodds);
		}

		initial_voxel_map[cube[0]][cube[1]][cube[2]] = 1;
		voxel_map_logodd_old[cube[0]][cube[1]][cube[2]] = voxel_map_logodd[cube[0]][cube[1]][cube[2]];	//set this value as old data in next loop 
	}
	mutex_t_m2g.unlock();
}

#ifdef Map_Expand
void shift_map(int axis)	// shift: 1 meter unit = unit_length_x = 5 cells; x:0, y:1, z:2
{
	int shift_unit = mps / block_unit_m;
	switch (axis)
	{
	case 0:		//x-axis
		// For Lower Limit
		if(shift_unit > 0){	//camera.y < 4, shift = +1, camera.y = camera.y + shift, v[i][j][k] = v[i][j-5][k]
			printf("shift_unit(x) > 0");
			// Shift Maps
			for (int i = grid_x - 1; i > shift_unit - 1; i--) {	//i = 99~5
				for (int j = 0; j < grid_y; j++) {	// j = 29~5
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i - shift_unit][j][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i - shift_unit][j][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i - shift_unit][j][k];
					}
				}
			}		
			// Initialize new voxels
			for (int i = shift_unit - 1; i >= 0; i--) {	//i = 4~0
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
					}
				}
			}

		}
		// For Upper Limit
		else	// shift_unit <= 0
		{
			printf("shift_unit(x) < 0");
			// For Lower Limit
			// Shift Maps
			for (int i = 0; i < grid_x - abs(shift_unit); i++) {	//i = 0~94
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i - shift_unit][j][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i - shift_unit][j][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i - shift_unit][j][k];
					}
				}
			}
			// Initialize new voxels
			for (int i = grid_x - abs(shift_unit); i < grid_x; i++) {	// i = 95~99
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
					}
				}
			}
		}
		break;
	case 1:		//y-axis
		// For Lower Limit
		if(shift_unit > 0){	//camera.y < 4, shift = +1, camera.y = camera.y + shift, v[i][j][k] = v[i][j-5][k]
			printf("shift_unit(y) > 0");
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = grid_y - 1; j > shift_unit - 1; j--) {	// j = 29~5
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j - shift_unit][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j - shift_unit][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j - shift_unit][k];
					}
				}
			}		
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = shift_unit - 1; j >= 0; j--) {	// j = 0~4
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
					}
				}
			}
		}

		// For Upper Limit
		else	// shift_unit <= 0
		{
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y - abs(shift_unit); j++) {	// j = 0~24
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j - shift_unit][k];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j - shift_unit][k];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j - shift_unit][k];
					}
				}
			}
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = grid_y - abs(shift_unit); j < grid_y; j++) {	// j = 25~29
					for (int k = 0; k < grid_z; k++) {
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
					}
				}
			}
		}	
		break;
	case 2:		//z-axis
		// For Lower Limit
		if(shift_unit > 0){	//camera.y < 4, shift = +1, camera.y = camera.y + shift, v[i][j][k] = v[i][j-5][k]
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = grid_z - 1; k > shift_unit - 1; k--) {	//k = 99~5
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j][k - shift_unit];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j][k - shift_unit];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j][k - shift_unit];
					}
				}
			}		
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = shift_unit - 1; k >= 0; k--) {	//k = 4~0
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
					}
				}
			}

		}

		// For Upper Limit
		else	// shift_unit <= 0
		{
			// Shift Maps
			for (int i = 0; i < grid_x; i++) {
				for (int j = 0; j < grid_y; j++) {
					for (int k = 0; k < grid_z - abs(shift_unit); k++) {	//k = 0~94
						voxel_map_logodd[i][j][k] = voxel_map_logodd[i][j][k - shift_unit];
						voxel_map_logodd_old[i][j][k] = voxel_map_logodd_old[i][j][k - shift_unit];
						initial_voxel_map[i][j][k] = initial_voxel_map[i][j][k - shift_unit];
					}
				}
			}
			// Initialize new voxels
			for (int i = 0; i < grid_x; i++) {
				for (int j =0; j < grid_y; j++) {
					for (int k = grid_z - abs(shift_unit); k < grid_z; k++) {	//k = 95~99
						voxel_map_logodd[i][j][k] = 127;
						voxel_map_logodd_old[i][j][k] = 127;
						initial_voxel_map[i][j][k] = 0;
					}
				}
			}
		}
		break;	
	}
}
#endif

#ifdef GL
// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	// moving mode
	float cameraSpeed = 1.0f; // camera moving speed
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
		fp = fopen(map_dir, "w");
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
	// Change Texture
	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
		unsigned int diffuseMap = loadTexture("../res/creeper.jpg");
	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
		unsigned int diffuseMap = loadTexture("../res/wall.jpg");
	lightPos = cameraPos + glm::vec3(2.0f, 0.0f, -0.5f);
}

void mouse_button_callback(GLFWwindow * window, int button, int action, int mode)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		//std::cout << "Left Key Pressed!" << std::endl;
		rotation_started = true;
	}
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

	return textureID;
}
#endif

static void CVonMouse(int event,int x,int y,int,void*)
{
    if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
     }
}
#ifdef VICON
void startViconThread(){
	std::string parentdir = "../../res/testdata/vicon"; //03112020_boxPos_01.txt
	std::string filepath;
	time_t now = time(NULL);
	struct tm *info;
	info = localtime(&now);
	char date_time[30];
	strftime(date_time,30,"%y-%m-%d_%H-%M-%S",info);
	std::string filetime(date_time);
	filepath +=  parentdir + "/" + filetime;
	std::string filename = filetime;
	std::cout << filename << std::endl;

	std::ofstream fileID;
	fileID.open(filepath, std::ofstream::out | std::ofstream::app);
	if(!fileID.is_open()){
		printf("\n\nCould not open file");
		exit;
	}

    using namespace ViconDataStreamSDK::CPP;
    using namespace std::chrono;
    // *********************************
    //      Make a new Vicon client  *
    //      Put IP address here      *
    // *********************************
    std::string HostName = "192.168.1.100:801";
    Client MyClient;

    // Connect to a server
    printf("\n\nConnecting to Vicon at %s ...", HostName.c_str());
    while( !MyClient.IsConnected().Connected )
    {
        // Direct connection

        bool ok = false;
        ok =( MyClient.Connect( HostName ).Result == Result::Success );
        if(!ok)
        {
            printf("\n\nWarning - connection failed");
        }

        printf(".");
        sleep(1);
    }
    printf("\n\nConnected to Vicon at %s\n", HostName.c_str());
    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableMarkerRayData();
    MyClient.EnableDeviceData();
    MyClient.EnableDebugData();

    // Set the streaming mode
    MyClient.SetStreamMode( StreamMode::ClientPull );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward,
                             Direction::Right,
                             Direction::Down ); // Z-down (FRD - coordinate)

    while(mapper_status){
        // Get a frame
    	output_stream << "Waiting for new frame...";
		// Get Vicon data
        while(MyClient.GetFrame().Result != Result::Success){
            sleep(1);
            printf(".\n");
        }

        time_point<system_clock,microseconds> ts = time_point_cast<microseconds>(system_clock::now());
        uint64_t time_stmp = ts.time_since_epoch().count();
        unsigned short SubjectCount = MyClient.GetSubjectCount().SubjectCount;
		output_stream << "Subjects (" << SubjectCount << "):" << std::endl;
		double BoxPos[7], BoxAtt[4], H450Pos[7], H450Att[4];
    	std::string sjname;
        for(unsigned short SubIndex = 0; SubIndex < SubjectCount; ++SubIndex){
            // Get subject name
            std::string SubjectName = MyClient.GetSubjectName(SubIndex).SubjectName;

            // Get segment name
            int i = 0;
            unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
            for (unsigned short SegIndex = 0; SegIndex < SegmentCount; ++SegIndex){
                std::string SegmentName = MyClient.GetSegmentName( SubjectName, SegIndex ).SegmentName;
                if ((SubjectName == "VoxelBox") || (SegmentName == "VoxelBox")){
                    Output_GetSegmentGlobalTranslation BoxXYZ = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
                    Output_GetSegmentGlobalRotationQuaternion BoxQuat = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
                    BoxPos[0] = BoxXYZ.Translation[0]; BoxPos[1] = BoxXYZ.Translation[1]; BoxPos[2] = BoxXYZ.Translation[2];
                    BoxPos[3] = BoxQuat.Rotation[ 0 ];
					BoxPos[4] = BoxQuat.Rotation[ 1 ];
					BoxPos[5] = BoxQuat.Rotation[ 2 ];
					BoxPos[7] = BoxQuat.Rotation[ 3 ];
					//double ViconQuat[4] = {BoxQuat.Rotation[3], BoxQuat.Rotation[0], BoxQuat.Rotation[1], BoxQuat.Rotation[2]};
                    //double q_vb2b[4] = {0., 1., 0., 0.};
                    //quat_mult(ViconQuat, q_vb2b, BoxAtt);
                }else if ((SubjectName == "H450") || (SegmentName == "H450")){
                    Output_GetSegmentGlobalTranslation H450XYZ = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
                    Output_GetSegmentGlobalRotationQuaternion H450Quat = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
                    H450Pos[0] = H450XYZ.Translation[0]; H450Pos[1] = H450XYZ.Translation[1]; H450Pos[2] = H450XYZ.Translation[2];
                    H450Pos[3] = H450Quat.Rotation[ 0 ];
					H450Pos[4] = H450Quat.Rotation[ 1 ];
					H450Pos[5] = H450Quat.Rotation[ 2 ];
					H450Pos[7] = H450Quat.Rotation[ 3 ];
					//double ViconQuat[4] = {H450Quat.Rotation[3], H450Quat.Rotation[0], H450Quat.Rotation[1], H450Quat.Rotation[2]};
                    //double q_vb2b[4] = {0., 1., 0., 0.};
                    //quat_mult(ViconQuat, q_vb2b, H450Att);
                }
            }
        }
        std::string msg = std::to_string(time_stmp) + "," + std::to_string(BoxPos[0]) + "," + std::to_string(BoxPos[1]) + "," + std::to_string(BoxPos[2]) + "," +
						std::to_string(BoxPos[3]) + "," + std::to_string(BoxPos[4]) + "," + std::to_string(BoxPos[5]) + "," + std::to_string(BoxPos[6]) + "," +
						//std::to_string(BoxAtt[0]) + "," + std::to_string(BoxAtt[1]) + "," + std::to_string(BoxAtt[2]) + "," + std::to_string(BoxAtt[3]) + "," +
						std::to_string(H450Pos[0]) + "," + std::to_string(H450Pos[1]) + "," + std::to_string(H450Pos[2]) + "," +
						std::to_string(H450Pos[3]) + "," + std::to_string(H450Pos[4]) + "," + std::to_string(H450Pos[5]) + "," + std::to_string(H450Pos[6]) + "," +
						//std::to_string(H450Att[0]) + "," + std::to_string(H450Att[1]) + "," + std::to_string(H450Att[2]) + "," + std::to_string(H450Att[3]);
						std::to_string(camera_state[0]) + "," + std::to_string(camera_state[1]) + "," + std::to_string(camera_state[2]) + "," +
						std::to_string(q.w) + "," + std::to_string(q.x) + "," + std::to_string(q.y) + "," + std::to_string(q.z) + "," + 
						std::to_string(map_shift[0]) + "," + std::to_string(map_shift[1]) + "," + std::to_string(map_shift[2]);

		std::cout << msg << std::endl;
		fileID << msg;
		fileID.close();
    }

    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    // Disconnect and dispose
    printf("\n\nDisconnecting ...\n");
    MyClient.Disconnect();
}

void quat_mult( double a[4], double b[4], double c[4])
{

    c[0] = b[0]*a[0] - b[1]*a[1] - b[2]*a[2] - b[3]*a[3];
    c[1] = b[0]*a[1] + b[1]*a[0] - b[2]*a[3] + b[3]*a[2];
    c[2] = b[0]*a[2] + b[1]*a[3] + b[2]*a[0] - b[3]*a[1];
    c[3] = b[0]*a[3] - b[1]*a[2] + b[2]*a[1] + b[3]*a[0];

}
#endif //VICON