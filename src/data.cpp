#include "data.h"
#include <stdio.h>
#include <math.h>

//--------------------
// Voxel Map Parameter
//--------------------
float block_unit_m = 0.2;	//(meter/cell)
unsigned int unit_length = 5;
unsigned int unit_length_x = 5;
unsigned int unit_length_y = 5;
unsigned int unit_length_z = 5;
unsigned int specific_row = 15; //initial visualized layer

//-----------------
// Camera Parameter
//-----------------
// Camera Pose (meter)
float max_distance = 12;
// X,Y,Z,phi,theta,psi,x,y,z,shift(x,y,z)
float camera_pose[12] = { 0 };
float init_camera_global_pos[3] = { 10 , 3 , 10 };	
float camera_scaled_pose[3] = { 0 };
float specific_point[3]; //for FOV
float inv_C[3][3];
float4 q;    // Rotation quaternion

// Camera resolution
const int width = 848;     //1280,848,640,480, 424; check D435 manual
const int height = 480;    //720,480,360,270,240
const int resolution = width * height;
const int framerate = 30;

// Field of View
float FOVxz = 1.487021;;
float FOVyz = 1.01229;;
float half_FOVxz = FOVxz / 2;
float half_FOVyz = FOVyz / 2;
float FOV[2];
float f1_2 = (unit_length_z / block_unit_m) * sqrt(pow(1, 2) / (pow(tan(half_FOVxz), 2) + pow(tan(half_FOVyz), 2) + 1));
float f1_0 = tan(half_FOVxz)*f1_2;
float f1_1 = tan(half_FOVyz)*f1_2;
float f1[3] = { f1_0, f1_1, f1_2 };

// Point cloud
float3 pc_vertices[1000000];
int points_size;

// Voxel Map Data array
unsigned char voxelmap[grid_x][grid_y][grid_z] = { 0 };
unsigned char voxelmap_old[grid_x][grid_y][grid_z] = { 0 };
bool init_voxelmap[grid_x][grid_y][grid_z] = { 0 };

// Loop Status
bool camera_stream;
bool mapper_stream;
bool connect_stream;
bool mapper_status;

Data::Data()
{
    printf("Data called\n");
}

Data::~Data()
{

}

int Data::a = 333;