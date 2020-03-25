#ifndef DATA_H
#define DATA_H

// building dimension
#define LENGTH 20
#define HEIGHT 6
#define WIDTH 20
// voxel grid map dimension
#define grid_x 100  // LENGTH/block_unit_m
#define grid_y 30
#define grid_z 100

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

//--------------------
// Voxel Map Parameter
//--------------------
extern float block_unit_m;	//(meter/cell)
extern unsigned int unit_length;
extern unsigned int unit_length_x;
extern unsigned int unit_length_y;
extern unsigned int unit_length_z;
extern unsigned int specific_row;

//-----------------
// Camera Parameter
//-----------------
// Camera Pose (meter)
extern float max_distance;
extern float init_camera_global_pos[3];	
// X,Y,Z,phi,theta,psi,x,y,z,shift(x,y,z)
extern float camera_pose[12]; 
extern float4 q;    // Rotation quaternion

// Camera resolution
extern const int width;     //1280,848,640,480, 424
extern const int height;    //720,480,360,270,240
extern const int resolution;
extern const int framerate;


// Field of View
extern float FOVxz;
extern float FOVyz;
extern float half_FOVxz;
extern float half_FOVyz;
extern float FOV[2];
extern float f1_2;
extern float f1_0;
extern float f1_1;
extern float f1[3];

// Voxel Map Data array
extern unsigned char voxel_map_logodd[grid_x][grid_y][grid_z];
extern unsigned char voxel_map_logodd_old[grid_x][grid_y][grid_z];
extern bool initial_voxel_map[grid_x][grid_y][grid_z];


struct vehicleState{
    float x;
    float y;
    float z;
    float qw;
    float qx;
    float qy;
    float qz;
    float X;
    float Y;
    float Z;
    float map_shit_x;
    float map_shit_y;
    float map_shit_z;
};

class Data
{
private:
    vehicleState * _VState = nullptr;

public:
    Data();
    ~Data();

    static int a;
    
    vehicleState *getCamPos() {return _VState;};
};

#endif //DATA_H