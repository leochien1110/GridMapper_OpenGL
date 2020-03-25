#ifndef DATA_H
#define DATA_H

#define grid_x 100
#define grid_y 30
#define grid_z 100

// int
extern unsigned int unit_length;
extern unsigned int specific_row;

// float

// double

// char

// array
extern float camera_global_pose[12];
extern float camera_scaled_pose[3];

extern unsigned char voxel_map[grid_x][grid_y][grid_z];
extern unsigned char initial_voxel_map[grid_x][grid_y][grid_z];

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

    vehicleState *getCamPos() {return _VState;};

};

#endif //DATA_H