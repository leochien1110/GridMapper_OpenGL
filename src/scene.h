#ifndef SCENE_H
#define SCENE_H

// opengl
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>
#include <iostream> 
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>   // Include OpenCV API

//imgui
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include "stb_image.h"
#include "shader_m.h"
#include "data.h"

#define GUI

using namespace cv;
using namespace std;

class Scene
{
public:
    Scene();
    ~Scene();

    //---------------------
    // OpenGL Visualization
    //---------------------
    void glInit(GLFWwindow * window);
    void glSetup(unsigned int VAO, unsigned int VBO, float Vertices[], unsigned int AttribSize1,
        unsigned int AttribSize2, unsigned int Stride, unsigned int VertexOffset);
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    unsigned int loadTexture(char const * path);
    unsigned int matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter);
    void start();
    void update();
    void stop();
    //Input Setup
    void processInput(GLFWwindow *window);
    static void mouse_button_callback(GLFWwindow * window, int button, int action, int mode);
    static void mouse_position_callback(GLFWwindow * window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow * window, double xoffset, double yoffset);
    void mouseLeftPressEvent(GLFWwindow * window, double xpos, double ypos);
    void mouseLeftReleaseEvent();
    void mouseLeftDragEvent(GLFWwindow * window, double xpos, double ypos);

    // Window settings
    static const unsigned int SCR_WIDTH = 1600;
    static const unsigned int SCR_HEIGHT = 900;

    // Camera movement
    static glm::vec3 cameraPos;
    static glm::vec3 cameraFront;
    glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

    // Source light
    glm::vec3 lightPos = glm::vec3(0.0f, 5.0f, 10.0f);

    //
    GLFWwindow* window;

    // Save map

    // ImGUI
    bool show_2d_map;
    bool show_demo_window;
    bool show_another_window;
    ImVec2 window_size_2d;
    ImVec4 clear_color;

private:
    int specific_row = 15;
    int map_shift[3] = {0};
    // texture index for different texture
    unsigned int texture_index = 0;

    // 2D map parameters
    float f1L[3], f1R[3], f2L[3], f2R[3];
    const int v_x = LENGTH * mapscale_x;	//1500
    const int v_y = HEIGHT * mapscale_y;	//900
    const int v_z = WIDTH * mapscale_z;	//1500

    // Streaming
    std::thread streamThread;
    bool scene_stream = false;
    //unsigned char (*map)[30][100];
    //float *camera_pose;
    //float *camera_scaled_pose;
    //int grid_x = 100, grid_y = 30, grid_z = 100;

    // Mouse setting
    static bool firstMouse;
    static float yaw;
    static float pitch;
    static float lastX;
    static float lastY;
    static float fov;
    static bool rotation_started;
    static bool viewpoint;
    static int start_x;
    static int start_y;

    // Filed of view
    //float f1_2;
    //float f1_0;
    //float f1_1;
    //float f1[3];

    //Threading
    std::mutex mutex;

    float vertices[288] = {
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

    float axisVertices[36] = {
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

    float fovVertices[96] = {
        //top-right
          0.0f,    0.0f,   0.0f, 0.5f, 0.1f, 0.3f,
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
         f1[0],   f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
        -f1[0],   f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
        //left
        -f1[0],   f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
        -f1[0],  -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
        //down
        -f1[0],  -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
         f1[0],  -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
        //right
         f1[0],  -f1[1], -f1[2], 0.5f, 0.1f, 0.3f,
         f1[0],   f1[1], -f1[2], 0.5f, 0.1f, 0.3f
    };


};

#endif  //SCENE_H