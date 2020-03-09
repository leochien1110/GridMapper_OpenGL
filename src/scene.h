#ifndef SCENE_H
#define SCENE_H

// opengl
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GLFW/glfw3.h>

//#define STB_IMAGE_IMPLEMENTATION    
#include "stb_image.h"
#include "shader_m.h"

class Scene
{
public:
    Scene(int,float,float,float);
    ~Scene();

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

    // Window settings
    const unsigned int SCR_WIDTH = 800;
    const unsigned int SCR_HEIGHT = 600;

    // Camera movement
    glm::vec3 cameraPos = glm::vec3(0.0f, -5.0f, 10.0f);
    glm::vec3 cameraFront = glm::vec3(0.0f, -0.5f, -1.0f);
    glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

    // Source light
    glm::vec3 lightPos = glm::vec3(0.0f, 5.0f, 10.0f);

    //
    GLFWwindow* window;


private:

    // Mouse setting
    bool firstMouse = true;
    float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
    float pitch = -30.0f;
    float lastX = 800.0f / 2.0;
    float lastY = 600.0 / 2.0;
    float fov = 45.0f;
    bool rotation_started = false;
    bool viewpoint = true;
    int start_x = 0;
    int start_y = 0;

    // Filed of view
    float f1_2;
    float f1_0;
    float f1_1;
    float f1[3];

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