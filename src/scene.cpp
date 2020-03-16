#define STB_IMAGE_IMPLEMENTATION 
#include "scene.h"

glm::vec3 Scene::cameraPos = glm::vec3(0.0f, -5.0f, 10.0f);
glm::vec3 Scene::cameraFront = glm::vec3(0.0f, -0.5f, -1.0f);

bool Scene::firstMouse = true;
float Scene::yaw = -90.0f; // yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float Scene::pitch = -30.0f;
float Scene::lastX = 800.0f / 2.0;
float Scene::lastY = 600.0 / 2.0;
float Scene::fov = 45.0f;
bool Scene::rotation_started = false;
bool Scene::viewpoint = true;
int Scene::start_x = 0;
int Scene::start_y = 0;

Scene::Scene(int unit_length_x, float block_unit_m, float half_FOVxz, float half_FOVyz)
{
    // glfw: initialize and configure(version 3.3)
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Voxel 3D", NULL, NULL);
    if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit;
	}

	// Filed of view
	f1_2 = (unit_length_x / block_unit_m) * sqrt(pow(1, 2) / (pow(tan(half_FOVxz), 2) + pow(tan(half_FOVyz), 2) + 1));
	f1_0 = tan(half_FOVxz)*f1_2;
	f1_1 = tan(half_FOVyz)*f1_2;
	float f1[3] = { f1_0, f1_1, f1_2 };

}

Scene::~Scene()
{
	//delete window;
}

void Scene::processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	// moving mode
	float cameraSpeed = 1.0f; // adjust accordingly
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
		unsigned int diffuseMap = loadTexture("../res/creeper.jpg");
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
		unsigned int diffuseMap = loadTexture("../res/wall.jpg");
	lightPos = cameraPos + glm::vec3(2.0f, 0.0f, -0.5f);
}

void Scene::mouse_button_callback(GLFWwindow * window, int button, int action, int mode)
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
			cameraPos = glm::vec3(0.0f, -5.0f, 10.0f);
			fov = 45;
		}
	}
	else if (button == GLFW_RELEASE) {
		//std::cout << "Left Key Released!" << std::endl;
		rotation_started = false;
		firstMouse = true;
	}
}

void Scene::mouse_position_callback(GLFWwindow * window, double xpos, double ypos)
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

void Scene::scroll_callback(GLFWwindow * window, double xoffset, double yoffset)
{
	if (fov >= 1.0f && fov <= 70.0f)
		fov -= yoffset;
	if (fov <= 1.0f)
		fov = 1.0f;
	if (fov >= 70.0f)
		fov = 70.0f;
}

void Scene::mouseLeftPressEvent(GLFWwindow * window, double xpos, double ypos)
{
	rotation_started = true;
	start_x = xpos;
	start_y = ypos;
}

void Scene::mouseLeftReleaseEvent()
{
	rotation_started = false;

}

void Scene::mouseLeftDragEvent(GLFWwindow * window, double xpos, double ypos)
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

void Scene::glInit(GLFWwindow * window)
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

void Scene::glSetup(unsigned int VAO, unsigned int VBO, float *Vertices, unsigned int AttribSize1, unsigned int AttribSize2, unsigned int Stride, unsigned int VertexOffset)
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
void Scene::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int Scene::loadTexture(char const * path)
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