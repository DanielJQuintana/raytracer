#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <tuple>
#include <cstring>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

// Callback used to resize the viewport when the user changes the size.
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void getPixels(float* pixels, std::ifstream& framesFile) {
    // std::cout << "Getting pixels..." << std::endl;

    std::string line;
    std::string del = " ";
    int i = 0;

    if (framesFile.is_open()) {
        // std::cout << "File is open..." << std::endl;
        while ( getline (framesFile,line) ) {
            if (line[0] == '-') {
                // std::cout << "Reached image end..." << std::endl;
                return;
            }

            int start, end = -1*del.size();
            do {
                start = end + del.size();
                end = line.find(del, start);
                if (end != -1) {
                    pixels[i] = stof(line.substr(start, end - start));
                    i++;
                }
            } while (end != -1);
        }
    }
}

// Callback used to move the next frame of the robot path.
void pause(GLFWwindow* window, int key, int scancode, int action, int mods) {
    bool* pauseFlag = (bool*)glfwGetWindowUserPointer(window);
    if (key == GLFW_KEY_ENTER && action == GLFW_PRESS) *pauseFlag = !(*pauseFlag);
}

int main(int argc, char **argv) {
    std::ifstream framesFile; 
    framesFile.open(argv[1]);

    if (!framesFile.is_open()) {
        return -1;
    }

    std::string paramString;
    getline(framesFile, paramString);

    size_t delimPos = paramString.find(",");
    int numFrames = stoi(paramString.substr(0, delimPos));
    int viewDimX = stoi(paramString.substr(delimPos + 1, paramString.length() - delimPos + 1));

    glfwInit();

    int viewDimY = viewDimX;
    
    GLFWwindow* window = glfwCreateWindow(viewDimX, viewDimY, "PRM Visualizer", NULL, NULL);

    if (window == NULL)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }  

    glViewport(0, 0, viewDimX, viewDimY);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);  

    float* pixels = (float*)malloc(sizeof(float) * viewDimX * viewDimY * 3);

    int posIdx = 0;
    bool* pauseFlag = (bool*)malloc(sizeof(bool));
    *pauseFlag = false;

    bool display = false;

    glfwSetWindowUserPointer(window, (void*)pauseFlag);
    glfwSetKeyCallback(window, pause);

    const double MIN_ANIM_LENGTH = 5.0;
    const double MAX_FRAME_RATE = 20;

    std::cout << std::to_string(numFrames) << std::endl;

    double anim_length = std::max(MIN_ANIM_LENGTH, static_cast<double>(numFrames) / MAX_FRAME_RATE);
    double frame_rate = static_cast<double>(numFrames) / anim_length;

    // std::cout << anim_length << " " << frame_rate << std::endl;

    double prev_time = 0.0;
    double curr_time = 0.0;

    glfwSetTime(0.0);

    while(!glfwWindowShouldClose(window))
    {
        if ((curr_time - prev_time > 1.0 / frame_rate && !(*pauseFlag)) || curr_time == 0.0) {
            std::cout << "Frame " << posIdx << std::endl;
            getPixels(pixels, framesFile);
            posIdx += 1;
            prev_time = curr_time;
        }

        if (*pauseFlag && !display) {
            display = true;
            std::cout << "Paused" << std::endl;
        }

        if (!(*pauseFlag) && display) {
            display = false;
        }

        if (posIdx >= numFrames) break;

        // display(img, triangles, num_triangles, spheres, num_spheres, c);
        glClearColor(1, 1, 1, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDrawPixels(viewDimX, viewDimY, GL_RGB, GL_FLOAT, pixels);

        glfwSwapBuffers(window);
        glfwPollEvents();

        curr_time = glfwGetTime();
    }
    
    framesFile.close();
    
    glfwTerminate();

    return 0;
}