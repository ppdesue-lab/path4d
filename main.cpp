#include <iostream>
#include <map>
#include <vector>
#include <limits>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "meshslicer.h"
#include "FPSCamera.h"
#include "Pipe.h"


void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

// Camera
FPSCamera camera(glm::vec3(0.0f, 0.0f, 30.0f));
float lastX = 400, lastY = 300;
bool firstMouse = true;

// Timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}


bool testMDS()
{
    MDS mds1(0.0f, 30.0f);
    MDS mds2(20.0f, 50.0f);
    if (!mds1.Intersects(mds2))
        return false;
	auto range1 = mds1.IntersectsRange(mds2);

    //test cross x-axis
    MDS mds3(350.0f, 10.0f);
    MDS mds4(5.0f, 20.0f);
    if (!mds3.Intersects(mds4))
        return false;
	auto range2 = mds3.IntersectsRange(mds4);
    
    //test one cross xaxis and other near axis
	MDS mds5(350.0f, 10.0f);
	MDS mds6(330.0f, 355.0f);
    if (!mds5.Intersects(mds6))
        return false;
    auto range3 = mds5.IntersectsRange(mds6);
    

    return true;
}
int main()
{
    //assert(testMDS());
    // Load model and make slices
    auto positions_all = LoadModelAndMakeSlices(
        "Data/15252_Key_Ring_Wall_Mount_Hand_v1.obj",
        //"Data/test.obj",
        glm::vec3(0, 1, 0), 0.1f);

    // Draw and save canvas
    Pipe pipe;
    std::map<int, MDSContours> contours;
    for (auto contour : positions_all)
    {
        auto mds_contour = computeMDSContours(contour.second);
        contours.insert({ contour.first, mds_contour });
    }
    pipe.test_idx =60;// 76;//40
    pipe.initTool(0.4f, 10.0f);
	auto start_time = clock();
    pipe.CalMDSForEachSlice(contours);
    //pipe.drawAndSaveCanvas(contours, pipe.test_idx);
    //system("sliced_model.png");
    //return 0;
    pipe.GenerateContoursFromMDS(contours);
    pipe.connectLayerContoursWithSafeHeight(10.0f);

    auto end_time = clock();
    std::cout << "calculation time: " << (end_time - start_time) / (float)CLOCKS_PER_SEC << " seconds." << std::endl;
    auto savedContours = pipe.SavedContours;
    pipe.exportToGCode("output.gcode");
    //ouput savedContours' size info
    //for (const auto& pair : savedContours)
    //{
    //    std::cout << "Slice " << pair.first << ": " << pair.second.size() << " contours saved." << std::endl;
    //}

    bool hide_rawmodel = false;

    // Initialize GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Set GLFW options
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a GLFW window
    GLFWwindow* window = glfwCreateWindow(800, 600, "Mesh Slicer", NULL, NULL);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // Tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Load GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    
    // Prepare vertex data for original slices
    std::vector<float> vertices;
    for (const auto& pair : positions_all)
    {
        const auto& contours = pair.second;
        for (auto contour : contours)
        {
            auto& positions = contour;
            uint32_t count = positions.size();
            for (size_t i = 0; i < count; i ++)
            {
                vertices.push_back(positions[i].x);
                vertices.push_back(positions[i].y);
                vertices.push_back(positions[i].z);
                vertices.push_back(positions[(i + 1)% count].x);
                vertices.push_back(positions[(i + 1) % count].y);
                vertices.push_back(positions[(i + 1) % count].z);
            }
        }
    }

    //print vertex count
    // Create VAO and VBO for original slices
    unsigned int VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Prepare vertex data for saved contours
    std::vector<float> savedVertices;
    for (const auto& pair : savedContours)
    {
        float height_value = pair.first * 0.1f;
        const auto& contourList = pair.second;
        for (const auto& contour : contourList)
        {
            uint32_t count = contour.size();
            for (size_t i = 0; i < count; i++)
            {
                savedVertices.push_back(contour[i].Position.x);
                savedVertices.push_back(contour[i].Position.y);
                savedVertices.push_back(contour[i].Position.z);
                savedVertices.push_back(contour[(i + 1) % count].Position.x);
                savedVertices.push_back(contour[(i + 1) % count].Position.y);
                savedVertices.push_back(contour[(i + 1) % count].Position.z);

                //print xyz
                //std::cout << "Vertex: (" << contour[i].x << ", " << contour[i].y << ", " << contour[i].z << ")" << std::endl;
                //print line end xyz
                //std::cout << "Line end: (" << contour[(i + 1) % count].x << ", " << contour[(i + 1) % count].y << ", " << contour[(i + 1) % count].z << ")" << std::endl;
            }
        }
    }
    //print vertex count
    std::cout << "Saved contour vertices count: " << savedVertices.size() / 3 << std::endl;
    // Create VAO and VBO for saved contours
    unsigned int savedVAO, savedVBO;
    glGenVertexArrays(1, &savedVAO);
    glGenBuffers(1, &savedVBO);

    glBindVertexArray(savedVAO);
    glBindBuffer(GL_ARRAY_BUFFER, savedVBO);
    glBufferData(GL_ARRAY_BUFFER, savedVertices.size() * sizeof(float), savedVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Prepare vertex data for normal vectors
    std::vector<float> normalVertices;
    float normalLength = 0.5f; // Length of normal vectors for visualization

    for (const auto& pair : savedContours)
    {
        float height_value = pair.first * 0.1f;
        const auto& contourList = pair.second;
        for (const auto& contour : contourList)
        {
            for (size_t i = 0; i < contour.size(); i++)
            {
                const auto& cp = contour[i];
                // Start point of normal
                normalVertices.push_back(cp.Position.x);
                normalVertices.push_back(cp.Position.y);
                normalVertices.push_back(cp.Position.z);
                // End point of normal (start + normal * length)
                normalVertices.push_back(cp.Position.x + cp.Normal.x * normalLength);
                normalVertices.push_back(cp.Position.y + cp.Normal.y * normalLength);
                normalVertices.push_back(cp.Position.z + cp.Normal.z * normalLength);
            }
        }
    }
    std::cout << "Normal vectors count: " << normalVertices.size() / 6 << std::endl;
    // Create VAO and VBO for normal vectors
    unsigned int normalVAO, normalVBO;
    glGenVertexArrays(1, &normalVAO);
    glGenBuffers(1, &normalVBO);

    glBindVertexArray(normalVAO);
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glBufferData(GL_ARRAY_BUFFER, normalVertices.size() * sizeof(float), normalVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Shader sources
    const char* vertexShaderSource = "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "uniform mat4 model;\n"
        "uniform mat4 view;\n"
        "uniform mat4 projection;\n"
        "void main()\n"
        "{\n"
        "   gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
        "}\0";

    const char* fragmentShaderSource = "#version 330 core\n"
        "uniform vec3 color;\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(color, 1.0f);\n"
        "}\n\0";

    // Compile vertex shader
    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // Compile fragment shader
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // Link shaders
    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        // Per-frame time logic
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Input
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(0, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(1, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(2, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(3, deltaTime);

        if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
            hide_rawmodel = !hide_rawmodel;

        // Render
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Activate shader
        glUseProgram(shaderProgram);

        // Pass transformation matrices to the shader
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)800 / (float)600, 0.1f, 100.0f);

        unsigned int modelLoc = glGetUniformLocation(shaderProgram, "model");
        unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
        unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
        unsigned int colorLoc = glGetUniformLocation(shaderProgram, "color");

        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

        // Draw original slices in white
        if (!hide_rawmodel)
        {
            glUniform3f(colorLoc, 1.0f, 1.0f, 1.0f);
            glBindVertexArray(VAO);
            glDrawArrays(GL_LINES, 0, vertices.size() / 3);
        }
        // Draw saved contours in yellow
        glUniform3f(colorLoc, 1.0f, 1.0f, 0.0f);
        glBindVertexArray(savedVAO);
        glDrawArrays(GL_LINES, 0, savedVertices.size() / 3);

        // Draw normal vectors in cyan
        glUniform3f(colorLoc, 0.0f, 1.0f, 1.0f);
        glBindVertexArray(normalVAO);
        glDrawArrays(GL_LINES, 0, normalVertices.size() / 3);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &savedVAO);
    glDeleteBuffers(1, &savedVBO);
    glDeleteVertexArrays(1, &normalVAO);
    glDeleteBuffers(1, &normalVBO);
    glDeleteProgram(shaderProgram);

    glfwTerminate();
    return 0;
}