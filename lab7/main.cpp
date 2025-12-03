/*
Author: Shanyu Jiang
Class: ECE6122
Last Date Modified: 02/12/2025
Description:
    Main file for UAV simulation and visualization using OpenGL.
*/

#include "simulation.h"
#include <GL/freeglut.h>
#include <vector>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <cmath>

// Global UAV list
std::vector<std::unique_ptr<sim::UAV>> g_uavs;

// Field texture
GLuint g_fieldTex = 0;

// Field dimensions
const float FIELD_LENGTH = 120.0f; // x
const float FIELD_WIDTH  = 53.3f;  // y

// Simple camera parameters
float g_camDist = 80.0f;

// OBJ model
// ------------------------------------------
constexpr float CHICKEN_SCALE = 0.0273f;

struct ObjVertex {
    float x, y, z;   // position
    float u, v;      // texture coordinates
};

struct ObjModel {
    std::vector<ObjVertex> tris;  // 3*N entries (already triangulated)
    bool loaded = false;
};

ObjModel g_chicken;
GLuint   g_uavTex = 0;            // UAV texture ID

// Minimal OBJ loader
// ------------------------------------------
bool loadOBJ(const char* filename, ObjModel& model)
{
    std::ifstream in(filename);
    if (!in)
    {
        std::printf("Failed to open OBJ: %s\n", filename);
        return false;
    }

    // Temporary arrays for raw OBJ data
    std::vector<control::Vec3> positions;   // v
    struct Vec2 { float u, v; };
    std::vector<Vec2> texcoords;            // vt

    std::vector<ObjVertex> tris;            // final flattened triangles

    std::string line;
    while (std::getline(in, line))
    {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v")
        {
            float x, y, z;
            iss >> x >> y >> z;
            if (!iss) continue;
            positions.emplace_back(x, y, z);
        }
        else if (type == "vt")
        {
            float u, v;
            iss >> u >> v;
            if (!iss) continue;
            texcoords.push_back({u, v});
        }
        else if (type == "f")
        {
            // faces: f v/vt/vn v/vt/vn v/vt/vn ...
            std::string vstr;
            std::vector<unsigned int> posIdx;  // position indices
            std::vector<unsigned int> texIdx;  // texcoord indices (optional)

            while (iss >> vstr) {
                std::istringstream viss(vstr);
                std::string idxStr;

                // vIdx
                std::getline(viss, idxStr, '/');
                if (idxStr.empty()) continue;
                int vIdxSigned = std::stoi(idxStr);

                // vtIdx (may be empty)
                std::string vtStr;
                if (std::getline(viss, vtStr, '/'))
                {
                    // vtStr may be "" if no vt
                }

                int posZero = 0;
                if (vIdxSigned > 0)
                {
                    posZero = vIdxSigned - 1;
                }
                else
                {
                    // negative index: -1 = last, etc.
                    posZero = static_cast<int>(positions.size()) + vIdxSigned;
                }

                if (posZero < 0 || posZero >= static_cast<int>(positions.size()))
                    continue;

                posIdx.push_back(static_cast<unsigned int>(posZero));

                if (!vtStr.empty())
                {
                    int vtSigned = std::stoi(vtStr);
                    int vtZero;
                    if (vtSigned > 0)
                    {
                        vtZero = vtSigned - 1;
                    }
                    else
                    {
                        vtZero = static_cast<int>(texcoords.size()) + vtSigned;
                    }
                    if (vtZero < 0 || vtZero >= static_cast<int>(texcoords.size())) {
                        texIdx.push_back(static_cast<unsigned int>(-1)); // mark invalid
                    }
                    else
                    {
                        texIdx.push_back(static_cast<unsigned int>(vtZero));
                    }
                }
                else
                {
                    texIdx.push_back(static_cast<unsigned int>(-1));
                }
            }

            if (posIdx.size() >= 3)
            {
                // faceIndices and texIdx are aligned by vertex
                for (size_t i = 1; i + 1 < posIdx.size(); ++i) 
                {
                    size_t triIdx[3] = { 0, i, i + 1 };
                    for (int k = 0; k < 3; ++k)
                    {
                        size_t fi = triIdx[k];

                        unsigned int pIndex = posIdx[fi];
                        ObjVertex vtx{};
                        const auto& p = positions[pIndex];
                        vtx.x = static_cast<float>(p.x);
                        vtx.y = static_cast<float>(p.y);
                        vtx.z = static_cast<float>(p.z);

                        if (texIdx.size() == posIdx.size())
                        {
                            unsigned int tIndex = texIdx[fi];
                            if (tIndex != static_cast<unsigned int>(-1) &&
                                tIndex < texcoords.size())
                            {
                                vtx.u = texcoords[tIndex].u;
                                vtx.v = texcoords[tIndex].v;
                            }
                            else
                            {
                                vtx.u = 0.0f;
                                vtx.v = 0.0f;
                            }
                        }
                        else
                        {
                            vtx.u = 0.0f;
                            vtx.v = 0.0f;
                        }

                        tris.push_back(vtx);
                    }
                }
            }
        }
    }

    if (positions.empty() || tris.empty()) {
        std::printf("OBJ has no geometry: %s\n", filename);
        return false;
    }

    model.tris   = std::move(tris);
    model.loaded = true;

    std::printf(
        "Loaded OBJ %s: %zu vertices (flattened tris)\n",
        filename,
        model.tris.size()
    );

    return true;
}

// BMP loader for field texture
// ------------------------------------------
bool loadBMP(const char* filename, GLuint& texId) {
    unsigned char header[54];
    unsigned int dataPos;
    unsigned int width, height;
    unsigned int imageSize;

    FILE* file = fopen(filename, "rb");
    if (!file) {
        printf("Image could not be opened: %s\n", filename);
        return false;
    }

    if (fread(header, 1, 54, file) != 54) {
        printf("Not a correct BMP file (header size): %s\n", filename);
        fclose(file);
        return false;
    }

    if (header[0] != 'B' || header[1] != 'M') {
        printf("Not a correct BMP file (BM signature): %s\n", filename);
        fclose(file);
        return false;
    }

    dataPos   = *(int*)&(header[0x0A]);
    imageSize = *(int*)&(header[0x22]);
    width     = *(int*)&(header[0x12]);
    height    = *(int*)&(header[0x16]);

    if (imageSize == 0) imageSize = width * height * 3;
    if (dataPos   == 0) dataPos   = 54;

    unsigned char* data = (unsigned char*)malloc(imageSize);
    fseek(file, dataPos, SEEK_SET);
    fread(data, 1, imageSize, file);
    fclose(file);

    glGenTextures(1, &texId);
    glBindTexture(GL_TEXTURE_2D, texId);

    // BMPs are usually BGR
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 width, height,
                 0, GL_BGR, GL_UNSIGNED_BYTE, data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    free(data);
    return true;
}

// Convert from field / world units directly to OpenGL space
// ------------------------------------------
void drawField() 
{
    if (g_fieldTex == 0) 
    {
        // fallback to simple green quad if texture failed
        glColor3f(0.1f, 0.5f, 0.1f);
        glBegin(GL_QUADS);
            glVertex3f(-FIELD_LENGTH * 0.5f, -FIELD_WIDTH * 0.5f, 0.0f);
            glVertex3f( FIELD_LENGTH * 0.5f, -FIELD_WIDTH * 0.5f, 0.0f);
            glVertex3f( FIELD_LENGTH * 0.5f,  FIELD_WIDTH * 0.5f, 0.0f);
            glVertex3f(-FIELD_LENGTH * 0.5f,  FIELD_WIDTH * 0.5f, 0.0f);
        glEnd();
        return;
    }

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_fieldTex);

    glColor3f(1.0f, 1.0f, 1.0f);

    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(-FIELD_LENGTH * 0.5f, -FIELD_WIDTH * 0.5f, 0.0f);

        glTexCoord2f(1.0f, 0.0f);
        glVertex3f( FIELD_LENGTH * 0.5f, -FIELD_WIDTH * 0.5f, 0.0f);

        glTexCoord2f(1.0f, 1.0f);
        glVertex3f( FIELD_LENGTH * 0.5f,  FIELD_WIDTH * 0.5f, 0.0f);

        glTexCoord2f(0.0f, 1.0f);
        glVertex3f(-FIELD_LENGTH * 0.5f,  FIELD_WIDTH * 0.5f, 0.0f);
    glEnd();

    glDisable(GL_TEXTURE_2D);
}

// Draw a UAV at given position
// ------------------------------------------
void drawUAV(const control::Vec3& pos)
{
    glPushMatrix();
    glTranslated(pos.x, pos.y, pos.z);

     // Time in seconds
    float t = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
    const float PI = 3.14159265f;

    // Oscillate between 0.5 and 1.0 at 0.5 Hz
    float brightness = 0.75f + 0.25f * std::sin(PI * t);

    if (g_chicken.loaded && g_uavTex != 0)
    {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, g_uavTex);

        // Make texture *multiply* by glColor (for brightness pulsing)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

        // Scale model to fit in 20 cm cube
        glScalef(CHICKEN_SCALE, CHICKEN_SCALE, CHICKEN_SCALE);

        // This color modulates the texture → brightness oscillation
        glColor3f(brightness, brightness, brightness);

        glBegin(GL_TRIANGLES);

        for (const auto& v : g_chicken.tris)
        {
            glTexCoord2f(v.u, v.v);
            glVertex3f(v.x, v.y, v.z);
        }
        glEnd();
    }
    else
    {
        // Fallback: simple sphere with pulsing brightness
        glDisable(GL_TEXTURE_2D);
        glColor3f(brightness, brightness * 0.5f, brightness * 0.5f);
        glutSolidSphere(0.1, 16, 16);
    }

    glPopMatrix();
}

void display() 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Simple camera looking at the origin from above-diagonal
    gluLookAt(g_camDist, g_camDist, g_camDist,
              0.0, 0.0, 20.0,
              0.0, 0.0, 1.0);

    drawField();

    // Draw all UAVs
    for (auto& u : g_uavs) 
    {
        control::Vec3 p = u->getPosition();
        drawUAV(p);
    }

    glutSwapBuffers();
}

void reshape(int w, int h) 
{
    if (h == 0) h = 1;
    float aspect = static_cast<float>(w) / h;

    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, aspect, 1.0, 500.0);
}

void timer(int value) 
{
    // Simulation-side collision handling
    sim::checkAndResolveCollisions(g_uavs, 0.01);

    glutPostRedisplay();
    glutTimerFunc(30, timer, 0); // ~30 ms
}

void initGL() 
{
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.2f, 0.4f, 0.7f, 1.0f);

    // Load field texture
    if (!loadBMP("ff.bmp", g_fieldTex)) 
    {
        printf("Failed to load ff.bmp, using flat green field.\n");
        g_fieldTex = 0;
    }

    // Load UAV texture
    if (!loadBMP("Buzzy_blue.bmp", g_uavTex)) 
    {
        printf("Failed to load uav.bmp, UAVs will be untextured.\n");
        g_uavTex = 0;
    }

    glEnable(GL_TEXTURE_2D);

    // Load chicken UAV model
    if (!loadOBJ("chicken_01.obj", g_chicken))
    {
        printf("Failed to load Pingu_obj.obj, falling back to sphere.\n");
        g_chicken.loaded = false;
    }
}

// Main
// ------------------------------------------
int main(int argc, char** argv) 
{
    // Create 15 UAVs on different start positions
    control::ControlConfig cfg;
    cfg.center = control::Vec3(0, 0, 50);
    cfg.sphereRadius = 10.0;

    // 3 rows × 5 columns = 15
    std::vector<double> xCols = { -46, -24, -2, 20, 44.0 };
    std::vector<double> yRows = { -22.5, 0.0, 22.5 };

    for (double y : yRows) 
    {
        for (double x : xCols) 
        {
            control::Vec3 startPos(x, y, 0.0); // on ground grid points
            g_uavs.emplace_back(std::make_unique<sim::UAV>(startPos, cfg));
        }
    }

    // Start all worker threads
    for (auto& u : g_uavs) 
    {
        u->start();
    }

    // OpenGL setup
    // ------------------------------------------
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(400, 400);
    glutCreateWindow("ECE UAV Sphere Simulation");

    initGL();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutTimerFunc(30, timer, 0);

    glutMainLoop();

    // Cleanup
    for (auto& u : g_uavs) 
    {
        u->stop();
    }

    return 0;
}
