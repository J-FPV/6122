#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <algorithm>

struct Vec3 
{
    double x, y, z;
};

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " Torus.obj\n";
        return 1;
    }

    const char* filename = argv[1];
    std::ifstream in(filename);
    if (!in)
    {
        std::cerr << "Failed to open OBJ: " << filename << "\n";
        return 1;
    }

    bool hasVertex = false;
    Vec3 bbMin{ std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max() };
    Vec3 bbMax{ std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest() };

    std::string line;
    while (std::getline(in, line))
    {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            double x, y, z;
            iss >> x >> y >> z;
            if (!iss) continue;

            hasVertex = true;
            bbMin.x = std::min(bbMin.x, x);
            bbMin.y = std::min(bbMin.y, y);
            bbMin.z = std::min(bbMin.z, z);

            bbMax.x = std::max(bbMax.x, x);
            bbMax.y = std::max(bbMax.y, y);
            bbMax.z = std::max(bbMax.z, z);
        }
    }

    if (!hasVertex)
    {
        std::cerr << "No vertices found in OBJ.\n";
        return 1;
    }

    double dx = bbMax.x - bbMin.x;
    double dy = bbMax.y - bbMin.y;
    double dz = bbMax.z - bbMin.z;
    double maxDim = std::max({dx, dy, dz});

    if (maxDim <= 0.0)
    {
        std::cerr << "Degenerate bounding box.\n";
        return 1;
    }

    // World units = meters. We want max dimension <= 0.2 m.
    // Use 0.19 for a tiny safety margin.
    double targetSize = 0.19;  // meters
    double scale = targetSize / maxDim;

    // Also compute center in original OBJ space (optional)
    Vec3 center{
        0.5 * (bbMin.x + bbMax.x),
        0.5 * (bbMin.y + bbMax.y),
        0.5 * (bbMin.z + bbMax.z)
    };

    std::cout << "File: " << filename << "\n";
    std::cout << "Scale to fit 0.19m cube: " << scale << "\n";
    std::cout << "Center: ("
              << center.x << ", " << center.y << ", " << center.z << ")\n";

    return 0;
}
