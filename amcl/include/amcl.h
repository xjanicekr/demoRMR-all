#ifndef AMCL_H
#define AMCL_H

#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>
#include <string>
#include <limits>
//#include "amcl_global.h"
struct Particle {
    float x, y, theta; // pose
    float weight;      // weight (lower is better in our error model)
};
struct GridMap {
    int width, height;
    double offsetX;
    double offsetY;
    float resolution; // meters per cell
    std::vector<int> distanceField; // 0 = obstacle, higher = further away

    int index(int x, int y) const {
        return y * width + x;
    }

    int getDistance(int x, int y) const {
        if (x < 0 || y < 0 || x >= width || y >= height) return 255;
        int indx=index(x, y);
        return distanceField[indx]-1; //-- the -1 is here, because i was lazy and the field has obstacle at distance 1
    }
};

//struct LaserMeasurement;
class /*AMCL_EXPORT*/ amcl
{
public:
    amcl();
    std::default_random_engine rng;
    std::normal_distribution<float> noise_translation;//(0.0f, 0.02f);
    std::normal_distribution<float> noise_rotation;//(0.0f, 0.02f);
    void setSTDs(double transStd,double rotStd);
    void worldToGrid(float x, float y, int& gx, int& gy, const GridMap& map);
    float evaluateParticle(const Particle& p, const LaserMeasurement& scan, const GridMap& map);
    std::vector<Particle> resampleParticles(const std::vector<Particle>& particles,const GridMap& map,double newParticlePercentage=10);
    void predictParticles(std::vector<Particle>& particles, float ddist, float dtheta);
    std::vector<Particle> initializeParticles(int numParticles, const GridMap& map);
    void amclStep(std::vector<Particle>& particles, const LaserMeasurement& scan, const GridMap& map,
                  float ddist,  float dtheta);
    void computeDistanceField(GridMap& map);

    Particle getBestParticle(const std::vector<Particle>& particles) {
        int minindex=0;
        for(int i=1;i<particles.size();i++)
        {
            if(particles[i].weight<particles[minindex].weight)
                minindex=i;
        }
        return particles[minindex];
        return *std::min_element(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) {
            return a.weight < b.weight;
        });
    }
    bool loadGridMap(const std::string& filename, GridMap& map) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Cannot open file " << filename << std::endl;
            return false;
        }

        double offsetX, offsetY;
        double resX, resY;
        file >> offsetX >> offsetY >> resX >> resY;

        // Skip newline after header
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std::vector<std::string> lines;
        std::string line;
        while (std::getline(file, line)) {
            if (!line.empty())
                lines.push_back(line);
        }

        map.height = static_cast<int>(lines.size());
        map.width = static_cast<int>(lines.front().size());
        map.offsetX = offsetX;
        map.offsetY = offsetY;
        map.resolution = static_cast<float>(resX); // assuming square cells

        map.distanceField.resize(map.width * map.height);

        for (int y = 0; y < map.height; ++y) {
            const std::string& row = lines[y];
            for (int x = 0; x < map.width; ++x) {
                map.distanceField[map.index(x, y)] = (row[x] == '1') ? 1 : 0;
            }
        }
        computeDistanceField(map);
        return true;
    }
};

#endif // AMCL_H
