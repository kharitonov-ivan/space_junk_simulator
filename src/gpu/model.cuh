#pragma once
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <unordered_map>

namespace World {
     struct Object {   
        void PrintObject() {
            std::cout << "Position: " << x << ' ' << y << ' ' << z << '\n';
            std::cout << "Velocity: " << vx << ' ' << vy << ' ' << vz << '\n';
            std::cout << "Size: " << size << '\n';
        }

        float R() {
            return sqrt(x * x + y * y + z * z);
        }

        float V() {
            return sqrt(vx * vx + vy * vy + vz * vz);
        }

        float x, y, z, vx, vy, vz, size;
    };


    class Force {
      public:
        virtual void GetAcceleration (float x, float y, float z,
                                      float vx, float vy, float vz,
                                      float *ax, float *ay, float *az);
    };


    namespace Physics {
        const float EARTH_MASS = 5.972e24;
        const float EARTH_RADIUS = 6.371e6;
        const float GRAVITY_CONSTANT = 6.67408e-11;

        //aliases

        const float M = EARTH_MASS;
        const float G = GRAVITY_CONSTANT;
        const float R = EARTH_RADIUS;

        class GravityForce : public Force {
            public:
                virtual void GetAcceleration(float x, float y, float z,
                    float vx, float vy, float vz,
                    float *ax, float *ay, float *az);
        };
    };

    class Solver {
      public:
        virtual void Solve(float dt,
                           float time,
                           std::vector<Object>& objects,
                           std::vector<Force>& forces,
                           size_t stepsNumber,
                           std::vector<std::vector<Object> >& positions) = 0;
    };


    class World {
      public:
        World(float dt,
              float time,
              std::vector<Object>& objects,
              std::vector<Force>& forces,
              Solver* solver,
              size_t maxSteps,
              std::vector<size_t>& logTrajectories) :
                time_(time), objects_(objects), forces_(forces), 
                solver_(solver), maxSteps_(maxSteps), dt_(dt) {
            for (const auto& id : logTrajectories) {
                trajectories_[id] = std::vector<Object>();
            }
        };

        void Save(const std::string& filepath);

        void Load(const std::string& filepath);

        void Simulate(size_t stepsNumber);

        void AddForce(Force& force);

        void AddObject(Object& object);

        void PrintObject(size_t id) {
            //std::cout << "Ojbect id: " << id << '\n';
            objects_[id].PrintObject();
        }

        Object& GetObject(size_t id) {
            return objects_[id];
        }

        void DumpTrajectories(std::string& outputFile);

      private:
        std::vector<std::vector<Object> > CalculatePositions(size_t stepsNumber);
        std::vector<Object>& objects_;
        std::vector<Force>& forces_;
        std::unordered_map<size_t, std::vector<Object> > trajectories_;
        Solver* solver_;
        size_t maxSteps_;
        float time_;
        float dt_;
        std::ofstream output_;
    };
}

