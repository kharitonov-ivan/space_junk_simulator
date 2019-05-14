#pragma once
#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <unordered_map>
#include <iomanip>

namespace World {
    struct Object {
        void PrintObject() {
            std::cout << "Position: " << x << ' ' << y << ' ' << z << '\n';
            std::cout << "Velocity: " << vx << ' ' << vy << ' ' << vz << '\n';
            std::cout << "Size: " << size << '\n';
        }

        double R() {
            return sqrt(x * x + y * y + z * z);
        }

        double V() {
            return sqrt(vx * vx + vy * vy + vz * vz);
        }

        double x, y, z, vx, vy, vz, size;
    };


    namespace Physics {
        const double EARTH_MASS = 5.972e24;
        const double EARTH_RADIUS = 6.371e6;
        const double GRAVITY_CONSTANT = 6.67408e-11;

        //aliases

        const double M = EARTH_MASS;
        const double G = GRAVITY_CONSTANT;
        const double R = EARTH_RADIUS;
    };

    class Solver {
    public:
        virtual void Solve(double dt,
            double time,
            std::vector<Object>& objects,
            std::vector<std::string>& forces,
            size_t stepsNumber,
            std::vector<std::vector<Object> >& positions) = 0;
    };


    class World {
    public:
        World(double dt,
            double time,
            std::vector<Object>& objects,
            std::vector<std::string>& forces,
            Solver* solver,
            size_t maxSteps,
            std::vector<size_t>& logTrajectories,
            std::string collisionOption) :
            time_(time), objects_(objects), forces_(forces),
            solver_(solver), maxSteps_(maxSteps), dt_(dt), collisionOption_(collisionOption) {
            for (const auto& id : logTrajectories) {
                trajectories_[id] = std::vector<Object>();
            }
        };

        void Save(const std::string& filepath);

        void Load(const std::string& filepath);

        void Simulate(size_t stepsNumber);

        void AddObject(Object& object);

        void PrintObject(size_t id) {
            //std::cout << "Ojbect id: " << id << '\n';
            objects_[id].PrintObject();
        }

        Object& GetObject(size_t id) {
            return objects_[id];
        }

        void DumpTrajectories(std::string& outputFile);

        void DumpCollisions(std::string& outputFile);

    private:
        std::vector<std::vector<Object> > CalculatePositions(size_t stepsNumber);
        std::vector<Object>& objects_;
        std::vector<std::string>& forces_;
        std::unordered_map<size_t, std::vector<Object> > trajectories_;
        std::vector<std::tuple<double, size_t, size_t> > collisions_;
        std::string collisionOption_;
        Solver* solver_;
        size_t maxSteps_;
        double time_;
        double dt_;
        std::ofstream output_;
    };
}

