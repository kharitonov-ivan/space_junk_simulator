#include <thrust/transform.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/functional.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include "model.cuh"
#include "gpu/gpu_solver.cuh"

void ReadConfig(std::istream& in,
                std::vector<World::Object>& objects,
                std::vector<size_t>& logTrajectories,
                std::vector<std::string>& forces,
                size_t& maxSteps,
                size_t& steps,
                double& dt,
                double& time,
                std::string& trajectoriesOutputPath,
                std::string& collisionsOutputPath,
                std::string& collisionOption) {
    in >> time >> dt >> maxSteps >> steps;
    size_t objectsNum, logTrajectoriesNum, forcesNum;
    in >> objectsNum;
    for (size_t i = 0; i < objectsNum; ++i) {
        double x, y, z, vx, vy, vz, size;
        in >> x >> y >> z >> vx >> vy >> vz >> size;
        objects.push_back({ x, y, z, vx, vy, vz, size });
    }

    in >> logTrajectoriesNum;
    for (size_t i = 0; i < logTrajectoriesNum; ++i) {
        size_t id;
        in >> id;
        logTrajectories.push_back(id);
    }

    in >> forcesNum;
    for (size_t i = 0; i < forcesNum; ++i) {
        std::string force;
        in >> force;
        forces.push_back(force);
    }

    in >> trajectoriesOutputPath;
    in >> collisionsOutputPath;
    in >> collisionOption;
}
int main(int argc, char *argv[]) {
    using namespace World::Physics;

    std::string trajectoriesOutputPath, collisionsOutputPath, collisionOption;
    std::vector<World::Object> objects;
    std::vector<std::string> forces;
    std::vector<size_t> logTrajectories;
    double time, dt;
    size_t maxSteps, steps;
    auto in = std::ifstream(argv[1], std::ios_base::in);
    ReadConfig(in, objects, logTrajectories,
        forces,
        maxSteps,
        steps,
        dt,
        time,
        trajectoriesOutputPath,
        collisionsOutputPath,
        collisionOption
    );

    World::Solver *generalSolver = new GPUSolver::Solver();

    World::World world(dt, time, objects, forces, generalSolver, maxSteps, logTrajectories, collisionOption);

    world.Simulate(steps);
    world.DumpTrajectories(trajectoriesOutputPath);
    world.DumpCollisions(collisionsOutputPath);
    delete (generalSolver);
}
