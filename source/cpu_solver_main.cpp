#include "model.h"
#include "cpu_solver.h"
#include "json.hpp"
#include <fstream>

int main(int argc, char* argv[]) {
    using namespace World::Physics;
    using json = nlohmann::json;

    std::ifstream cfgStream(argv[1]);
    json cfg;
    cfgStream >> cfg;

    std::vector<World::Object> objects;
    for (const auto& i : cfg["objects"]) {
        objects.push_back(World::Object(i["x"], i["y"], i["z"],
            i["vx"], i["vy"], i["vz"], i["size"]));
    }

    std::vector<size_t> logTrajectories;
    for (const auto& i : cfg["logObjects"]) {
        logTrajectories.push_back(i);
    }
    World::Solver* generalSolver = new CPUSolver::Solver();

    std::vector<World::Force> forces;
    World::Physics::GravityForce gravity = World::Physics::GravityForce();
    forces.push_back(gravity);

    std::string outputFile = argv[2];

    World::World world((double)cfg["dt"], (double)cfg["time"], objects, forces, generalSolver, (size_t)cfg["maxSteps"], logTrajectories);
    
    world.Simulate((size_t)cfg["steps"]);
    world.DumpTrajectories(outputFile);

    delete(generalSolver);
}
