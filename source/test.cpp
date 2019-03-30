#include "model.h"
#include "cpu_solver.h"
#include "json.hpp"
#include <fstream>

/* int main() {
    using namespace World::Physics;

    std::vector<World::Object> objects;
    std::vector<World::Force> forces;

    objects.push_back(World::Object(0.0, 1.2 * R, 0.0,
                             sqrt(G * M / (1.2 * R)), 0.0, 0.0, 1.0));


    World::Physics::GravityForce gravity = World::Physics::GravityForce();
    forces.push_back(gravity);

    World::Solver* generalSolver = new CPUSolver::Solver();
    World::World world(1.0, 0.0, objects, forces, generalSolver, 100, std::vector<size_t>());

    std::cout << "Model created. Starting simulations\n";

    world.PrintObject(0);
    for (size_t i = 0; i < 1; ++i) {
        world.Simulate(1);
        std::cout << "Step " << i << ": R = " << world.GetObject(0).R() << ", V = " << world.GetObject(0).V() << ' ' 
                  << world.GetObject(0).x << ' ' << world.GetObject(0).y << ' ' << world.GetObject(0).z << '\n';
    }
}*/
