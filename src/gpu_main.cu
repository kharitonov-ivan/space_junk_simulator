#include <thrust/transform.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/functional.h>
#include <iostream>
#include <iterator>
#include <algorithm>
#include "model.h"
#include "gpu_solver.cuh"

int main() {
     using namespace World::Physics;

     std::vector<World::Object> objects;
     std::vector<World::Force> forces;

     World::Object start;
     start.x = 0.0;
     start.y = 1.2 * R;
     start.z = 0.0;
     start.vx = sqrt(G * M / (1.2 * R));
     start.vy = 0.0;
     start.vz = 0.0;
     start.size = 1.0;
     objects.push_back(start);


     World::Physics::GravityForce gravity = World::Physics::GravityForce();
     forces.push_back(gravity);

     World::Solver* generalSolver = new GPUSolver::Solver();
     World::World world(1.0, 0.0, objects, forces, generalSolver, 100, std::vector<size_t>());

     std::cout << "Model created. Starting simulations\n";

     world.PrintObject(0);
     for (size_t i = 0; i < 100; ++i) {
         world.Simulate(1);
         std::cout << "Step " << i << ": R = " << world.GetObject(0).R() << ", V = " << world.GetObject(0).V() << ' '
                   << world.GetObject(0).x << ' ' << world.GetObject(0).y << ' ' << world.GetObject(0).z << '\n';
     }
}
