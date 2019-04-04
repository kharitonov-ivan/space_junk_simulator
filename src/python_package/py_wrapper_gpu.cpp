#include "../model.h"
#include "../cpu/cpu_solver.h"
#include "../../lib/json.hpp"
#include <fstream>

extern "C"
void solve_gpu(const double *x,
               const double *y,
               const double *z,
               const double *vx,
               const double *vy,
               const double *vz,
               double *x_res,
               double *y_res,
               double *z_res,
               double *vx_res,
               double *vy_res,
               double *vz_res,
               size_t simulate_steps,
               float timestep) {
  using namespace World::Physics;
  std::vector<World::Object> objects;
  std::vector<World::Force> forces;

  for (size_t i = 0; i < sizeof(x); ++i) {
    objects.push_back(World::Object(x[i], y[i], z[i],
                                    vx[i], vy[i], vy[i], 1.0));

  }

  World::Physics::GravityForce gravity = World::Physics::GravityForce();
  forces.push_back(gravity);

  std::vector<size_t> log_trajectories;

  World::Solver *generalSolver = new GPUSolver::Solver();
  World::World world(timestep, 0.0, objects, forces, generalSolver, simulate_steps, log_trajectories);

  std::cout << "Model created. Starting simulations\n";

  world.PrintObject(0);
  for (size_t i = 0; i < 1; ++i) {
    world.Simulate(simulate_steps);
    std::cout << "Step " << i << ": R = " << world.GetObject(0).R() << ", V = " << world.GetObject(0).V() << ' '
              << world.GetObject(0).x << ' ' << world.GetObject(0).y << ' ' << world.GetObject(0).z << '\n';
  }

  for (size_t i = 0; i < sizeof(objects); ++i) {
    x_res[i] = world.GetObject(i).x;
    y_res[i] = world.GetObject(i).y;
    z_res[i] = world.GetObject(i).z;
    vx_res[i] = world.GetObject(i).vx;
    vy_res[i] = world.GetObject(i).vy;
    vz_res[i] = world.GetObject(i).vz;
  }

}