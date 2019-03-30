#pragma once
#include "model.h"

namespace CPUSolver {
    class Solver : public World::Solver {
      public:
        Solver() {};


        void Solve (double dt,
                    double time,
                    std::vector<World::Object>& objects,
                    std::vector<World::Force>& forces,
                    size_t stepsNumber,
                    std::vector<std::vector<World::Object> >& positions);
      private:
        void GetAccel(double x, double y, double z,
                      double vx, double vy, double vz,
                      double *ax, double *ay, double *az,
                      std::vector<World::Force>& forces);

        World::Object GetNextState(World::Object& object, 
                                   std::vector<World::Force>& forces, 
                                   double dt,
                                   double time);
    };
}
