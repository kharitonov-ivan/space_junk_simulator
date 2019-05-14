#pragma once
#include <unordered_map>
#include "../model.cuh"

namespace GPUSolver {
    class Solver : public World::Solver {
    public:
        Solver() {};


        void Solve(double dt,
                   double time,
                   std::vector<World::Object>& objects,
                   std::vector<std::string>& forces,
                   size_t stepsNumber,
                   std::vector<std::vector<World::Object> >& positions);
    };
}
