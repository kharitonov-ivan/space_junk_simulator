#pragma once
#include "../model.h"
#include <unordered_map>

namespace GPUSolver {
    class Solver : public World::Solver {
    public:
        Solver() {};


        void Solve(double dt,
            double time,
            std::vector<World::Object>& objects,
            std::vector<World::Force>& forces,
            size_t stepsNumber,
            std::vector<std::vector<World::Object> >& positions);
    };
}
