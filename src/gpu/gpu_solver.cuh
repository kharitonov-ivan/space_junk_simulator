#pragma once

#include "model.cuh"
#include <unordered_map>
namespace GPUSolver {
    class Solver : public World::Solver {
    public:
        Solver() {};


        void Solve(float dt,
            float time,
            std::vector<World::Object>& objects,
            std::vector<World::Force>& forces,
            size_t stepsNumber,
            std::vector<std::vector<World::Object> >& positions);
    };
}
