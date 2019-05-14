#include "model.cuh"

void World::World::Save(const std::string& filepath) {
    std::cout << "World::Save not implemented yet\n";
}

void World::World::Load(const std::string& filepath) {
    std::cout << "World::Load not implemented yet\n";
}

void World::World::AddObject(Object& object) {
    objects_.push_back(object);
}

std::vector<std::vector<World::Object> > World::World::CalculatePositions(size_t stepsNumber) {
    std::vector<std::vector<Object> > positions;
    solver_->Solve(dt_, time_, objects_, forces_, stepsNumber, positions);
    return positions;
}

void CheckCollisions(std::vector<std::vector<World::Object> >& positions, std::vector<std::tuple<double, size_t, size_t> >& collisions, double time, double dt) {
    for (size_t i = 0; i < positions[0].size(); ++i) {
        for (size_t j = 0; j < positions.size(); ++j) {
            for (size_t k = j + 1; k < positions.size(); ++k) {
                double dist = pow(positions[j][i].x - positions[k][i].x, 2) + pow(positions[j][i].y - positions[k][i].y, 2) + pow(positions[j][i].z - positions[k][i].z, 2);
                if (dist < pow(positions[j][i].size - positions[k][i].size, 2)) {
                    collisions.push_back(std::make_tuple(time + dt * i, j, k));
                }
            }
        }
    }
}

void World::World::Simulate(size_t stepsNumber) {
    size_t curStep = 0;
    while (curStep < stepsNumber) {
        //std::cout << curStep << '\n';
        std::vector<std::vector<Object> > positions = CalculatePositions(
            std::min(maxSteps_, stepsNumber - curStep)
        );
        curStep += maxSteps_;
        //check collisions and log trajectories here
//        if (collisionOption_ != "disabled") {
//            CheckCollisions(positions, collisions_, time_, dt_);
//        }
        for (auto it = trajectories_.begin(); it != trajectories_.end(); ++it) {
            it->second.insert(it->second.end(), positions[it->first].begin(), positions[it->first].end());
        }

        for (size_t i = 0; i < positions.size(); ++i) {
            objects_[i] = positions[i][positions[i].size() - 1];
        }

        time_ += dt_ * std::min(maxSteps_, stepsNumber - curStep);
    }
}

void World::World::DumpTrajectories(std::string& outputFile) {
    std::ofstream out(outputFile);
    for (auto it = trajectories_.begin(); it != trajectories_.end(); ++it) {
        out << it->first << '\n';
        for (auto& obj : it->second) {
            out << std::setprecision(20) << obj.x << ' ' << obj.y << ' ' << obj.z << ' ' << obj.vx << ' ' << obj.vy << ' ' << obj.vz << '\n';
        }
    }
}

void World::World::DumpCollisions(std::string& outputFile) {
    std::ofstream out(outputFile);
    for (const auto collision : collisions_) {
        out << std::setprecision(20) << std::get<0>(collision) << ' ' << std::get<1>(collision) << ' ' << std::get<2>(collision) << '\n';
    }
}