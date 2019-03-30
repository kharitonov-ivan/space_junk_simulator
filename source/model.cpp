#include "model.h"

void World::World::Save(const std::string& filepath) {
    std::cout << "World::Save not implemented yet\n";
}

void World::World::Load(const std::string& filepath) {
    std::cout << "World::Load not implemented yet\n";
}

void World::World::AddForce(Force& force) {
    forces_.push_back(force);
}

void World::World::AddObject(Object& object) {
    objects_.push_back(object);
}

std::vector<std::vector<World::Object> > World::World::CalculatePositions(size_t stepsNumber) {
    std::vector<std::vector<Object> > positions;
    solver_->Solve(dt_, time_, objects_, forces_, stepsNumber, positions);
    return positions;
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
        for (auto& it = trajectories_.begin(); it != trajectories_.end(); ++it) {
            it->second.insert(it->second.end(), positions[it->first].begin(), positions[it->first].end());
        }

        for (size_t i = 0; i < positions.size(); ++i) {
            objects_[i] = positions[i][positions[i].size() - 1];
        }

        time_ += dt_ * std::min(maxSteps_, stepsNumber - curStep);
    }
}

void World::Physics::GravityForce::GetAcceleration(double x, double y, double z,
                                              double vx, double vy, double vz,
                                              double *ax, double *ay, double *az)  {
    double r = sqrt(x * x + y * y + z * z);
    double k = -G * M / r / r / r;
    *ax = x * k;
    *ay = y * k;
    *az = z * k;
};

void World::Force::GetAcceleration(double x, double y, double z, 
                                   double vx, double vy, double vz,
                                   double *ax, double *ay, double *az) {
    *ax = 0.0;
    *ay = 0.0;
    *az = 0.0;
    std::cout << "Warning: GetAcceleration called from base class 'Force'!\n";
}

void World::World::DumpTrajectories(std::string& outputFile) {
    std::ofstream out(outputFile);
    for (auto it = trajectories_.begin(); it != trajectories_.end(); ++it) {
        out << it->first << '\n';
        for (auto& obj : it->second) {
            out << obj.x << ' ' << obj.y << ' ' << obj.z << ' ' << obj.vx << ' ' << obj.vy << ' ' << obj.vz << '\n';
        }
    }
}
