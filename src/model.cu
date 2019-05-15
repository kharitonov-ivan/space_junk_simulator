#include "model.cuh"
#include <sys/types.h>

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

typedef std::tuple<int, int, int> key_t_;
struct key_hash : public std::unary_function<key_t_, std::size_t>
{
    std::size_t operator()(const key_t_& k) const
    {
        return std::get<0>(k) ^ std::get<1>(k) ^ std::get<2>(k);
    }
};

void CheckCollisionsFast(std::vector<std::vector<World::Object> >& positions, std::vector<std::tuple<double, size_t, size_t> >& collisions, double time, double dt) {
    std::unordered_map<std::tuple<int, int, int>, std::vector<size_t>, key_hash> cubes;
    double size_first = 0.0; 
    double size_second = 0.0;
    for (size_t i = 0; i < positions.size(); ++i) {
        if (positions[i][0].size > size_first) {
            size_second = size_first;
            size_first = positions[i][0].size;
        } else if (positions[i][0].size > size_second) {
            size_second = positions[i][0].size;
        }
    }
    double cube_edge = size_first + size_second;
    for (size_t i = 0; i < positions[0].size(); ++i) {
        cubes.clear();
        for (size_t j = 0; j < positions.size(); ++j) {
            std::tuple<int, int, int> cube = std::make_tuple<int, int, int> (
                trunc(positions[j][i].x / cube_edge),
                trunc(positions[j][i].y / cube_edge),
                trunc(positions[j][i].z / cube_edge)
            );
            cubes[cube].push_back(j);
        }
        for (auto it = cubes.begin(); it != cubes.end(); ++it) {
            auto cur_cube = it->first;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        auto next_cube = std::make_tuple<int, int, int>(
                            std::get<0>(cur_cube) + dx,
                            std::get<1>(cur_cube) + dy,
                            std::get<2>(cur_cube) + dz);
                        auto nextIt = cubes.find(next_cube);
                        if (nextIt != cubes.end()) {
                            for (auto nextObj : nextIt->second) {
                                for (auto curObj : it->second) {
                                    if (nextObj < curObj) {
                                        auto nextCoord = positions[nextObj][i];
                                        auto curCoord = positions[curObj][i];
                                        if (pow(nextCoord.x - curCoord.x, 2) + pow(nextCoord.y - curCoord.y, 2) + pow(nextCoord.z - curCoord.z, 2) <
                                            pow(nextCoord.size + curCoord.size, 2)) 
                                        {
                                            collisions.push_back({ time + dt * i, nextObj, curObj });
                                        }
                                    }
                                }
                            }
                        }
                    }
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

        if (collisionOption_ == "slow") {
            CheckCollisions(positions, collisions_, time_, dt_);
        } else if (collisionOption_ != "disabled") {
            CheckCollisionsFast(positions, collisions_, time_, dt_);
        }

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
    for (auto& collision : collisions_) {
        out << std::setprecision(20) << std::get<0>(collision) << ' ' << std::get<1>(collision) << ' ' << std::get<2>(collision) << '\n';
    }
}