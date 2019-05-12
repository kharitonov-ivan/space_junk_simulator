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
//    CheckCollisions(positions, collisions_, time_, dt_);

    for (auto it = trajectories_.begin(); it != trajectories_.end(); ++it) {
      it->second.insert(it->second.end(), positions[it->first].begin(), positions[it->first].end());
    }

    for (size_t i = 0; i < positions.size(); ++i) {
      objects_[i] = positions[i][positions[i].size() - 1];
    }

    time_ += dt_ * std::min(maxSteps_, stepsNumber - curStep);
  }
}

void World::Physics::SimpleGravityForce::GetAcceleration(double x, double y, double z,
                                                         double vx, double vy, double vz,
                                                         double *ax, double *ay, double *az)  {
  double r = sqrt(x * x + y * y + z * z);
  double k = -G * M / r / r / r;
  *ax = x * k;
  *ay = y * k;
  *az = z * k;
};


void World::Physics::HeterogeneousGravityForce::GetAcceleration(double x, double y, double z,
                                                                double vx, double vy, double vz,
                                                                double *ax, double *ay, double *az)  {
  double r = sqrt(x * x + y * y + z * z);
  double pi = 3.14159265359;


  // TODO Gravity latitude correction (WGS - elipsoid) _ WIP
  /*double a = 6378137.0; //WGS84 Semimajor axis
  double f = 1/298.257223563; //WGS84 Flatteing
  double b = a*(1-f); //Semiminor axis
  double g_equator_const = 9.7803453359;
  double g_pole_const = 9.8321849378;

  double z_wgs = sqrt(b*(1-((x*x+y*y)/a*a))); //Solving for the positive z value on the ellipsoid
  double p = sqrt(x*x+y*y); // Distance from the z-axis
  double wgs_lat = atan(z/(p*(1-f)*p*(1-f)))*180/pi; // Solving the latitude value;

  double eccentricity_squared = 1 - (b /a) * (b /a);
  double k_constant = (b * g_pole_const - a * g_equator_const) / a / g_equator_const;
  double g_corr_lat_num = g_equator_const * (1 + k_constant * sin(wgs_lat) * sin(wgs_lat));
  double g_corr_lat_denum = sqrt(1 - eccentricity_squared * sin(wgs_lat)* sin(wgs_lat));
  double g_corr_wgs = g_corr_lat_num/g_corr_lat_denum;*/

  // TODO Simle circle gravity

  double lat = asin(z / r) * 360 / (2 * pi);

  // Gravity latitude correction (non WGS)
  double g_corr_lat = 9.780327 * (1 + 0.0053024 * sin(lat)*sin(lat)- 0.0000058 * sin(2 * lat)* sin(2 * lat));

  // Gravert altitude correction
  double alt = r - R;
  double g_corr_alt = - 3.086e-6 * alt;
  double g_corr = g_corr_lat + g_corr_alt;
  double k = - g_corr/r;

  *ax +=  x * k;
  *ay += y * k;
  *az += z * k;
};

double GetAirDensity(double height) {
  // TODO: Look up table for atmosphere density
  return 0.0;
}

void World::Physics::AirDrag::GetAcceleration(double x, double y, double z,
                                                                double vx, double vy, double vz,
                                                                double *ax, double *ay, double *az)  {
  double r = sqrt(x * x + y * y + z * z);
  double v_norm = vx * vx + vy * vy + vz * vz;
  double v = sqrt(v_norm);


  double air_drag_coeff = 8.e-4;
  double air_drag = air_drag_coeff * GetAirDensity(r) * v_norm;

  double air_drag_ax = - air_drag * vx / v;
  double air_drag_ay = - air_drag * vy / v;
  double air_drag_az = - air_drag * vz / v;

  *ax += air_drag_ax;
  *ay += air_drag_ay;
  *az += air_drag_az;
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
