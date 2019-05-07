#include "gpu_solver.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

__host__ __device__
struct GpuObject {
  __host__ __device__ GpuObject(double x, double y, double z, double vx, double vy, double vz, double size) :
      x(x), y(y), z(z), vx(vx), vy(vy), vz(vz), size(size) {};
  double x, y, z, vx, vy, vz, size;
};

void GetAccel(double x, double y, double z,
              double vx, double vy, double vz,
              double *ax, double *ay, double *az) {
  //TODO: calculate for each force
  using namespace World::Physics;
  //std::cout << "Called GetAccel with: " << x << ' ' << y << ' ' << z << ' ' << vx << ' ' << vy << ' ' << vz << '\n';
  const double r = sqrt(x * x + y * y + z * z);
  //std::cout << r << '\n';
  const double k = -G * M / r / r / r;
  //std::cout << k << '\n';
  *ax = k * x;
  *ay = k * y;
  *az = k * z;
};

World::Object GetNextState(World::Object &obj,
                           double dt,
                           double time) {
  double dt2 = dt / 2.0;
  double dt1 = time + dt;
  double dh = time + dt2;
  //std::cout << "times: " << time << ' ' << dt2 << ' ' << dt1 << ' ' << dh << '\n';
  double ax, ay, az;
  GetAccel(obj.x, obj.y, obj.z, obj.vx, obj.vy, obj.vz, &ax, &ay, &az);

  //std::cout << ax << ' ' << ay << ' ' << az << '\n';

  double kx1 = dt2 * ax;
  double ky1 = dt2 * ay;
  double kz1 = dt2 * az;

  double lx1 = dt2 * obj.vx;
  double ly1 = dt2 * obj.vy;
  double lz1 = dt2 * obj.vz;

  //std::cout << kx1 << ' ' << ky1 << ' ' << kz1 << ' ' << lx1 << ' ' << ly1 << ' ' << lz1 << ' ' << dt2 << ' ' << dt << ' ' << time << '\n';
  GetAccel(obj.x + lx1, obj.y + ly1, obj.z + lz1,
           obj.vx + kx1, obj.vy + ky1, obj.vz + kz1,
           &ax, &ay, &az);

  //std::cout << ax << ' ' << ay << ' ' << az << '\n';

  double kx2 = dt2 * ax;
  double ky2 = dt2 * ay;
  double kz2 = dt2 * az;

  double lx2 = dt2 * (obj.vx + kx1);
  double ly2 = dt2 * (obj.vy + ky1);
  double lz2 = dt2 * (obj.vz + kz1);

  GetAccel(obj.x + lx2, obj.y + ly2, obj.z + lz2,
           obj.vx + kx2, obj.vy + ky2, obj.vz + kz2,
           &ax, &ay, &az);

  double kx3 = dt * ax;
  double ky3 = dt * ay;
  double kz3 = dt * az;

  double lx3 = dt * (obj.vx + kx2);
  double ly3 = dt * (obj.vy + ky2);
  double lz3 = dt * (obj.vz + kz2);

  GetAccel(obj.x + lx3, obj.y + ly3, obj.z + lz3,
           obj.vx + kx3, obj.vy + ky3, obj.vz + kz3,
           &ax, &ay, &az);

  double kx4 = dt2 * ax;
  double ky4 = dt2 * ay;
  double kz4 = dt2 * az;

  double lx4 = dt2 * (obj.vx + kx3);
  double ly4 = dt2 * (obj.vy + ky3);
  double lz4 = dt2 * (obj.vz + kz3);

  return World::Object(
      obj.x + (lx1 + 2.0 * lx2 + lx3 + lx4) / 3.0,
      obj.y + (ly1 + 2.0 * ly2 + ly3 + ly4) / 3.0,
      obj.z + (lz1 + 2.0 * lz2 + lz3 + lz4) / 3.0,
      obj.vx + (kx1 + 2.0 * kx2 + kx3 + kx4) / 3.0,
      obj.vy + (ky1 + 2.0 * ky2 + ky3 + ky4) / 3.0,
      obj.vz + (kz1 + 2.0 * kz2 + kz3 + kz4) / 3.0,
      obj.size);
}

void GpuSolve(double dt, double time,
              thrust::host_vector<World::Object> &objects,
              thrust::host_vector<World::Force> &forces,
              size_t stepsNumber,
              std::vector<std::vector<World::Object> > &positions) {

  positions.resize(objects.size());
  //std::cout << "time: " << time << '\n';
  //std::cout << "dt: " << dt << '\n';
  for (size_t id = 0; id < positions.size(); ++id) {
    positions[id].push_back(objects[id]);
    for (size_t step = 0; step < stepsNumber; ++step) {
      // std::cout << id << ' ' << step << '\n';
      positions[id].push_back(
          GetNextState(positions[id][step], dt, time + step * dt)
      );
    }
  }
}

void GPUSolver::Solver::Solve(double dt,
                              double time,
                              std::vector<World::Object> &objects,
                              std::vector<World::Force> &forces,
                              size_t stepsNumber,
                              std::vector<std::vector<World::Object> > &positions) {
  thrust::host_vector<GpuObject> host_objects;


  for (size_t idx = 0; idx < objects.size(); ++idx) {
    std::cout << objects[idx].x << '\n';
    auto gpu_object = GpuObject(objects[idx].x, objects[idx].y, objects[idx].z,
                                objects[idx].vx, objects[idx].vy, objects[idx].vz,
                                objects[idx].size);
    host_objects.push_back(gpu_object);
  }
  thrust::device_vector<GpuObject> device_objects(host_objects.begin(), host_objects.end());
//  thrust::host_vector<World::Force> host_forces(forces.begin(), forces.end());
//  GpuSolve(dt, time, host_objects, host_forces, stepsNumber, positions);
}