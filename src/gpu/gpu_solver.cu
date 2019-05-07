#include "gpu_solver.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>


__host__ __device__
void GetAccel(float x, float y, float z,
    float vx, float vy, float vz,
    float *ax, float *ay, float *az) {
    //TODO: calculate for each force

    const float EARTH_MASS = 5.972e24;
    const float EARTH_RADIUS = 6.371e6;
    const float GRAVITY_CONSTANT = 6.67408e-11;

    //aliases

    const float M = EARTH_MASS;
    const float G = GRAVITY_CONSTANT;
    const float R = EARTH_RADIUS;
    //std::cout << "Called GetAccel with: " << x << ' ' << y << ' ' << z << ' ' << vx << ' ' << vy << ' ' << vz << '\n';
    const float r = sqrt(x * x + y * y + z * z);
    //std::cout << r << '\n';
    const float k = -G * M / r / r / r;
    //std::cout << k << '\n';
    *ax = k * x;
    *ay = k * y;
    *az = k * z;
};

__host__ __device__
World::Object GetNextState(const 
    World::Object& obj,
    float dt,
    float time) {
    float dt2 = dt / 2.0;
    float dt1 = time + dt;
    float dh = time + dt2;
    //std::cout << "times: " << time << ' ' << dt2 << ' ' << dt1 << ' ' << dh << '\n';
    float ax, ay, az;
    GetAccel(obj.x, obj.y, obj.z, obj.vx, obj.vy, obj.vz, &ax, &ay, &az);

    //std::cout << ax << ' ' << ay << ' ' << az << '\n';

    float kx1 = dt2 * ax;
    float ky1 = dt2 * ay;
    float kz1 = dt2 * az;

    float lx1 = dt2 * obj.vx;
    float ly1 = dt2 * obj.vy;
    float lz1 = dt2 * obj.vz;

    //std::cout << kx1 << ' ' << ky1 << ' ' << kz1 << ' ' << lx1 << ' ' << ly1 << ' ' << lz1 << ' ' << dt2 << ' ' << dt << ' ' << time << '\n';
    GetAccel(obj.x + lx1, obj.y + ly1, obj.z + lz1,
        obj.vx + kx1, obj.vy + ky1, obj.vz + kz1,
        &ax, &ay, &az);

    //std::cout << ax << ' ' << ay << ' ' << az << '\n';

    float kx2 = dt2 * ax;
    float ky2 = dt2 * ay;
    float kz2 = dt2 * az;

    float lx2 = dt2 * (obj.vx + kx1);
    float ly2 = dt2 * (obj.vy + ky1);
    float lz2 = dt2 * (obj.vz + kz1);

    GetAccel(obj.x + lx2, obj.y + ly2, obj.z + lz2,
        obj.vx + kx2, obj.vy + ky2, obj.vz + kz2,
        &ax, &ay, &az);

    float kx3 = dt * ax;
    float ky3 = dt * ay;
    float kz3 = dt * az;

    float lx3 = dt * (obj.vx + kx2);
    float ly3 = dt * (obj.vy + ky2);
    float lz3 = dt * (obj.vz + kz2);

    GetAccel(obj.x + lx3, obj.y + ly3, obj.z + lz3,
        obj.vx + kx3, obj.vy + ky3, obj.vz + kz3,
        &ax, &ay, &az);

    float kx4 = dt2 * ax;
    float ky4 = dt2 * ay;
    float kz4 = dt2 * az;

    float lx4 = dt2 * (obj.vx + kx3);
    float ly4 = dt2 * (obj.vy + ky3);
    float lz4 = dt2 * (obj.vz + kz3);

    World::Object res;
    res.x = obj.x + (lx1 + 2.0 * lx2 + lx3 + lx4) / 3.0;
    res.y = obj.y + (ly1 + 2.0 * ly2 + ly3 + ly4) / 3.0;
    res.z = obj.z + (lz1 + 2.0 * lz2 + lz3 + lz4) / 3.0;
    res.vx = obj.vx + (kx1 + 2.0 * kx2 + kx3 + kx4) / 3.0;
    res.vy = obj.vy + (ky1 + 2.0 * ky2 + ky3 + ky4) / 3.0;
    res.vz = obj.vz + (kz1 + 2.0 * kz2 + kz3 + kz4) / 3.0;
    res.size = obj.size;

    return res;
}

struct next_pos
    : public thrust::unary_function<World::Object, World::Object >
{

    next_pos(float t, float dt) : t(t), dt(dt) {};

    //need constructor with dt and time
    __host__ __device__
        World::Object operator()(const World::Object& x) const
    {
        //std::unordered_map<float, float> m;
        return GetNextState(x, 1, 0);

        //return x;
    }

    float t, dt;
};


void GpuSolve(float dt, float time,
    thrust::host_vector<World::Object>& objects,
    thrust::host_vector<World::Force>& forces,
    size_t stepsNumber,
    std::vector<std::vector<World::Object> >& positions) {
    //here come some trash
    positions.resize(objects.size());
 
    thrust::device_vector<World::Object> d_objects(objects.begin(), objects.end());
    thrust::device_vector<World::Object> d_next_objects(objects.size());
    thrust::host_vector<World::Object> h_next_objects;
    for (size_t i = 0; i < stepsNumber; ++i) {
        thrust::transform(d_objects.begin(), d_objects.end(), d_next_objects.begin(), next_pos(0, 1));
        d_objects = d_next_objects;

        //this section is very slow
        h_next_objects = d_next_objects;
        for (size_t j = 0; j < h_next_objects.size(); ++j) {
            positions[j].push_back(h_next_objects[j]);
        }
    }
}

void GPUSolver::Solver::Solve(
    float dt,
    float time,
    std::vector<World::Object>& objects,
    std::vector<World::Force>& forces,
    size_t stepsNumber,
    std::vector<std::vector<World::Object> >& positions) {

    thrust::host_vector<World::Object> host_objects(objects.begin(), objects.end());
    thrust::host_vector<World::Force> host_forces(forces.begin(), forces.end());
    GpuSolve(dt, time, host_objects, host_forces, stepsNumber, positions);    
}