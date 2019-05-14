#include "gpu_solver.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

struct ForcesOptions {
    //add options for your force here
    bool gravityEnabled, heterogeneousGravityEnabled, airDensityEnabled;
};

__host__ __device__
void GetHeterogeneousGravityAcc(double x, double y, double z,
                                double vx, double vy, double vz,
                                double *ax, double *ay, double *az, ForcesOptions& options) {
    if (!options.heterogeneousGravityEnabled) {
        return;
    }
    double r = sqrt(x * x + y * y + z * z);
    const double pi = 3.14159265359;
    const double EARTH_MASS = 5.972e24;
    const double EARTH_RADIUS = 6.371e6;
    const double GRAVITY_CONSTANT = 6.67408e-11;

    //aliases

    const double M = EARTH_MASS;
    const double G = GRAVITY_CONSTANT;
    const double R = EARTH_RADIUS;


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
    double g_corr_lat = 9.780327 * (1 + 0.0053024 * sin(lat) * sin(lat) - 0.0000058 * sin(2 * lat) * sin(2 * lat));

    // Gravert altitude correction
    double alt = r - R;
    double g_corr_alt = -3.086e-6 * alt;
    double g_corr = g_corr_lat + g_corr_alt;
    double k = -g_corr / r;

    *ax += x * k;
    *ay += y * k;
    *az += z * k;
};

__host__ __device__
double GpuGetAirDensity(double x, double y, double z,
    double vx, double vy, double vz,
    double *ax, double *ay, double *az, const ForcesOptions& options) {
    if (!options.airDensityEnabled) {
        return 0.0;
    }
    // TODO: Look up table for atmosphere density
    return 0.0;
}

__host__ __device__
void GetSimpleGravityForceAcc(double x, double y, double z,
    double vx, double vy, double vz,
    double *ax, double *ay, double *az, const ForcesOptions& options) {
    if (!options.gravityEnabled) {
        return;
    }
    const double M = 5.972e24;
    const double G = 6.67408e-11;

    double r = sqrt(x * x + y * y + z * z);
    double k = -G * M / r / r / r;
    *ax = x * k;
    *ay = y * k;
    *az = z * k;
};

__host__ __device__
    void GetAccel(double x, double y, double z,
        double vx, double vy, double vz,
        double *ax, double *ay, double *az, const ForcesOptions& options) {
    *ax = 0.0;
    *ay = 0.0;
    *az = 0.0;
    //add your custom force call here
    GetSimpleGravityForceAcc(x, y, z, vx, vy, vz, ax, ay, az, options);
};

__host__ __device__
World::Object GetNextState(const World::Object& obj,
                           double dt,
                           double time,
                           const ForcesOptions& options) {
    double dt2 = dt / 2.0;
    double dt1 = time + dt;
    double dh = time + dt2;
    //std::cout << "times: " << time << ' ' << dt2 << ' ' << dt1 << ' ' << dh << '\n';
    double ax, ay, az;
    GetAccel(obj.x, obj.y, obj.z, obj.vx, obj.vy, obj.vz, &ax, &ay, &az, options);

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
        &ax, &ay, &az, options);

    //std::cout << ax << ' ' << ay << ' ' << az << '\n';

    double kx2 = dt2 * ax;
    double ky2 = dt2 * ay;
    double kz2 = dt2 * az;

    double lx2 = dt2 * (obj.vx + kx1);
    double ly2 = dt2 * (obj.vy + ky1);
    double lz2 = dt2 * (obj.vz + kz1);

    GetAccel(obj.x + lx2, obj.y + ly2, obj.z + lz2,
        obj.vx + kx2, obj.vy + ky2, obj.vz + kz2,
        &ax, &ay, &az, options);

    double kx3 = dt * ax;
    double ky3 = dt * ay;
    double kz3 = dt * az;

    double lx3 = dt * (obj.vx + kx2);
    double ly3 = dt * (obj.vy + ky2);
    double lz3 = dt * (obj.vz + kz2);

    GetAccel(obj.x + lx3, obj.y + ly3, obj.z + lz3,
        obj.vx + kx3, obj.vy + ky3, obj.vz + kz3,
        &ax, &ay, &az, options);

    double kx4 = dt2 * ax;
    double ky4 = dt2 * ay;
    double kz4 = dt2 * az;

    double lx4 = dt2 * (obj.vx + kx3);
    double ly4 = dt2 * (obj.vy + ky3);
    double lz4 = dt2 * (obj.vz + kz3);

    return { obj.x + (lx1 + 2.0 * lx2 + lx3 + lx4) / 3.0,
    obj.y + (ly1 + 2.0 * ly2 + ly3 + ly4) / 3.0,
    obj.z + (lz1 + 2.0 * lz2 + lz3 + lz4) / 3.0,
    obj.vx + (kx1 + 2.0 * kx2 + kx3 + kx4) / 3.0,
    obj.vy + (ky1 + 2.0 * ky2 + ky3 + ky4) / 3.0,
    obj.vz + (kz1 + 2.0 * kz2 + kz3 + kz4) / 3.0,
    obj.size };
}

struct next_pos
    : public thrust::unary_function<World::Object, World::Object> {
        next_pos(double t, double dt, const ForcesOptions& options) : t(t), dt(dt), options(options) {};

    //need constructor with dt and time
    __host__ __device__
        World::Object operator()(const World::Object &x) const {
        //std::unordered_map<double, double> m;
        return GetNextState(x, dt, t, options);
    }
    ForcesOptions options;
    double t, dt;
};

void GpuSolve(double dt, double time,
              thrust::host_vector<World::Object>& objects,
              size_t stepsNumber,
              std::vector<std::vector<World::Object> >& positions,
              const ForcesOptions options) {
    positions.resize(objects.size());

    thrust::device_vector<World::Object> d_objects(objects.begin(), objects.end());
    thrust::device_vector<World::Object> d_next_objects(objects.size());
    thrust::host_vector<World::Object> h_next_objects;
    for (size_t i = 0; i < stepsNumber; ++i) {
        thrust::transform(d_objects.begin(), d_objects.end(), d_next_objects.begin(), next_pos(time, dt, options));
        d_objects = d_next_objects;

        //this section is very slow
        h_next_objects = d_next_objects;
        for (size_t j = 0; j < h_next_objects.size(); ++j) {
            positions[j].push_back(h_next_objects[j]);
        }   
    }
}

ForcesOptions BuildOptions(const std::vector<std::string>& forces) {
    ForcesOptions res;
    for (const auto force : forces) {
        //fill fields you want to pass from config
        if (force == "gravity") {
            res.gravityEnabled = true;
        }
        if (force == "air_density") {
            res.airDensityEnabled = true;
        }
        if (force == "heterogeneous_gravity") {
            res.heterogeneousGravityEnabled = true;
        }
    }
    return res;
}

void GPUSolver::Solver::Solve(
    double dt,
    double time,
    std::vector<World::Object>& objects,
    std::vector<std::string>& forces,
    size_t stepsNumber,
    std::vector<std::vector<World::Object> > &positions) {

    thrust::host_vector<World::Object> host_objects(objects.begin(), objects.end());

    ForcesOptions options = BuildOptions(forces);

    GpuSolve(dt, time, host_objects, stepsNumber, positions, options);
}