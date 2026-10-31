/*
Author: Jan Darius Manzanilla
date: 08/19/2026
Stardance Software Project
Parametric Monte Carlo Simulation
*/

/*Libraries*/
#include <ios>
#include <iostream>
#include <vector>
#include <random>
#include <limits>
#include <cmath>
#include <numbers>
#include <chrono>
#include <algorithm>
#include <string>
#include <utility>
// Paralel Processing Libs I've worked with
#include <atomic>
#include <thread>
#include <queue>
// libs for frontend
#include<iomanip> // for frontend io formatting
#include <fstream> // fileout for our frontend

// since our frontend will be outside of this codefile, we'll need to send out packets using file io

// Note
/* 
Any funny comments that start with !, ?, * are because I use the Better Comments VSCODE extention -these comments are mainly for me.
This project is aimed for me to learn and dive deep into the understanding of ParticleType filters and localization alorhithims. 
This project will constantly be subject to change as I learn more and find more configurations for me to set up. This code will be riddled with comments
*/


/*Data Structures*/
// learning enum class, so cofigurations don't take up less space than strings
constexpr double pi = std::numbers::pi;
// STATE SPACE DIMENSIONS (ROBOT MOVEMENT DIMENSIONS)
enum class DriveConfig {
    Tank, //* Moves in 2 dimensions, as it can't move latterally: X(locked), Y, Yaw
    Omni, //* Moves in 3 dimensions, as the x dimension is now free: X, Y, Yaw
    Drone, //* Adheres to the usual dimensionality of objects in a 3D space: X, Y, Z, Pitch, Roll, Yaw
};

enum class LocalizationType {
    Global_Confined, //* Global Localization: A seemingly ironic name; Tracks position within a given space -you know a globe and where you are
    Local_FreeRoam //* Local Localization: Creates a local map and tracks position relative to the created map, sort of like a cartologist
};


// learning templates, they're pretty cool
// particle structs
template <DriveConfig Drive> struct ParticleType; // ? Must declare templates before assigning them;

template <> struct ParticleType<DriveConfig::Tank> {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double weight = 1;
};

template <> struct ParticleType<DriveConfig::Omni> {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double weight = 1;
};

template <> struct ParticleType<DriveConfig::Drone> {
    double x = 0;
    double y = 0;
    double z = 0;
    double pitch = 0;
    double roll = 0;
    double yaw = 0;
    double weight = 1;
};

// sensor configs
template <DriveConfig Drive> struct PhysParams;
template <> struct PhysParams<DriveConfig::Tank> {
    double track_width = 0.3; // default track width
};
template <> struct PhysParams<DriveConfig::Omni> {
    int num_wheels = 4; // standards four, so just set default constructor
    double off = 0; // fixed offset angle
    double radius = 0.1; // wheel mount radius from robot center in meters
};
template <> struct PhysParams<DriveConfig::Drone> {

};

// motion structs
template <DriveConfig Drive> struct Velocity; // ? m/s and rad/s
template <> struct Velocity<DriveConfig::Tank> {double vl = 0, vr = 0, w = 0;}; // velocity left and right, and angular; w represents angular velocity
template <> struct Velocity<DriveConfig::Omni> {std::vector<double> v; double w = 0, vx = 0, vy = 0;}; 
template <> struct Velocity<DriveConfig::Drone> {double vx, vy, vz, wx = 0, wy = 0, wz = 0;}; // includes angular velocity for all directions

template <DriveConfig Drive> struct RoboPose;
template <> struct RoboPose<DriveConfig::Tank> {double x = 0, y = 0, yaw = 0;}; 
template <> struct RoboPose<DriveConfig::Omni> {double x = 0, y = 0, yaw = 0;};
template <> struct RoboPose<DriveConfig::Drone> {double x, y, z, qw, qx, qy, qz;}; // todo LEARN QUATERNIAN ANGLES

// noise parameter struct; default noise values given by claude
template <DriveConfig Drive> struct NoiseParams;

template <> struct NoiseParams<DriveConfig::Tank> {
    double a1 = 0.1; // rot1 error from rotation
    double a2 = 0.1; // rot1 error from translation
    double a3 = 0.1; // trans error from translation
    double a4 = 0.1; // trans error from rotation
};

template <> struct NoiseParams<DriveConfig::Omni> {
    double a1 = 0.1; // v_x noise scaling
    double a2 = 0.1; // cross-term v_y->v_x noise
    double a3 = 0.1; // omega noise scaling
    double a4 = 0.1; // cross-term v->omega noise
};

template <> struct NoiseParams<DriveConfig::Drone> {
    double vxn = 0.05;   // linear velocity noise stddev source (m/s)
    double vyn = 0.05;   // vertical velocity noise stddev source (m/s)
    double rolln = 0.02;   // roll rate noise stddev source (rad/s)
    double pitchn = 0.02; // pitch rate noise stddev source (rad/s)
    double yawn = 0.02;   // yaw rate noise stddev source (rad/s)
};


// sensor configs
enum class SensorType {ToFSensor, LandmarkBearing, LiDAR};

template <SensorType Sensor> struct Measurement;
template <> struct Measurement<SensorType::ToFSensor> {double dist;}; // your ultrasonic and lazer distance sensors

template <> struct Measurement<SensorType::LandmarkBearing> { // specialized cameras
    double dist, angle;
    int landmark_id;
};

template <SensorType Sensor> struct SensorConfig; 
template <> struct SensorConfig<SensorType::ToFSensor> {
    double angleoffset = 0.0; // sensor mount angle relative to robot heading, radians
    double r_max = 5.0;       // max sensor range, meters
};

template <> struct SensorConfig<SensorType::LandmarkBearing> {
    double r_max = 5.0; // max sensor range, meters
};

template <> struct SensorConfig<SensorType::LiDAR> {
    double r_max = 5.0;   // max sensor range, meters
    double fov = 2 * pi;  // total angular field of view, radians
    int num_beams = 360;  // number of beams per scan
};

template <> struct Measurement<SensorType::LiDAR> {std::vector<double> ranges;}; // allots all our measurements in 1 place since lidar takes a bunch
// Sensor noise configs
template <SensorType Sensor> struct SensorNoise;
template <> struct SensorNoise<SensorType::ToFSensor> {
    double s_tof = 0.05; // std deviation of range of noise, meters -how much you trust your Tof reading
};

template <> struct SensorNoise<SensorType::LandmarkBearing> {
    double s_range = 0.1; // std deviation of range component in meters
    double s_bearing = 0.05; // std deviation of bearing component in radians
};

template <> struct SensorNoise<SensorType::LiDAR> {
    double s_beam = 0.05; // std deviation per beam of Gaussian noise in meters
    double z_hit = 0.9; 
    double z_max = 0.1;
};

// functions
double wrap(double a) {return atan2(std::sin(a), std::cos(a));}// wraps our angles, preventing them from exceeding ±pi

double sampleNoise(std::mt19937& rng, double sigma) { // sample noise model given by claude representing standard distribution
        if (sigma <= 0.0){return 0.0;}
        else{
            std::normal_distribution<double> dist(0.0, sigma);
            return dist(rng);
        }
    }

template <DriveConfig Drive, typename State> void move(State& s, Velocity<Drive> vm, double dt, const PhysParams<Drive>& phys, const NoiseParams<Drive>& noise, std::mt19937& rng){ // apply motion to bot
    if constexpr(Drive == DriveConfig::Tank){
        double v = (vm.vr + vm.vl)/2; // avg velocity
        double w = (vm.vr - vm.vl)/phys.track_width;

        // ideal future pose, these new variables are noiseless
        double n_x = s.x + v*std::cos(s.yaw)*dt;
        double n_y = s.y + v*std::sin(s.yaw)*dt;
        double n_yaw = s.yaw + w*dt;

        // decompose motion
        double dx = n_x - s.x;
        double dy = n_y - s.y;
        double dtrans = std::sqrt(dx*dx + dy*dy);
        double dr1, dr2;
        if (dtrans < 1e-6){ // any value input into atan2 that appraoches 0 returns a garbage value
            dr1 = 0.0;
            dr2 = wrap(n_yaw-s.yaw);
        }
        else {
            dr1 = wrap(std::atan2(dy, dx) - s.yaw);
            dr2 = wrap(n_yaw - s.yaw - dr1);
        }

        double dr1_est = dr1 - sampleNoise(rng, noise.a1 * std::abs(dr1) + noise.a2 * dtrans);
        double dtrans_est = dtrans - sampleNoise(rng, noise.a3 * dtrans + noise.a4*std::abs((dr1) + std:: abs(dr2)));
        double dr2_est = dr2 - sampleNoise(rng, noise.a1*std::abs(dr2) + noise.a2*dtrans);

        s.x += dtrans_est * std::cos(s.yaw + dr1_est);
        s.y += dtrans_est * std::sin(s.yaw + dr1_est);
        s.yaw = wrapAngle(s.yaw + dr1_est + dr2_est);
    }
    else if constexpr(Drive == DriveConfig::Omni){
        double N = static_cast<double>(phys.num_wheels); // just to prevent accidental integer division
        double sum_x = 0, sum_y = 0, sum_w = 0;
        for (int i = 0; i < phys.num_wheels; ++i) {
            double theta_i = 2 * pi * static_cast<double>(i) / N + phys.off;
            sum_x += vm.v[i] * std::sin(theta_i);
            sum_y += vm.v[i] * std::cos(theta_i);
            sum_w += vm.v[i];
        }
        vm.vx = -2 / N * sum_x;
        vm.vy = 2 / N * sum_y;
        vm.w = 2 / (N * phys.radius) * sum_w;
        
        // sample noisy velocities (sigma = sqrt of the variance terms)
        double vx_est = vm.vx + sampleNoise(rng, std::sqrt(noise.a1 * vm.vx * vm.vx + noise.a2 * vm.vy * vm.vy));
        double vy_est = vm.vy + sampleNoise(rng, std::sqrt(noise.a1 * vm.vy * vm.vy + noise.a2 * vm.vx * vm.vx));
        double w_est = vm.w + sampleNoise(rng, std::sqrt(noise.a3 *vm.w * vm.w + noise.a4 * (vm.vx * vm.vx + vm.vy * vm.vy)));

        s.x += (vx_est * std::cos(s.yaw) - vy_est * std::sin(s.yaw)) * dt;
        s.y += (vx_est * std::sin(s.yaw) + vy_est * std::cos(s.yaw)) * dt;
        s.yaw = wrapAngle(s.yaw + w_est * dt);
    }
    else if constexpr(Drive == DriveConfig::Drone){
        // TODO
    }
    else { // just incase something happen
        std::cout << "No driveconfig for prediction step";
    }
    
}

// Robot base Template
template <DriveConfig Drive> class Robot { // ? template determines what type of robot it is, no need to declare it as an instance variable
    private:
    RoboPose<Drive> pose;
    PhysParams<Drive> bot;
    NoiseParams<Drive> noise;
    std::mt19937 rng_engine; // random bit generator from the random lib
    public:
    // constructor
    Robot(RoboPose<Drive> initial_pose, PhysParams<Drive> phys_params,
          NoiseParams<Drive> noise_params, unsigned int seed)
        : pose(initial_pose), bot(phys_params), noise(noise_params), rng_engine(seed) {}

    // methods
    double sampleNoise(double b) { // sample noise model given by claude
        double sigma = std::sqrt(b);
        std::normal_distribution<double> dist(0.0, sigma);
        double vari = dist(rng_engine);
        if (vari<=0){return 0;} // prevents method from returning a garbage value
        else{return vari;}
    }
    
    void predict(std::chrono::seconds dt, Velocity<Drive>ctrl_input) {move(pose, ctrl_input, dt, bot, noise, rng_engine);}

    void setPose(const RoboPose<Drive>& p){pose = p;}

    // getter methods
    NoiseParams<Drive> getNoise()const{return noise;}
    PhysParams<Drive> getPhys()const{return bot;}
    RoboPose<Drive> getPose()const{return pose;}

};



// obstacle struct for map
template <DriveConfig Drive> struct Obstacle {};
template <> struct Obstacle<DriveConfig::Tank> {double x, y;};
template <> struct Obstacle<DriveConfig::Omni> {double x, y;};
template <> struct Obstacle<DriveConfig::Drone> {double x, y, z;};

// template for my map instance variables
template <DriveConfig Drive> struct InstanceMap{};

template <> struct InstanceMap<DriveConfig::Tank> {
    std::vector<std::vector<bool>> grid; // occupancy grid
    double res; // resolution -meters per cell
    double l, w; // length & width -basically fov from sensors if local localization model
    std::vector<std::vector<double>> distfield; // distance grid
    double minx = 0, miny = 0, maxx = 0, maxy = 0, orix = 0, oriy = 0; // origins and bounds
    int rows = 0, cols = 0;
};

template <> struct InstanceMap<DriveConfig::Omni> {
    std::vector<std::vector<bool>> grid; 
    std::vector<std::vector<double>> distfield; 
    double res = 0.1, l = 0, w = 0, minx = 0, miny = 0, maxx = 0, maxy = 0, orix = 0, oriy = 0; 
    int rows = 0, cols = 0;};

template <> struct InstanceMap<DriveConfig::Drone> {
    std::vector<std::vector<std::vector<bool>>> grid;
    double res, l, w, h;
    std::vector<std::vector<std::vector<double>>> distfield;
    double minx, miny, minz, maxx, maxy, maxz;
};

template <LocalizationType lcl, DriveConfig Drive> class Map {
    private:
    InstanceMap<Drive> m;
    std::vector<Obstacle<Drive>> obs; // vector of obstacles

    // methods
    // todo Rearrange grid conversions once drone
    std::pair<int, int> w2g(double x, double y) const { // distfield to grid 
        int col = static_cast<int>(std::floor((x - m.orix) / m.res));
        int row = static_cast<int>(std::floor((y - m.oriy) / m.res));
        return {row, col};
    }

    std::pair<double, double> g2w(int row, int col) const { // grid to distfield 
        double x = m.orix + (col + 0.5) * m.res;
        double y = m.oriy + (row + 0.5) * m.res;
        return {x, y};
    }

    void comp_grid (){
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            m.rows = static_cast<int>(std::ceil(m.l/m.res));
            m.cols = static_cast<int>(std::ceil(m.w/m.res));

            for (int r=0; r < m.rows; ++r){ // create each tile
                std::vector<bool> row;
                for (int col=0; col < m.cols; ++col){
                    row.push_back(false);
                }
                m.grid.push_back(row);
            }
        }
        else if constexpr(Drive == DriveConfig::Drone){

        }
    }

    void place_obstacles(){
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            for (const auto& o : obs) {
                auto [row, col] = w2g(o.x, o.y);

                if (row >= 0 && row < m.rows && col >= 0 && col < m.cols) {
                    m.grid[row][col] = true;
                }
            }
        }
        else if constexpr(Drive == DriveConfig::Drone){

        }

    }

    void comp_distfield(){ // utilizes a bfs, help from claude
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            for (int r=0; r < m.rows; ++r){ 
                std::vector<double> row;
                for (int col=0; col < m.cols; ++col){
                    row.push_back(std::numeric_limits<double>::infinity());
                }
                m.distfield.push_back(row);
            }

            std::queue<std::pair<int,int>> q; // queue to store adjacent cells that were updated and so fourth

            for (int r=0; r < m.rows; ++r){
                for (int c = 0; c < m.cols; ++c){
                    if (m.grid[r][c]) { // if the cell exists
                        m.distfield[r][c] = 0; 
                        q.push({r, c}); 
                    }
                }
            }

            static const int dr[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
            static const int dc[8] = {0, 0, -1, 1, -1, 1, -1, 1};

            while (!q.empty()) {
                auto [curRow, curCol] = q.front(); 
                q.pop(); // pop our finalized value

                for (int i=0; i < 8; ++i) {
                    int n_r = curRow + dr[i];
                    int n_c = curCol + dc[i];

                    if (n_r < 0 or n_r >= m.rows or n_c < 0 or n_c >= m.cols) continue;

                    double stpcost = (dr[i] != 0 and dc[i] != 0) ? m.res *std::sqrt(2) : m.res;

                    double candidate = m.distfield[curRow][curCol] + stpcost;

                    if (candidate < m.distfield[n_r][n_c]){
                        m.distfield[n_r][n_c] = candidate;
                        q.push({n_r, n_c});
                    }
                }
            }
        }
        else if constexpr(Drive == DriveConfig::Drone){

        }
    }

    bool occ_cell(int row, int col) const{ // shared occupancy check by grid index
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            bool out = (row < 0 or row >= m.rows or col < 0 or col >= m.cols);

                if constexpr(lcl == LocalizationType::Global_Confined){
                    if (out) return true; // sets all out of bounds values to occupied
                }
                else{
                    if (out) return false; // doesn't set out of bounds values to occupied since constraints are the fov of the sensors
                }
                return m.grid[row][col];
            }
        if constexpr(Drive == DriveConfig::Drone){
            return false; // todo 3d part
        }
    }
    
    public:
    // constructors
    Map(InstanceMap<Drive> parameters, std::vector<Obstacle<Drive>> obstacle_list): m(parameters), obs(obstacle_list) {
        // construct our maps components
        comp_grid();
        place_obstacles();
        comp_distfield(); 
    }


    // methods

    bool isoc(double x, double y) const{ // is occupied, connects cords to grid map 
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            auto [row, col] = w2g(x, y);
            return occ_cell(row, col);
        }
        else { // if cell doesn't exist, void values
            (void)x; (void)y;
            return false;
        }
    }

    bool isfree(double x, double y) const{ // same thing here
        return !isoc(x, y);
    }

    const InstanceMap<Drive>& params() const {return m;} // allows MCLsim read the map bounds

    double raycast(RoboPose<Drive> pose, double angleoffset, double r_max) const{ // angle offset in rad for sensors, assisted by claude
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
        // variables
            double theta = pose.yaw + angleoffset;
            double dx = std::cos(theta);
            double dy = std::sin(theta);
            // continuous grid space position of beam
            double gx = (pose.x - m.orix) / m.res;
            double gy = (pose.y - m.oriy) / m.res;

            int col = static_cast<int>(std::floor(gx));
            int row = static_cast<int>(std::floor(gy));

            int stepX = (dx > 0) ? 1 : -1;
            int stepY = (dy > 0) ? 1 : -1;

            double tMaxX = (dx != 0)
                ? ((stepX > 0 ? (col + 1 - gx) : (gx - col)) / std::abs(dx))
                : std::numeric_limits<double>::infinity();
            double tMaxY = (dy != 0)
                ? ((stepY > 0 ? (row + 1 - gy) : (gy - row)) / std::abs(dy))
                : std::numeric_limits<double>::infinity();

            double tDeltaX = (dx != 0) ? 1.0 / std::abs(dx) : std::numeric_limits<double>::infinity();
            double tDeltaY = (dy != 0) ? 1.0 / std::abs(dy) : std::numeric_limits<double>::infinity();

            double t = 0.0;
            double r_max_cells = r_max / m.res;

            while (t <= r_max_cells) {
                if (tMaxX < tMaxY) {
                    col += stepX;
                    t = tMaxX;
                    tMaxX += tDeltaX;
                } else {
                    row += stepY;
                    t = tMaxY;
                    tMaxY += tDeltaY;
                }

                if (occ_cell(row, col)) {
                    return std::min(t * m.res, r_max); // use alg to keep bounds in, well bounds
                }
            }
            return r_max;
        }

        if constexpr(Drive == DriveConfig::Drone){ // placeholder
            (void)pose; (void)angleoffset;
            return r_max;
        }
    }

    double proxima(double x, double y) const{ // distance to nearest object
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            auto [row, col] = w2g(x, y);
            bool out = (row < 0 or row >= m.rows or col < 0 or col >= m.cols);

            if (out) {
                if constexpr(lcl == LocalizationType::Global_Confined) {
                    double ox = std::max({m.orix - x, 0.0, x - (m.orix + m.cols * m.res)});
                    double oy = std::max({m.oriy - y, 0.0, y - (m.oriy + m.rows * m.res)});
                    return std::sqrt(ox * ox + oy * oy);
                } 
                else {return std::numeric_limits<double>::infinity();} // essentially no wall detected
            }
            else if (lcl == LocalizationType::Global_Confined){
                double d_wall = std::min({x - m.orix, m.orix + m.cols * m.res - x, y - m.oriy, m.oriy + m.rows * m.res - y});
                return std::min(m.distfield[row][col], d_wall);
            }
            return m.distfield[row][col];
        }
        else if constexpr(Drive == DriveConfig::Drone){
            return 0; // todo 3d part
        }
        else {
            (void)x; (void)y;
            return 0; // todo 3d part
        }
    }


};

// weight computations for update step, as well as helper with help from claude
double lidarBeamAngle(const SensorConfig<SensorType::LiDAR>& c, int b) {
    if (c.num_beams <= 1) return 0.0;
    // full circle: spacing fov/N so beam 0 and beam N-1 don't overlap at +-pi
    double step = (c.fov >= 2 * pi - 1e-9) ? c.fov / c.num_beams : c.fov / (c.num_beams - 1);
    return -c.fov / 2 + b * step;
}

template <SensorType Sensor> struct WeightCompute {
    template <DriveConfig Drive, LocalizationType Local> static double compute(
        const RoboPose<Drive>& pose, const Measurement<Sensor>& measure, const Map<Local, Drive>& map, const SensorNoise<Sensor>& noise, const SensorConfig<SensorType::ToFSensor> config);
};

template <> struct WeightCompute<SensorType::ToFSensor> {
    template <DriveConfig Drive, LocalizationType Local> static double compute(
        const RoboPose<Drive>& pose, const Measurement<SensorType::ToFSensor>& measure, const Map<Local, Drive>& map, const SensorNoise<SensorType::ToFSensor>& noise, const SensorConfig<SensorType::ToFSensor>& config){
            double z_star = map.raycast(pose, config.angleoffset, config.r_max);
            double diff = measure.dist - z_star;
            return -(diff*diff) / (2 * noise.s_tof * noise.s_tof);
        }
};

template <> struct WeightCompute<SensorType::LandmarkBearing> {}; // todo work on landmark bearing sensors

template <> struct WeightCompute<SensorType::LiDAR> {
    template <DriveConfig Drive, LocalizationType Local> static double compute(
        const RoboPose<Drive>& pose, const Measurement<SensorType::LiDAR>& measure, const Map<Local, Drive>& map, const SensorNoise<SensorType::LiDAR>& noise, const SensorConfig<SensorType::LiDAR>& config){
            double beam_sum = 0;
            for (int b=0; b < config.num_beams; ++b){ // calculate for each beam
                if (measure.ranges[b] >= config.r_max) continue; // if statement for loop, skip beam if it hits max range
                double phi_b = lidarBeamAngle(config, b); // beam angle from heading
                double x_end = pose.x + measure.ranges[b]*std::cos(pose.theta + phi_b);
                double y_end = pose.y + measure.ranges[b]*std::sin(pose.theta + phi_b);

                // beam likelyhood formula
                double db = map.proxima(x_end, y_end);
                double pb = noise.z_hit*std::exp(-(db*db/(2*noise.s_beam*noise.s_beam))) + noise.z_max; // this one

                // weight calculations based on all beams
                beam_sum += std::log(pb);
            }
            return beam_sum; // return final weight

        }
};

template <DriveConfig Drive, LocalizationType Local, SensorType Sensor>
double computeLogWeight(const RoboPose<Drive>& pose, // logarithmic weight
                      const Measurement<Sensor>& measurement,
                      const Map<Local, Drive>& map,
                      const SensorNoise<Sensor>& noise, 
                      const SensorConfig<Sensor>& config) {
    return WeightCompute<Sensor>::compute<Drive, Local>(pose, measurement, map, noise, config);
}

template <DriveConfig Drive, LocalizationType Local, SensorType Sensor>
Measurement<Sensor> genMeasurement(const RoboPose<Drive>& true_pose,
                                        const Map<Local, Drive>& map,
                                        const SensorNoise<Sensor>& noise,
                                        const SensorConfig<Sensor>& config,
                                        std::mt19937& rng) {
    if constexpr (Sensor == SensorType::ToFSensor){
        double d = map.raycast(true_pose, config.angleoffset, config.r_max) + sampleNoise(rng, noise.s_tof);
        return Measurement<Sensor>{std::clamp(d, 0.0, config.r_max)};
    }
    else if constexpr (Sensor == SensorType::LiDAR){
        Measurement<Sensor> out;
        out.ranges.resize(config.num_beams);
        for (int b = 0; b < config.num_beams; ++b){
            double d = map.raycast(true_pose, lidarBeamAngle(config, b), config.r_max) + sampleNoise(rng, noise.s_beam);
            out.ranges[b] = std::clamp(d, 0.0, config.r_max);
        }
        return out;
    }
    else {
        return Measurement<Sensor>{}; // todo LandmarkBearing
    }
}



template <DriveConfig Drive, SensorType Sensor, LocalizationType Local> class MCLSim {
    private:
    Map<Local, Drive> space;
    // particle related instance variables
    int num_particles;
    std::vector<ParticleType<Drive>> particles;
    SensorNoise<Sensor> noise;
    SensorConfig<Sensor> sconfig;
    double neff = 0; // # of effective particles
    PhysParams<Drive> phys;
    NoiseParams<Drive> motion_noise;
    mutable std::mt19937 rng_engine; // ? implement a smaller random number generator on robots with lower stoarge
    // ? mutable allows for change to happen even within const container
    int num_threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency())); // use of our multithread libs, claude assisted

    // likelihood temperature
    // ? not standard in MCL, but prevents bots with lots of beams from collapsing onto the wrong spot due to the adjacent beams accumilating the same error
    // ? Scales log-weights by a factor between 0 and 1, mitigating/softening the error
    // * mainly applicable to LiDAR
    double loglik_scale = 0.3;
    double jitter_xy = 0.02;  // roughening after resample (meters), keeps duplicates from collapsing into one point
    double jitter_yaw = 0.02; 

    void scatterUniform() { // distribute particles over map
        if constexpr (Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            const auto& mp = space.params();
            // deltas
            std::uniform_real_distribution<double> xd(mp.orix, mp.orix + mp.w);
            std::uniform_real_distribution<double> yd(mp.oriy, mp.oriy + mp.l);
            std::uniform_real_distribution<double> thd(-pi, pi);
            for (auto& p : particles){
                do { p.x = xd(rng_engine); p.y = yd(rng_engine);} while (space.isoc(p.x, p.y));
                p.yaw = thd(rng_engine);
                p.weight = 1.0 / num_particles;
            }
        }
    }

    // methods
    void applyMeasures(const Measurement<Sensor>& measurement){
        const double neg_inf = -std::numeric_limits<double>::infinity();
        std::vector<double> logw(num_particles, neg_inf);

        // * multithread evaluates particles in parallel
        // * each thread reads in the map and writes to logw, with atomic handing work distribution without the threads overlapping
        // needed a good amt of help from claude on this multithreading part
        std::atomic<int> next_start{0};
        const int chunk = 64;
        auto worker = [&]() {
            while (true) {
                int start = next_start.fetch_add(chunk);
                if (start >= num_particles) break;
                int end = std::min(start + chunk, num_particles);
                for (int i = start; i < end; ++i){
                    const auto& p = particles[i];
                    if (space.isoc(p.x, p.y) || p.weight <= 0.0) continue; // particle in a wall/obstacle -> weight 0
 
                    RoboPose<Drive> pose; // local pose built from the particle
                    pose.x = p.x; pose.y = p.y; pose.yaw = p.yaw;
                    logw[i] = std::log(p.weight) + loglik_scale * computeLogWeight<Drive, Local, Sensor>(pose, measurement, space, noise, sconfig);
                }
            }
        };
        std::vector<std::thread> pool; // our threads
        for (int t = 1; t < num_threads; ++t) pool.emplace_back(worker); // num_threads - 1 helper
        worker();                                                        
        for (auto& th : pool) th.join();                                 // wait for all helpers before reading logw
 
        // max is found serially AFTER the join (cheap, and avoids a shared max variable between threads)
        double max_logw = neg_inf;
        for (int i = 0; i < num_particles; ++i) max_logw = std::max(max_logw, logw[i]);
 
        if (!std::isfinite(max_logw)) { // safeguard, every particle had zero weight (all lost / all in walls)
            scatterUniform();
            return;
        }
 
        double sum = 0;
        for (int i = 0; i < num_particles; ++i) {
            particles[i].weight = std::exp(logw[i] - max_logw); // log-sum-exp trick
            sum += particles[i].weight;
        }
        for (auto& p : particles) p.weight /= sum; // normalize
    }

    void resample(){ // resample step placed above update step so it can call it after a resample condition
        double sum = 0;
        std::vector<double> csum(num_particles); // cumulative sum
        for (int i=0; i < num_particles-1; ++i){sum += particles[i].weight; csum[i] = sum;} // cumulative sum of normalized weights per particle


        // ? systemic resampling
        std::uniform_real_distribution<double> dist(0.0, 1.0 /num_particles); // draw random number for offset, and evenly space your sample points
        double u0 = dist(rng_engine);
        int i = 0; // index of csum array
        std::vector<ParticleType<Drive>> n_particles;
        n_particles.reserve(num_particles); // allocate memory like an array
        for (int j=0; j < num_particles; ++j){
            double u_j = u0 + static_cast<double>(j) / num_particles;
            while (i < num_particles - 1 and csum[i] < u_j){ // bounds check, then access each value of csum until its bigger than u_j
                ++i;
            }
            n_particles.push_back(particles[i]);
            n_particles.back().weight = 1.0 / num_particles;
            // likelihood temp variables
            n_particles.back().x += sampleNoise(rng_engine, jitter_xy); // roughening
            n_particles.back().y += sampleNoise(rng_engine, jitter_xy);
            n_particles.back().yaw = wrapAngle(n_particles.back().yaw + sampleNoise(rng_engine, jitter_yaw));
        }
        particles = n_particles; // set new particles
    }


    public:
    // construcotr
    MCLSim(Map<Local, Drive> map, std::vector<ParticleType<Drive>> container, SensorNoise<Sensor> noisy, SensorConfig<Sensor> sensortype, PhysParams<Drive> physp, NoiseParams<Drive> motionn, unsigned int seed) // unsigned int just to double its maximum range
        : space(map), particles(container), noise(noisy), sconfig(sensortype), phys(physp), motion_noise(motionn), rng_engine(seed){}


    // methods
    void update(const Measurement<Sensor>& measurement){ // update step
        applyMeasures(measurement);

        double sumsqr = 0; // sum of squared normalized particle weights
        for (auto& p : particles){sumsqr += p.weight*p.weight;}

        neff = 1/sumsqr; // # of effective particles

        if (neff < num_particles/2.0){resample();} // resample if effective particles falls lower than a certain threshold
        // ? you can tune the threshold
    }

    RoboPose<Drive> estimatePose() const { 
        RoboPose<Drive> n_pose;
        double xsum = 0, ysum = 0, sinsum = 0, cossum = 0;
        for (int i=0; i < num_particles; ++i){
            xsum += particles[i].weight * particles[i].x;
            ysum += particles[i].weight * particles[i].y;
            sinsum += particles[i].weight * std::sin(particles[i].yaw);
            cossum += particles[i].weight * std::cos(particles[i].yaw);
        }
        n_pose.x = xsum;
        n_pose.y = ysum;
        n_pose.yaw = std::atan2(sinsum, cossum);

        return n_pose;
    }


    void predict(std::chrono::seconds dt, Velocity<Drive>ctrl_input){ // a call to motion noise
        for (auto& p : particles){motion_noise<Drive>(p, ctrl_input, dt, phys, motion_noise, rng_engine);}
    }

    // getter functions
    void setThreads(int t){num_threads = std::max(1,t);} // @param # threads
    double getNeff()const {return neff;}
    const std::vector<ParticleType<Drive>>& getParticles()const {return particles;} // * useful for logging and frontend
};




/*Main Func*/
int main(int argc, char** argv){ // cmdline args
    // set templates
    // todo figure out how to set each template
    // ! temporary defaults
    constexpr DriveConfig qDrive = DriveConfig::Tank; 
    constexpr SensorType qSensor = SensorType::ToFSensor;
    constexpr LocalizationType qLocal = LocalizationType::Global_Confined;

    static_assert(qDrive == DriveConfig::Tank, "main's scripted controller is Tank-only for now (needs an Omni controller)");
    std::string out_path = (argc > 1) ? argv[1] : "MCLsim_output.jsonl";
    // NEW: optional flags after the output path:  --realtime   --threads=N
    bool realtime = false;
    int threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
    for (int a = 2; a < argc; ++a) {
        std::string arg = argv[a];
        if (arg == "--realtime") realtime = true;
        else if (arg.rfind("--threads=", 0) == 0) threads = std::max(1, std::stoi(arg.substr(10)));
    }



    // * map *
    InstanceMap<qDrive> map_params;
    // defaults
    map_params.res = 0.1; // default 10cm/cell
    map_params.l = 5; // map length
    map_params.w = 5; // map width
    // origin point
    map_params.orix = 0; 
    map_params.oriy = 0;

    std::vector<Obstacle<qDrive>> obstacles;
    auto addRect = [&](double x0, double y0, double x1, double y1){
        for (int r = static_cast<int>(std::round(y0 / map_params.res)); r < static_cast<int>(std::round(y1 / map_params.res)); ++r)
            for (int c = static_cast<int>(std::round(x0 / map_params.res)); c < static_cast<int>(std::round(x1 / map_params.res)); ++c)
                obstacles.push_back({(c + 0.5) * map_params.res, (r + 0.5) * map_params.res});
    };
    // add obstacles
    addRect(3.0, 2.0, 4.5, 3.0);
    addRect(6.0, 5.0, 6.5, 8.0);
    addRect(1.5, 6.0, 3.5, 6.4);
    addRect(7.5, 1.5, 8.5, 2.5);


    Map<qLocal, qDrive> world(map_params, obstacles);

    // Robot's true variables
    RoboPose<qDrive> init_pose;
    init_pose.x = 1.0; init_pose.y = 1.0; init_pose.yaw = 0.0;
    PhysParams<qDrive> phys_params;
    phys_params.track_width = 0.3;
    NoiseParams<qDrive> noise_params;

    Robot<qDrive> robot(init_pose, phys_params, noise_params, 123);


    // *particles* 
    int n = 3000; // # particles
     std::vector<ParticleType<qDrive>> init_particles;
    std::mt19937 init_rng(321); // rng engine

    std::uniform_real_distribution<double> x_dist(map_params.orix, map_params.orix + map_params.w);
    std::uniform_real_distribution<double> y_dist(map_params.oriy, map_params.oriy + map_params.l);
    std::uniform_real_distribution<double> yaw_dist(-pi, pi);

    while(static_cast<int>(init_particles.size()) < n){
        ParticleType<qDrive> p;
        p.x = x_dist(init_rng);
        p.y = y_dist(init_rng);
        p.yaw = yaw_dist(init_rng);
        p.weight = 1.0/n;
        if (world.isfree(p.x, p.y))init_particles.push_back(p); // append each noisy particle
    }

    SensorNoise<qSensor> sensor_noise;
    SensorConfig<qSensor> sensor_config;

    if constexpr (qSensor == SensorType::LiDAR) {
        sensor_config.num_beams = 72; // ? error is a result of the sensor config being different from the one set above
        sensor_noise.s_beam = 0.15;
    }

    MCLSim<qDrive, qSensor, qLocal> sim(world, init_particles, sensor_noise, sensor_config, phys_params, noise_params, 456);

    sim.setThreads(threads);
    std::mt19937 meas_rng(789); // seperate noise measure for sensors

    // fileo
    std::ofstream out(out_path);
    if (!out){std::cerr << "Unable to open " << out_path << "\n"; return 1;}
    out << std::fixed << std::setprecision(3);

    double dt = 0.1;
    int steps = 400; 
    using Clock = std::chrono::steady_clock; // just to save time given my deadline
    std::chrono::duration<double> t_predict{0}, t_upd{0}; // acumulated wall time
    const auto step_len = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(0.1)); // dt but using chrono essentially
    auto next_deadline = Clock::now;
    const auto run_start = Clock::now();

    const int log_stride = std::max(1, n/500); // limit to only 500ish particles per frame to keep file small

    // file output for map
    out << "{\"type\":\"map\",\"res\":" << map_params.res << ",\"w\":" << map_params.w << ",\"l\":" << map_params.l
        << ",\"orix\":" << map_params.orix << ",\"oriy\":" << map_params.oriy
        << ",\"dt\":" << dt << ",\"steps\":" << steps << ",\"n\":" << (n + log_stride - 1) / log_stride << ",\"obstacles\":[";
    for (size_t i = 0; i < obstacles.size(); ++i) {
        out << (i ? "," : "") << "[" << obstacles[i].x << "," << obstacles[i].y << "]";
    }
    out << "]}\n";

    // *main localization loop*
    for (int k = 0; k < steps; ++k){
        // scripted controller for the TRUE robot: wander, spin in place when something is close ahead
        RoboPose<qDrive> tp = robot.getPose();
        Velocity<qDrive> u;
        if (world.raycast(tp, 0.0, 1.0) < 0.6){u.vl = -0.3; u.vr = 0.3;}
        else{ u.vl = 0.40; u.vr = 0.45;}
 
        // true robot moves; if it ended up inside a wall, undo the move
        RoboPose<qDrive> prev = robot.getPose();
        robot.predict(dt, u);
        RoboPose<qDrive> cur = robot.getPose();
        if (world.isoc(cur.x, cur.y)) robot.setPose(prev);
 
        auto t0 = Clock::now();
        sim.predict(dt, u); // particles move
        auto t1 = Clock::now();
        t_predict += t1 - t0;
        auto z = genMeasurement<qDrive, qLocal, qSensor>(robot.getPose(), world, sensor_noise, sensor_config, meas_rng);
        auto t2 = Clock::now();
        sim.update(z);// weight + maybe resample(?)
        t_update += Clock::now() - t2;
        RoboPose<qDrive> est = sim.estimatePose();
 
        RoboPose<qDrive> tr = robot.getPose();
        out << "{\"type\":\"step\",\"k\":" << k << ",\"t\":" << k * dt
            << ",\"truth\":[" << tr.x << "," << tr.y << "," << tr.yaw << "]"
            << ",\"est\":[" << est.x << "," << est.y << "," << est.yaw << "]"
            << ",\"neff\":" << sim.getNeff() << ",\"particles\":[";
        const auto& ps = sim.getParticles();
        bool first = true;
        for (size_t i = 0; i < ps.size(); i += log_stride){ // every log_stride of the nth particle. w is weight * N (1.0 = uniform)
            out << (first ? "" : ",") << "[" << ps[i].x << "," << ps[i].y << "," << ps[i].yaw << "," << ps[i].weight * n << "]";
            first = false;
        }
        out << "]}\n";
 
        if (k % 20 == 0) std::cout << "step " << k << "  Neff = " << sim.getNeff() << "\n";
 
        if (realtime){ // real time mode 
            next_deadline += step_len;
            std::this_thread::sleep_until(next_deadline);
        }
    }
 
    // timing summary
    std::chrono::duration<double> total = Clock::now() - run_start;
    std::cout << std::fixed << std::setprecision(2)
              << "threads: " << threads << "  |  total " << total.count() << " s"
              << "  |  predict " << 1000 * t_predict.count() / steps << " ms/step"
              << "  |  update " << 1000 * t_update.count() / steps << " ms/step\n";
    std::cout << std::setprecision(6);
 
    RoboPose<qDrive> fin_t = robot.getPose(), fin_e = sim.estimatePose();
    std::cout << "final position error: " << std::hypot(fin_t.x - fin_e.x, fin_t.y - fin_e.y) << " m\n";
    std::cout << "wrote " << out_path << "\n";


    // todo implement way for frontend to adjust variables

    
    return 0;
}