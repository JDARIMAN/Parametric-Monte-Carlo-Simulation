/*
Author: Jan Darius Manzanilla
date: 08/19/2026
Stardance Software Project
Parametric Monte Carlo Simulation
*/

/*Libraries*/
#include <iostream>
#include <vector>
#include <random>
#include <limits>
#include <cmath>
#include <numbers>
#include <chrono>
// Paralel Processing Libraries I've worked with
#include <atomic>
#include <thread>
#include <queue>
// ! IMPLEMENT SOMETHING FOR FRONT END

// Note
/* 
Any funny comments that start with !, ?, * are because I use the Better Comments VSCODE extention -these comments are mainly for me.
This project is aimed for me to learn and dive deep into the understanding of ParticleType filters and localization alorhithims. 
This project will constantly be subject to change as I learn more and find more configurations for me to set up. This code will be riddled with comments
*/


/*Data Structures*/
// learning enum class, so cofigurations don't take up less space than strings
constexpr double pi = std::numbers::pi;
// ! STATE SPACE DIMENSIONS (ROBOT MOVEMENT DIMENSIONS)
enum class DriveConfig {
    Tank, //* Moves in 2 dimensions, as it can't move latterally: X(locked), Y, Yaw
    Omni, //* Moves in 3 dimensions, as the x dimension is now free: X, Y, Yaw
    Drone, //* Adheres to the usual dimensionality of objects in a 3D space: X, Y, Z, Pitch, Roll, Yaw
};

enum class LocalizationType {
    Global_Confined, //* Global Localization: A seemingly ironic name; Tracks position within a given space -you know a globe and where you are
    Local_FreeRoam //* Local Localization: Creates a local map and tracks position relative to the created map, sort of like a cartologist
};

// ! THETA REPRESENTS ROBOT HEADING
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
    double track_width;
};
template <> struct PhysParams<DriveConfig::Omni> {
    int num_wheels;
    double off = 0; // fixed offset angle
    double radius; // wheel mount radius from robot center in meters
};
template <> struct PhysParams<DriveConfig::Drone> {

};

// motion structs
template <DriveConfig Drive> struct Velocity; // ? m/s and rad/s
template <> struct Velocity<DriveConfig::Tank> {double vl, vr, w = 0;}; // velocity left and right, and angular; w represents angular velocity
template <> struct Velocity<DriveConfig::Omni> {std::vector<double> v; double w = 0, vx, vy;}; 
template <> struct Velocity<DriveConfig::Drone> {double vx, vy, vz, wx = 0, wy = 0, wz = 0;}; // includes angular velocity for all directions

template <DriveConfig Drive> struct RoboPose;
template <> struct RoboPose<DriveConfig::Tank> {double x, y, theta = 0;};
template <> struct RoboPose<DriveConfig::Omni> {double x, y, theta = 0;};
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
        return dist(rng_engine);
    }
    
    void predict(std::chrono::seconds dt, Velocity<Drive>ctrl_input) {
        if constexpr (Drive == DriveConfig::Tank){
            double v = (ctrl_input.vr + ctrl_input.vl)/2;
            ctrl_input.w = (ctrl_input.vr - ctrl_input.vl)/bot.track_width; 
            double dt_s = dt.count(); 
            // update pose based on velocities
            // ? n_ denotes new
            double n_x = pose.x + v*std::cos(pose.theta)*dt_s;
            double n_y = pose.y + v*std::sin(pose.theta)*dt_s;
            double n_theta = pose.theta + ctrl_input.w*dt_s;

            // decompose motion
            double dr1 = std::atan2(n_y-pose.y, n_x-pose.x) - pose.theta; // delta rotation
            double dtrans = std::sqrt((n_x-pose.x)*(n_x-pose.x) + (n_y-pose.y)*(n_y-pose.y)); // delta translation
            double dr2 = n_theta - pose.theta - dr1;

            // sample noisy version
            double dr1_est = dr1 - sampleNoise(noise.a1*std::abs(dr1) + noise.a2*dtrans);
            double dtrans_est = dtrans - sampleNoise(noise.a3*dtrans + noise.a4*(std::abs(dr1) + std::abs(dr2)));
            double dr2_est = dr2 - sampleNoise(noise.a1*std::abs(dr2) + noise.a2*dtrans);

            // set new to old and apply noise
            pose.x = n_x + dtrans_est*std::cos(n_theta + dr1_est);
            pose.y = n_y + dtrans_est*std::sin(n_theta + dr1_est);
            pose.theta = n_theta + dr1_est + dr2_est;
            
        }
        else if constexpr (Drive == DriveConfig::Omni){
            double dt_s = dt.count(); 

            double sum_x = 0; 
            double sum_y = 0;
            double sum_w = 0;



            for (int i=0; i < bot.num_wheels; ++i){ // velocity summation
                double theta_i = 2*pi*static_cast<double>(i) / static_cast<double>(bot.num_wheels) + bot.off;
                sum_x += ctrl_input.v[i]*std::sin(theta_i);
                sum_y += ctrl_input.v[i]*std::cos(theta_i);
                sum_w += ctrl_input.v[i];
            }
            ctrl_input.vx = -2/static_cast<double>(bot.num_wheels)*sum_x; ctrl_input.vy = 2/static_cast<double>(bot.num_wheels)*sum_y; ctrl_input.w = 2/(static_cast<double>(bot.num_wheels)*bot.radius)*sum_w;

            // sample noisy velocities
            double vx_est = ctrl_input.vx + sampleNoise(noise.a1*ctrl_input.vx*ctrl_input.vx + noise.a2*ctrl_input.vy*ctrl_input.vy);
            double vy_est = ctrl_input.vy + sampleNoise(noise.a1*ctrl_input.vy*ctrl_input.vy + noise.a2*ctrl_input.vx*ctrl_input.vx);
            double w_est = ctrl_input.w + sampleNoise(noise.a3*ctrl_input.w*ctrl_input.w + noise.a4*(ctrl_input.vx*ctrl_input.vx + ctrl_input.vy*ctrl_input.vy));

            // set new to old and apply noise
            pose.x += (vx_est*std::cos(pose.theta) - vy_est*std::sin(pose.theta))*dt_s;
            pose.y += (vx_est*std::sin(pose.theta) +vy_est*std::cos(pose.theta))*dt_s;
            pose.theta += w_est*dt_s;
        }
        else if constexpr (Drive == DriveConfig::Drone){

        }
        else { // just incase something happen
            std::cout << "No driveconfig for prediction step";
        }
    }

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
    double minx, miny, maxx, maxy, orix, oriy; // origins and bounds
    int rows, cols;
};

template <> struct InstanceMap<DriveConfig::Omni> {std::vector<std::vector<bool>> grid; std::vector<std::vector<double>> distfield; double res, l, w, minx, miny, maxx, maxy, orix, oriy; int rows, cols;};

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

                    if (nr < 0 or n_r >= m.rows or n_c < 0 or nc >= m.cols) continue;

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
            occ_cell(row, col);
        }
    }

    bool isfree(double x, double y) const{ // same thing here
        return !isoc(x, y);
    }

    double raycast(RoboPose<Drive> pose, double angleoffset, double r_max) const{ // angle offset in rad for sensors, assisted by claude
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
        // variables
            double theta = pose.theta + angleoffset;
            double dx = std::cos(theta);
            double dy = std::sin(theta);
            // continuous grid space position of beam
            double gx = (pose.x - m.orix) / m.res;
            double gy = (pose.y - m.oriy) / m.res;

            int col = static_cast<int>(std::flor(gx));
            int row = static_cast<int>(std::flor(gy));

            int stepX = (dx >0) ? 1 : -1;
            int stepY = (dy 0) ? 1 : -1;

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
                    return t * m.res;
                }
            }
            return r_max;
        }


        if constexpr(Drive == DriveConfig::Drone){

        }
    
    }

    double proxima(double x, double y) const{ // distance to nearest object
        if constexpr(Drive == DriveConfig::Tank or Drive == DriveConfig::Omni){
            auto row [rol, col] = w2g(x, y);
            bool out = (row < 0 or row >= m.rows or col < 0 or col >= m.cols);

            if (out) {
                if constexpr(lcl == LocalizationType::Global_Confined) {return 0;} // touching wall
                else {return std::numeric_limits<double>::infinity();} // essentially no wall detected
                return m.distfield[row][col];
            }
        }
        else if constexpr(Drive == DriveConfig::Drone){
            return 0; // todo 3d part
        }
    }


};

// weight computations for update step, with help from claude
template <DriveConfig Drive, LocalizationType Local> static double compute(
    const RoboPose<Drive>& pose, const Measurement<SensorType::ToFSensor>& measure, 
    const Map<Local, Drive>& map, const SensorNoise<SensorType::ToFSensor>& noise);



template <SensorType Sensor> struct WeightCompute {
    template <DriveConfig Drive, LocalizationType Local> static double compute(
        const RoboPose<Drive>& pose, const Measurement<Sensor>& measure, const Map<Local, Drive>& map, const SensorNoise<Sensor>& noise);
};

template <> struct WeightCompute<SensorType::ToFSensor> {
    template <DriveConfig Drive, LocalizationType Local> static double compute(
        const RoboPose<Drive>& pose, const Measurement<SensorType::ToFSensor>& measure, const Map<Local, Drive>& map, const SensorNoise<SensorType::ToFSensor>& noise, double angoff, double r_max){
            double z_star = map.raycast(pose, angoff, r_max);
            double diff = measure.dist - z_star;
            return std::exp(-(diff*diff) / (2 * noise.s_tof * noise.s_tof));
        }
};

template <> struct WeightCompute<SensorType::LandmarkBearing> {}; // todo work on landmark bearing sensors

template <> struct WeightCompute<SensorType::LiDAR> {
    template <DriveConfig Drive, LocalizationType Local> static double compute(
        const RoboPose<Drive>& pose, const Measurement<SensorType::LiDAR>& measure, const Map<Local, Drive>& map, const SensorNoise<SensorType::LiDAR>& noise, const SensorConfig<SensorType::LiDAR>& config){
            double beam_sum = 0;
            for (int b=0; b < config.num_beams; ++b){ // calculate for each beam
                double phi_b = -config.fov/2 + b*(config.fov/(config.num_beams - 1)); // beam angle from heading
                double x_end = pose.x + measure.ranges[b]*std::cos(pose.theta + phi_b);
                double y_end = pose.y + measure.ranges[b]*std::sin(pose.theta + phi_b);

                // beam likelyhood formula
                double db = map.proxima(x_end, y_end);
                double pb = noise.z_hit*std::exp(-(db*db/(2*noise.s_beam*noise.s_beam))) + noise.z_max; // this one

                // weight calculations based on all beams
                beam_sum += std::log(pb);
            }
            return std::exp(beam_sum); // return final weight

        }
};

template <DriveConfig Drive, LocalizationType Local, SensorType Sensor>
double computeWeight(const RoboPose<Drive>& pose,
                      const Measurement<Sensor>& measurement,
                      const Map<Local, Drive>& map,
                      const SensorNoise<Sensor>& noise, 
                      const SensorConfig<Sensor>& config) {
    return WeightCompute<Sensor>::compute<Drive, Local>(pose, measurement, map, noise, config.angoff, config.r_max);
}




template <DriveConfig Drive, SensorType Sensor, LocalizationType Local> class MCLSim {
    private:
    Robot<Drive> bot;
    Map<Local, Drive> space;
    // particle related instance variables
    int num_particles;
    ParticleType<Drive> particles;


    // methods
    // methods
    void applyMeasures(){
        
    }
    public:


    // methods
    void update(){ // update step

    }

};











/*Main Func*/
int main(){
    return 0;
}