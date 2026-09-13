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
// ! IMPLEMENT SOMETHING FOR FRONT END

// Note
/* 
Any funny comments that start with !, ?, * are because I use the Better Comments VSCODE extention -these comments are mainly for me.
This project is aimed for me to learn and dive deep into the understanding of ParticleType filters and localization alorhithims. 
This project will constantly be subject to change as I learn more and find more configurations for me to set up. This code will be riddled with comments
*/



/*Data Structures*/
// learning enum class, so cofigurations don't take up less space than strings
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
    double wheel_mnt_ang; // wheel mounting angle
    double radius; // radius from robot center in meters
};
template <> struct PhysParams<DriveConfig::Drone> {

};

// motion structs
template <DriveConfig Drive> struct Velocity; // ? m/s and rad/s
template <> struct Velocity<DriveConfig::Tank> {double vl, vr, w = 0;}; // velocity left and right, and angular; w represents angular velocity
template <> struct Velocity<DriveConfig::Omni> {double v1, v2, v3, v4, w = 0, vx, vy;}; // ! figure out the inverse kinematics
template <> struct Velocity<DriveConfig::Drone> {double vx, vy, vz, wx = 0, wy = 0, wz = 0;}; // includes angular velocity for all directions

template <DriveConfig Drive> struct RoboPose;
template <> struct RoboPose<DriveConfig::Tank> {double x, y, theta = 0;};
template <> struct RoboPose<DriveConfig::Omni> {double x, y, theta = 0;};
template <> struct RoboPose<DriveConfig::Drone> {double x, y, z, qw, qx, qy, qz;}; // ! LEARN QUATERNIAN ANGLES

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
template <> struct Measurement<SensorType::ToFSensor> { // your ultrasonic and lazer distance sensors
    double dist;
};
template <> struct Measurement<SensorType::LandmarkBearing> { // specialized cameras
    double dist, angle;
    int landmark_id;
};
template <> struct Measurement<SensorType::LiDAR> {
    std::vector<double> ranges; // allots all our measurements in 1 place since lidar takes a bunch
};

// Robot base Template
template <DriveConfig Drive> class Robot { // ? template determines what type of robot it is, no need to declare it as an instance variable
    private:
    RoboPose<Drive> pose;
    PhysParams<Drive> bot;
    NoiseParams<Drive> noise;
    std::mt19937 rng_engine(seed); // random bit generator from the random lib
    public:
    // constructor

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
            // update pose based on velocities
            // ? n_ denotes new
            double n_x = pose.x + v*std::cos(pose.theta)*dt;
            double n_y = pose.y + v*std::cos(pose.theta)*dt;
            double n_theta = pose.theta + w*dt;

            // decompose motion
            double dr1 = std::atan2(n_y-pose.y, n_x-pose.x) - pose.theta; // delta rotation
            double dtrans = std::sqrt((n_x-pose.x)*(n_x-pose.x) + (n_y-pose.y)*(n_y-pose.y)); // delta translation
            double dr2 = n_theta - pose.theta - dr1;

            // sample noisy version
            double dr1_est = dr1 - sampleNoise(noise.a1*std::abs(dr1) + noise.a2*dtrans);
            double dtrans_est = dtrans - sampleNoise(noise.a3*dtrans + noise.a4*(std::abs(dr1) + std::abs(dr2)));
            double dr2_est = dr2 - sampleNoise(a1*std::abs(dr2) + a2*dtrans);

            // set new to old and apply noise
            pose.x = n_x + dtrans_est*std::cos(n_theta + dr1_est);
            pose.y = n_y + dtrans_est*std::sin(n_theta + dr1_est);
            pose.theta = n_theta + dr1_est + dr2_est;
            
        }
        else if constexpr (Drive == DriveConfig::Omni){

        }
        else if constexpr (Drive == DriveConfig::Drone){

        }
        else { // just incase something happen

        }
    }

};






template <LocalizationType lcl, DriveConfig Drive> class Map {
    private:
    std::vector<std::vector<bool>> grid; // occupancy grid
    double resolution; // meters per cell
    double length, width;
    std::vector<std::vector<double>> distfield; // distance grid
    double min_x, min_y, max_x, max_y;
    
    public:
    // constructor

    // methods
    void comp_distfield(){}

    void w2g (double x, double y){ // distfield to grid 

    }

    void g2w(double x, double y){ // grid to distfield 

    }

    bool isoc(double x, double y){ // is occupied, connects cords to grid map 

    }

    bool isfree(double x, double y){ // same thing here

    }

    double raycast(RoboPose pose, ParticleType<Drive> particle, double angleoffset){ // angle offset in rad for sensors

    }

    double proxima(double x, double y){ // distance to nearest object

    }


    

};

template <DriveConfig Drive, SensorType Sensor, LocalizationType Local> class MCLSim {
    private:
    Robot<Drive> bot;
    Map<Local> space;
    // particle related instance variables
    int num_particles;
    ParticleType<Drive> particles;

    public:

};











/*Main Func*/
int main(){

}