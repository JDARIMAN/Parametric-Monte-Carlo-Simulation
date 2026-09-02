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

//* robot parameter structs
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
template <DriveConfig Drive> struct Velocity;

template <> struct Velocity<DriveConfig::Tank> {double vl, vr, theta = 0;}; // velocity left and right

template <> struct Velocity<DriveConfig::Omni> {double v1, v2, v3, v4, theta = 0;}; // 

template <> struct Velocity<DriveConfig::Drone> {double vx, vy, vz, qw, qx, qy, qz;}; // ! LEARN QUATERNIAN ANGLES


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
    int num_particles;
    ParticleType<Drive> particles;
    public:
    // constructor

    // functions
    void predcit(std::chrono::seconds dt, Velocity ctrl_input) {
        if constexpr (Drive == DriveConfig::Tank){
            
        }
        else if constexpr (Drive == DriveConfig::Omni){

        }
        else if constexpr (Drive == DriveConfig::Drone){

        }
        else { // just incase something happen

        }
    }

};





// Map point templates
template <DriveConfig Drive> struct Point;
template <> struct Point<DriveConfig::Tank> {double x, y;};
template <> struct Point<DriveConfig::Omni> {double x,y;};
template <> struct Point <DriveConfig::Drone> {double x, y, z;};



template <LocalizationType lcl> class Map {
    
};

template <DriveConfig Drive, SensorType Sensor>class MCLSim {
    private:
    Robot<Drive> bot;
    public:

};











/*Main Func*/
int main(){

}