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
This project is aimed for me to learn and dive deep into the understanding of particle filters and localization alorhithims. 
This project will constantly be subject to change as I learn more and find more configurations for me to set up. This code will be riddled with comments
*/


/*Data Structures*/
// learning enum class, so cofigurations don't take up less space than strings
// ! STATE SPACE DIMENSIONS (ROBOT MOVEMENT DIMENSIONS)
enum class DriveConfig {
    Tank, //* Moves in 2 dimensions, as it can't move latterally: X(locked), Y, Yaw
    OmniDirectional, //* Moves in 3 dimensions, as the x dimension is now free: X, Y, Yaw
    Drone, //* Adheres to the usual dimensionality of objects in a 3D space: X, Y, Z, Pitch, Roll, Yaw
};

enum class LocalizationType {
    Global_Confined, //* Global Localization: A seemingly ironic name; Tracks position within a given space -you know a globe and where you are
    Local_FreeRoam //* Local Localization: Creates a local map and tracks position relative to the created map, sort of like a cartologist
};

// ! THETA REPRESENTS ROBOT HEADING
// learning templates, they're pretty cool
template <DriveConfig Drive> struct Particle; // ? Must declare templates before assigning them;

template <> struct Particle<DriveConfig::Tank> {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double weight = 0;
};

template <> struct Particle<DriveConfig::OmniDirectional> {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double weight = 0;
};

template <> struct Particle<DriveConfig::Drone> {
    double x = 0;
    double y = 0;
    double z = 0;
    double pitch = 0;
    double roll = 0;
    double yaw = 0;
    double weight = 0;
};















/*Main Func*/
int main(){

}