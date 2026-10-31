# Parametric-Monte-Carlo-Simulation
#### A Monte Carlo Simulation that takes in parameters to structure the localization algorithm for robots and optimize it

## What is Monte Carlo Localization?
Monte Carlo Localization is a form of robot localization using a particle filter, which utilizes a probabalilstic algorithim to estimate the pose based on distance sensor readings and odometry.
a # of particles are distributed around the given map of a robot, with readings and a motion model deciding on the distribution every time the particles are updated.
These particles are given a weight depending on the coorespondence with sensor readings -- particles with lower readings die off, with particles with higher weights eventually converging into the robots 'true pose', which is why MCL is often reffered to as a 'survival of the fittest' algorithim.
These particles are eventually resampled, adjusting the particle positions and adjusting the # of particles and updating the robot's pose once again.

## Purpose of this project?
The purpose of this project is to make a collection of simulations for various autonomous movement systems for robots that can utilize MCL, easing the burden of implementation and tuning variables with a visualizer and premade motion models to be tweaked. This project came from a place of personal utilization, as some of my other projects are drones or robots on wheels.

### Scattered Particles
<img width="1912" height="906" alt="image" src="https://github.com/user-attachments/assets/376a701c-cdd2-4ca2-9b4b-e8fd9f9d9954" />

### Estimated pose & True pose Comparison
<img width="1482" height="695" alt="image" src="https://github.com/user-attachments/assets/50697cce-a89e-4fd8-b9b5-c169da9296c6" />


## Plans for the future of this project
#### This Project currently has a decent amount of issues and things to implement for the user expirience and movement types -- here's the current list I've made:
- Configure Drone Model
- Configure Holonomic Model
- Fix Tof Sensor configuration
- Add a direct link from frontend to backend to adjust and change configurations
