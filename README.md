# Distributed-Gradient-Seeking
Project for the "Distributed Estimation for Robots and Vehicles" course for the Mechatronics Engineering master degree @ University of Trento

### Abstract
This project explores the design and implementation of a distributed system for autonomous navigation, where a  ground vehicle aims to move from an initial location to a destination point at the center of a scalar field following the gradient of the field. The ground vehicle is equipped with an Extended Kalman Filter (EKF) to estimate its position because it has no direct knowledge of its true position. The measurement model consists of estimates of its true position derived by a network of drones via Time-Difference-Of-Arrival (TDOA) measurements. By receiving the signal emitted by the ground vehicle at each time instant and by collecting information from its neighbors, each drone is able to compute locally the TDOA measurements and, using them in its own EKF, it estimates the position of the vehicle. To ensure a robust and accurate estimation, the project introduces a weighted average consensus algorithm. This algorithm gives more weight to drones with a larger number of measurements, thus improving the reliability of the collective estimates. The consensus process propagates these weighted estimates through the drone network, eventually leading to a global agreement on the position of the ground vehicle. This overall estimate is then communicated to the ground vehicle, which integrates it into its EKF for continuous position updating and navigation. The distributed nature of the system, combined with the consensus algorithm, ensures that the ground vehicle can reliably and precisely navigate to its target point, even in the presence of uncertainties and without direct knowledge of the position.


The project is built as follow : 
```
classes/
├── AGENT.m             // ROBOT class
├── DRONE.m             // DRONE class
doc/                    // useful papers used for the project
├── ...
functions/
├── agent               // matlab function for the robot/agent class
├── drone               // matlab function for the drone class
├── others              // others useful matlab function
images/                 // images for the report
├── ...
init.m                  // matlab file for initialization
main_Matveev.m          // MAIN Matlab file for the simulation
parameters.m            // matlab file with the simulation parameters
README.md
```



<img src="/images/ROBOT_est_vs_real_path_ellipses.svg" alt="ROBOT_est_vs_real_path_ellipses" width="500"/>

<img src="/images/unicycle_robot_gradient_descent.gif" alt="Description of Image" width="500"/>