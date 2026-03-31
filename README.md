# odom_motion_model
This project implements the probabilistic odometry motion model for mobile robots based on noisy control inputs (rot1, trans, rot2). The model is integrated within a ROS2 simulation using the Articulated Robot (ArticubotOne) in Gazebo.

The implementation focuses on accurately modeling motion uncertainty as a foundation for localization and SLAM algorithms.

<img width="1920" height="1080" alt="Screenshot from 2026-03-31 17-21-11" src="https://github.com/user-attachments/assets/75ca8640-d872-4fcd-8ff6-a6a686eee217" />

<img width="1920" height="1080" alt="Screenshot from 2026-03-31 17-20-56" src="https://github.com/user-attachments/assets/d3199ff1-4c86-4daf-b36e-ee35d02eb1e4" />


## Simulation Setup
All algorithms are implemented and tested using:
- ROS2 (Humble)
- Gazebo
- ArticubotOne robot (based on John Evan’s tutorial)
