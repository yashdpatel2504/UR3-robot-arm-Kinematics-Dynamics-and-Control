# UR3 robot arm: Kinematics, Dynamics and Control

UR3 is a 6-degree-of-freedom (DoF) robotic manipulator manufactured by Universal Robots Company.

The robot arm kinematic model is built according to the Denavit-Hartenberg convention. A Decentralized PID Controller and a Centralized Inverse Dynamics Controller was designed. Finally, a trajectory is performed and the performance/results of both controllers compared and analysed.



## Results



### Conclusion

As this report demonstrates, **the dynamic model, with the Newton-Euler formulation, was successfully built and validated** by two tests. One with the manipulator arm at rest and another by making the manipulator arm behave as a gravitational pendulum. Regarding the controllers, both were successfully designed and implemented. The decentralized controller wasn't as successful as the centralized one for high velocity trajectories and also as a higher position and pose errors, but more significant in the position one. From this, **the best controller is the centralized one**.
