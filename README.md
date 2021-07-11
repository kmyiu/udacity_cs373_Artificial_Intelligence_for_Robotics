# Artificial Intelligence for Robotics(Udacity CS373)

This repository contains the source code for ***Project - Runaway Robot***, Part 1-4

---

### Introduction of the problem statement
There is a robot lost its position and heading direction. In each time step, the robot will turn at a fixed angle and distance, then use its sensors to estimate its position. Given the measurements, estimate the next position of the robot. We solve the problem by using Extended Kalman filter (EKF) [Reference](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

----
### Part 1 Noiseless Prediction

Implementation of EKF. We set the state transition $F$ to be 

$$
 \left[
 \begin{matrix}
   1 & 0 & 1 & 0 \\
   0 & 1 & 0 & 1 \\
   0 & 0 & cos(\rho) & -sin(\rho) \\
   0 & 0 & sin(\rho) & cos(\rho) \\
  \end{matrix}
  \right]
$$

, where $\rho$ is the rotation in each step

As the robot moves in fixed turning angle and direction, we can use a pure geometry method to solve Part 1-2.

### Part 2 Adding Noise

With measurement noise, the solution is not always passing all the test case.

---

### Part 3 The Chase Begins

We chase the robot start from other position. As our movement is faster than the target, We will be sufficiently close after a finite step.

### Part 4 Chasing with a Plan

This time our movement speed is the same as the target. Instead of pointing the target directly, we estimate its position after $n$ seconds and check we are able to meet the target location within $n$ seconds. The solution is guaranteed by triangle inequality.

---
### Judging environment in Udacity
Python 2.7.6 (default, Nov 23 2017, 15:49:48), GCC 4.8.4
