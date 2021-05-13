# Motion-Planning
This Repository contains the Assignments and Projects that were completed as a part of the Motion Planning course at CUICAR (Spring '20)

## Project 1 - Sampling Based Local Navigation
Implemented Sampling based velocity approach for three and eight multi agent navigation scenarios. an excel sheet is provided for each of the scenarios which has the followinf parameters respectively:
agent_id, group_id, start_position_x, start_position_y, goal_position_x, goal_position_y, preferred_speed, maximum_speed, radius
The program does the following:
- Loads a simulation scenario and set the simulationtime step, delta t, to 0.1 s
- Computes new velocity for each of the agent
- Determins the neighbouring agents which are present within a distance less than sensing radius, dh.
- Samples a velocity within the admissible space. Candidate velcoities are chosen between 100-1000.
- Uses the following cost function:

 ![image](https://user-images.githubusercontent.com/59737146/118138097-eff1ba80-b3d3-11eb-85a3-73aca6cd70e2.png)
- Calculates time to collision
- Updates cadidate velocity which has less cost function value
The environment looks like this:
![image](https://user-images.githubusercontent.com/59737146/118138903-d9982e80-b3d4-11eb-8832-c3b98fc56485.png)

 ![image](https://user-images.githubusercontent.com/59737146/118147100-3dbef080-b3dd-11eb-8409-8a1c14b2d35d.png)

## Project 2 - Local Navigation with TTC forces
Implement the predictive TTC forces approach for local navigation.The approach can be considered as a variant of the PowerLaw model, as both approaches rely on forces that depend on the relative displacement of agents at the moment of a collision.
The program does the following:
- Load a simulation scenario and set the time step delta t to be 0:05 s
- Computes a collection of forces that determine the behavior of the agent at each simulation step, and then update its position and velocity for each of the n agents.
- The following formula was used to determine the goal force:

![image](https://user-images.githubusercontent.com/59737146/118146550-b8d3d700-b3dc-11eb-8d4a-e9fa03854d8b.png)
- Determines all neighbors of the agent that are less than dH m away from the agent
- Estimates the time to collision value for each of the neighbours.
- Adds a repulsive force if collision is detected. the formula used for repulsive force:

![image](https://user-images.githubusercontent.com/59737146/118146857-03edea00-b3dd-11eb-835e-8efd35b0b04e.png)
- Updates the velocities and positions of the agents based on the computed forces by using simple Euler integration
 
## Project 3 - Discrete Planning - A Star 

## Project 4 - Sampling Based Navigation

## Final Project - Motion Planning for Dubins Car using RRT 


# Licensing Information:  
You are free to use or extend these projects for educational purposes provided that (1) you do not distribute or publish solutions, (2) you retain this notice, and (3) you provide clear attribution to Clemson University and the author.

### Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
### Modified by : Rakshitaa Geetha Mohan
Source: Clemson University
