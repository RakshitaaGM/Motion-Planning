# Motion-Planning
This Repository contains the Assignments and Projects that were completed as a part of the Motion Planning course at CUICAR (Spring '20)

## Project 1 - Sampling Based Local Navigation
Implemented Sampling based velocity approach for three and eight multi agent navigation scenarios. an excel sheet is provided for each of the scenarios which has the followinf parameters respectively:
agent_id, group_id, start_position_x, start_position_y, goal_position_x, goal_position_y, preferred_speed, maximum_speed, radius
The program does the following:
- load a simulation scenario and set the simulationtime step, delta t, to 0.1 s
- computes new velocity for each of the agent
- Determins the neighbouring agents which are present within a distance less than sensing radius, dh.
- Samples a velocity within the admissible space. Candidate velcoities are chosen between 100-1000.
- Uses the following cost function:
 ![image](https://user-images.githubusercontent.com/59737146/118138097-eff1ba80-b3d3-11eb-85a3-73aca6cd70e2.png)
- Calculates time to collision
- updates cadidate velocity which has less cost function value
The environment looks like this:
![image](https://user-images.githubusercontent.com/59737146/118138903-d9982e80-b3d4-11eb-8832-c3b98fc56485.png)

## Project 2 - Local Navigation with TTC forces

## Project 3 - Discrete Planning - A Star 

## Project 4 - Sampling Based Navigation

## Final Project - Motion Planning for Dubins Car using RRT 
