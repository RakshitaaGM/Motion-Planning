# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from math import sqrt


class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor=10, timehor=4, goalRadiusSq=1, maxF=10):
        """
            Takes an input line from the csv file,
            and initializes the agent
        """
        self.id = int(csvParameters[0])  # the id of the agent
        self.gid = int(csvParameters[1])  # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])])  # the position of the agent
        self.vel = np.zeros(2)  # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])])  # the goal of the agent
        self.prefspeed = float(csvParameters[6])  # the preferred speed of the agent
        self.gvel = self.goal - self.pos  # the goal velocity of the agent
        self.gvel = self.gvel / (sqrt(self.gvel.dot(self.gvel))) * self.prefspeed
        self.maxspeed = float(csvParameters[7])  # the maximum sped of the agent
        self.radius = float(csvParameters[8])  # the radius of the agent
        self.goalRadiusSq = goalRadiusSq  # parameter to determine if agent is close to the goal
        self.atGoal = False  # has the agent reached its goal?
        self.ksi = ksi  # the relaxation time used to compute the goal force
        self.dhor = dhor  # the sensing radius
        self.timehor = timehor  # the time horizon for computing avoidance forces
        self.F = np.zeros(2)  # the total force acting on the agent
        self.maxF = maxF  # the maximum force that can be applied to the agent
        zeta = 0.5  # time difference
        self.Fgoal = self.gvel - self.vel / zeta  # goal force
        self.u = 0.2 # uncertainity

    def computeForces(self, neighbors=[]):
        """
            Your code to compute the forces acting on the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """
        F = np.zeros(2)
        self.F =np.zeros(2)
        print("next agent")
        if not self.atGoal:

            for neighbor in neighbors:

                print("for loop starts here")

                if self.id != neighbor.id:

                    distance = sqrt((self.pos[0] - neighbor.pos[0]) ** 2 + (
                                self.pos[1] - neighbor.pos[1]) ** 2) - self.radius - neighbor.radius
                    print("distance:")
                    print(distance)
                    print("sensing radius")
                    print(self.dhor)

                    if distance < self.dhor:
                        print("inside")

                        print('vel:')
                        print(self.vel)
                        def ttc():

                            r = (self.radius + neighbor.radius);
                            w = (self.pos - neighbor.pos);
                            c = np.dot(w, w) - r * r;

                            if (c < 0):
                                return 0

                            v = (neighbor.vel - self.vel);
                            a = np.dot(v, v) - (self.u * self.u)
                            b = np.dot(w, v) - (self.u * self.radius)
                            discr = b * b - a * c

                            if (discr) <= 0:
                                return float('inf')

                            tau = (b - sqrt(discr)) / a
                            if (tau < 0):
                                return float('inf')

                            return tau

                        tau = ttc()
                        print("time:")
                        print(tau)
                        if (tau == float('inf')):
                            print("ttc is inf")
                            F = self.Fgoal
                            print("F:")
                            print(F)
                        elif tau == 0:
                            continue

                        else:
                            k = 1
                            tau_not = 3
                            m = 2
                            r = (self.radius + neighbor.radius);
                            w = (self.pos - neighbor.pos);
                            c = np.dot(w, w) - r * r;
                            v = (neighbor.vel - self.vel);
                            a = np.dot(v, v) - (self.u * self.u)
                            b = np.dot(w, v) - (self.u * self.radius)
                            discr = b * b - a * c
                            #D = abs(b ** 2 - a ** 2 * (c ** 2 - r ** 2))
                            F = (k * np.exp(-tau / tau_not)) * (m + (tau / tau_not)) * (w + (v * tau)) / ((tau ** (m + 1)) * sqrt(discr))

                    else:
                        F = 0
                        print("outside")
                    self.F = self.F + F
                    print("force updated")
                    print(self.F)


    def update(self, dt):
        """
            Code to update the velocity and position of the agents.
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            # To cap the maximum force and maximum speed

            if (np.linalg.norm(self.F) > self.maxF):
                v = np.linalg.norm(self.F)
                self.F = self.F * (self.maxF / v)

            if (np.any(self.vel) > self.maxspeed):
                self.vel = self.maxspeed
            print("vel is updated")
            if np.all(self.F) == 0:
                self.vel = self.gvel
            self.vel += self.F * dt  # update the velocity
            self.pos += self.vel * dt  # update the position
            print("updated velocity")
            print(self.vel)
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel / sqrt(distGoalSq) * self.prefspeed


