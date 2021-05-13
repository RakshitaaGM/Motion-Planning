# agent.py
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
import math


class Agent(object):

    def __init__(self, csvParameters, dhor=10, goalRadiusSq=1):
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
        self.dhor = dhor  # the sensing radius
        self.vnew = np.zeros(2)  # the new velocity of the agent
        self.candidatevelocity = [[0], [0]]

    def computeNewVelocity(self, neighbors=[]):
        """
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
        cost1 = []
        cv1 = []
        minpos = 0
        N = float(0)
        AV = self.maxspeed
        dist = sqrt((self.pos[0] - self.goal[0]) ** 2 + (self.pos[1] - self.goal[1]) ** 2) - self.radius

        if dist > 1:

            for neighbor in neighbors:

                if self.id != neighbor.id:

                    distance = sqrt((self.pos[0] - neighbor.pos[0]) ** 2 + (
                                self.pos[1] - neighbor.pos[1]) ** 2) - self.radius - neighbor.radius

                    if (distance < self.dhor) and (self.pos[0] != neighbor.pos[0]) and (self.pos[1] != neighbor.pos[1])  :

                        N = 1000;
                        step = (2 * math.pi - 0) / N
                        cost1 = []

                        # Sampling - Using the circle method

                        for delta in np.arange(0, 2 * math.pi, step):

                            Vx = sqrt(AV) * math.cos(delta)
                            Vy = sqrt(AV) * math.sin(delta)
                            self.candidatevelocity = ([Vx, Vy])
                            cv1.append(self.candidatevelocity)

                            def ttc(candidatevelocity):

                                r = (self.radius + neighbor.radius);
                                w = (self.pos - neighbor.pos);
                                c = np.dot(w, w) - r * r;

                                if (c < 0):
                                    return 0

                                v = (neighbor.vel - candidatevelocity);
                                a = np.dot(v, v)
                                b = np.dot(w, v)
                                discr = b * b - a * c

                                if (discr) <= 0:
                                    return float('inf')

                                tau = (b - sqrt(discr)) / a

                                if (tau < 0):
                                    return float('inf')

                                return tau

                            tau = ttc(self.candidatevelocity)
                            alpha1 = 100;
                            beta = 200;
                            gamma = 1000;
                            v1 = np.linalg.norm(self.candidatevelocity - self.gvel)
                            v2 = np.linalg.norm(self.candidatevelocity - self.vel)
                            cost = ((alpha1 * v1) + beta * (v2) + (gamma / (tau + 1e-100)))
                            cost1.append(cost)

                        minpos = cost1.index(min((cost1)))
                        self.vnew[:] = cv1[minpos]
                else:
                    self.vnew[:] = self.gvel[:]

        else:
            self.atGoal = True

            #   # here I just set the new velocity to be the goal velocity

    def update(self, dt):
        """
            Code to update the velocity and position of the agent
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel * dt  # update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel / sqrt(distGoalSq) * self.prefspeed



