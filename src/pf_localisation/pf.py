import random

import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from .pf_base import PFLocaliserBase
import math
import rospy

from .util import rotateQuaternion, getHeading
from random import *

from time import time


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        self.NUMBER_OF_PARTICLES = 50
        self.INIT_VARIANCE = 2
        self.EPSILON = 5
        self.NEIGHBOUR_LIM = 3

        # ----- Set motion model parameters CHANGE THESE??
        self.ODOM_ROTATION_NOISE = 0.5  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.5  # Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.5  # Odometry y axis (side-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict

    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """

        particles = []

        for i in range(self.NUMBER_OF_PARTICLES):
            particle = Pose()
            particle.position.x = initialpose.position.x + (gauss(initialpose.position.x, self.INIT_VARIANCE) * self.ODOM_TRANSLATION_NOISE)
            particle.position.y = initialpose.position.y + (gauss(initialpose.position.y, self.INIT_VARIANCE) * self.ODOM_DRIFT_NOISE)
            particle.position.z = initialpose.position.z  # We don't think we need to add noise to the z axis ???

            heading = getHeading(initialpose.orientation)
            random_heading = gauss(heading, self.INIT_VARIANCE)
            yaw = random_heading - heading

            particle.orientation = rotateQuaternion(initialpose.orientation, (yaw * self.ODOM_ROTATION_NOISE))

            particles.append(particle)

        particle_cloud = PoseArray(particles)

        return particle_cloud

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        # Loop through the cloud, for each pose get_weight
        weights = []

        for pose in self.particlecloud.poses:
            weights += self.sensor_model.get_weight(scan, pose)

        # Do roulette to determine how many dupes of the pose
        new_particles = []

        thresholds = [weights[0]]

        for i in range(1, len(weights) - 1):
            thresholds += thresholds[i-1] + weights[i]

        u = random.uniform(0, 1/len(weights))

        # instantiate new poses
        i = 0

        for j in range(len(weights) - 1):
            while u > thresholds[i]:
                i += 1

            old_p = self.particlecloud.poses[j]

            new_p = Pose()
            new_p.position.x = old_p.position.x + (gauss(old_p.position.x, self.INIT_VARIANCE) * self.ODOM_TRANSLATION_NOISE)
            new_p.position.y = old_p.position.y + (gauss(old_p.position.y, self.INIT_VARIANCE) * self.ODOM_DRIFT_NOISE)
            new_p.position.z = old_p.position.z  # We don't think we need to add noise to the z axis ???

            heading = getHeading(old_p.orientation)
            random_heading = gauss(heading, self.INIT_VARIANCE)
            yaw = random_heading - heading

            new_p.orientation = rotateQuaternion(old_p.orientation, (yaw * self.ODOM_ROTATION_NOISE))

            # add to new posearray
            new_particles.append(new_p)

            u = u + 1/len(weights)

        self.particlecloud = new_particles

        # update self.particlecloud

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

        # Attempt 3 - DBSCAN

        neighbours = []

        # Search for clusters
        for i in range(self.NUMBER_OF_PARTICLES):
            neighbours_i = []
            for j in range(i, self.NUMBER_OF_PARTICLES):
                p_i = self.particlecloud[i]
                p_j = self.particlecloud[j]

                dist = (p_i.position.x - p_j.position.x) ** 2 + (p_i.position.y - p_j.position.y) ** 2 + (p_i.position.z - p_j.position.z) ** 2

                if dist <= self.EPSILON:  # If distance to other pose is less than our set max then we have a neighbour
                    neighbours_i += [p_j]
            neighbours.append(neighbours_i)

        # Now we have to cluster the neighbours
        for k in range(self.NUMBER_OF_PARTICLES):
            for m in range(len(neighbours[k])-1):
                neighbours[k] += neighbours[m]

        # Now we find the largest cluster

        top_len = -1
        top_index = -1

        for n in range(len(neighbours) - 1):
            curr_len = len(neighbours[n])
            if curr_len > top_len:
                top_index = n
                top_len = curr_len

        # Now we take the average of the cluster

        total_xyzh = [0, 0, 0, 0]

        for i in range(top_len - 1):
            current = neighbours[top_index][i]
            total_xyzh[0] += current.position.x
            total_xyzh[1] += current.position.y
            total_xyzh[2] += current.position.z
            total_xyzh[3] += getHeading(current.orientation)

        pose = Pose()
        pose.position.x = total_xyzh[0] / top_len
        pose.position.y = total_xyzh[1] / top_len
        pose.position.z = total_xyzh[2] / top_len

        avg_heading = total_xyzh[3] / top_len
        change = getHeading(neighbours[top_index][0].orientation) - avg_heading

        pose.orientation = rotateQuaternion(neighbours[top_index][0], change)

        return pose
