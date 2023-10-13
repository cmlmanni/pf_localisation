import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        self.NUMBER_OF_PARTICLES = 50
        self.INIT_VARIANCE = 2
        
        # ----- Set motion model parameters CHANGE THESE??
        self.ODOM_ROTATION_NOISE = 0.5  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.5  # Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.5  # Odometry y axis (side-side) noise

        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        
       
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
            particle.position.x = initialpose.position.x + (random.gauss(initialpose.position.x, self.INIT_VARIANCE) * self.ODOM_TRANSLATION_NOISE)
            particle.position.y = initialpose.position.y + (random.gauss(initialpose.position.y, self.INIT_VARIANCE) * self.ODOM_DRIFT_NOISE)
            particle.position.z = initialpose.position.z # We don't think we need to add noise to the z axis ???

            heading = getHeading(initialpose.orientation)
            random_heading = random.gauss(heading, self.INIT_VARIANCE)
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
        # Do roulette to determine how many dupes of the pose
        # instantiate new poses
        # create new posearray
        # update self.particlecloud

        pass

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

        posenum = len(self.particlecloud.poses)

        total_xyzh = [0, 0, 0, 0]

        for i in range(posenum):
            current = self.particlecloud.poses[i]
            total_xyzh[0] += current.position.x
            total_xyzh[1] += current.position.y
            total_xyzh[2] += current.position.z
            total_xyzh[3] += current.orientation

        pose = Pose()
        pose.position.x = total_xyzh[0] / posenum
        pose.position.y = total_xyzh[1] / posenum
        pose.position.z = total_xyzh[2] / posenum
        pose.orientation = rotateQuarternion(total_xyzh[3] / posenum)

        return pose
