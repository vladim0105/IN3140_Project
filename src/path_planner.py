#!/usr/bin/env python

"""
This node is designed to take in a circle drawing description and perform
the necessary calculations and commands to draw the circle using the
Crustcrawler platform
"""

from __future__ import print_function
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
import actionlib
import numpy as np
import rospy


def path_length(path):
    """
    Calculate path length in centimeters

    :param path: List of points
    :returns: Length of path in centimeters
    """
    length = 0.0
    for p1, p0 in zip(path[1:], path):
        length += np.linalg.norm(p1 - p0)
    return length


def inverse_kinematic(position):
    """
    MATTIAS
    Calculate the inverse kinematic of the Crustcrawler

    :param position: Desired end-point position
    :returns: Three element vector of joint angles
    """
    # TODO: Implement inverse kinematics function using your equations from assignment 1 task 5).
 
    pass


def create_trajectory_point(position, seconds):
    """
    Create a JointTrajectoryPoint

    :param position: Joint positions
    :param seconds: Time from start in seconds
    :returns: JointTrajectoryPoint
    """
    point = JointTrajectoryPoint()
    point.positions.extend(position)
    point.time_from_start = rospy.Duration(seconds)
    return point


def rotate_path(path, angle, axis):
    """
    DANIELA
    Rotate all elements of a path by angle-axis rotation

    :param path: List of points
    :param angle: Angle in radians to rotate by
    :param axis: Unit vector to rotate around
    :returns: List of rotated points
    """
    # TODO: Implement angle-axis rotation
    return path


def generate_path(origin, radius, num, angle, axis):
    """
    VLADIMMO
    Generate path in 3D space of where to draw circle
    :param origin: 3D point of circle origin
    :param radius: Radius of circle in centimeters
    :param num: Number of points in circle
    :param angle: Angle to rotate circle by
    :param axis: Unit vector to rotate circle around
    :returns: List of points to draw, where a point is an array: [x, y, z]
    """
    path = []
    #Distance between points in radians
    point_distance = 2*Math.pi/num
    #Plot the points from 0 to 2PI with spacing
    for rad in range(0, 2*Math.pi, point_distance):
        x = origin+radius*Math.cos(rad)
        y = origin+radius*Math.sin(rad)
        z = origin
        point = [x, y, z]
        path.append(point)

    return rotate_path(path, angle, axis)


def generate_movement(path):
    """
    MATTIAS
    Generate Crustcrawler arm movement through a message

    :param path: List of points to draw
    :returns: FollowJointTrajectoryGoal describing the arm movement
    """
    pass


def draw_circle(origin, radius, num, angle, axis):
    """
    VLADIMMO
    Draw circle using Crustcrawler

    :param origin: 3D point of circle origin
    :param radius: Radius of circle in centimeters
    :param num: Number of points in circle
    :param angle: Angle to rotate circle by
    :param axis: Unit vector to rotate circle around
    """
    # First start by creating action client, this is responsible for passing
    # our parameters and monitoring the Crustcrawler during operations
    client = actionlib.SimpleActionClient(
            '/crustcrawler/controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
    # Generate circle path
    path = generate_path(origin, radius, num, angle, axis)
    # Generate arm movement path
    goal = generate_movement(path)
    # Wait for arm to respond to action client
    client.wait_for_server()
    # Send goal
    client.send_goal(goal)
    # Wait for arm to perform our movement
    client.wait_for_result()
    # Finally print status of arm, did it work or not?
    result = client.get_result()
    if not result.error_code:
        print("Crustcrawler done!")
    else:
        print("Crustcrawler failed due to: '{!s}'({!s})"
              .format(result.error_string, result.error_code))
    return result.error_code


if __name__ == '__main__':
    import argparse
    import sys
    # Create command line parser and add options:
    parser = argparse.ArgumentParser(
            description="CrustCrawler circle drawer TM(C), patent pending!",
            version="Spring 2018")
    parser.add_argument(
            '--origin', '-o', type=float, nargs=3,
            metavar=('x', 'y', 'z'), required=True,
            help="Origin of the board")
    parser.add_argument(
            '--radius', '-r', type=float, default=5.0,
            metavar='cm', help="The radius of the circle to draw")
    parser.add_argument(
            '--num_points', '-n', type=int,
            default=4, metavar='num',
            help="Number of points to use when drawing the circle")
    parser.add_argument(
            '--orientation', '-orient', type=float, default=0.0,
            metavar='degrees',
            help="Orientation of the board along the X-axis")
    args = parser.parse_args()
    # Ensure points are NumPy arrays
    args.origin = np.array(args.origin)
    orient = np.array([0, 1., 0])
    # Ensure that arguments are within legal limits:
    if 0.0 > args.orientation or args.orientation > 90.0:
        sys.exit("Orientation must be in range [0.0, 90.0], was: {:.1f}"
                 .format(args.orientation))
    if 3 >= args.num_points <= 101:
        sys.exit("Number of points must be in range [3, 101] was: {:d}"
                 .format(args.num_points))
    max_dist = np.linalg.norm(args.origin)
    if max_dist - args.radius < 20.0:
        sys.exit("Circle to close to the robot! Minimum: 40cm, was: {:.2f}"
                 .format(max_dist - args.radius))
    # Create ROS node
    rospy.init_node('circle_drawer', anonymous=True)
    # Call function to draw circle
    try:
        sys.exit(draw_circle(args.origin, args.radius, args.num_points,
                             np.deg2rad(args.orientation), orient))
    except rospy.ROSInterruptException:
        sys.exit("Program aborted during circle drawing")
