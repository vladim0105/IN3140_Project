#!/usr/bin/env python

"""
This node is designed to take in a image drawing description and perform
the necessary calculations and commands to draw the image using the
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
from image_to_path import imageToPath, imageToPathAlt
from inverse_kinematics import getThetas


def inverse_kinematic(position):
    """
    Calculate the inverse kinematic of the Crustcrawler

    :param position: Desired end-point position
    :returns: Three element vector of joint angles
    """
    return getThetas(position)


def create_trajectory_point(position, seconds):
    """
    Create a JointTrajectoryPoint

    :param position: Joint positions
    :param seconds: Time from start in seconds
    :returns: JointTrajectoryPoint
    """
    point = JointTrajectoryPoint()
    point.positions.extend(position)
    point.velocities = [0.001, 0.001, 0.001]
    point.accelerations = [0.0001, 0.0001, 0.0001]
    point.effort = [10000, 10000, 10000]
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
    c = np.cos(angle)
    s = np.sin(angle)
    v = 1 - c
    kx = axis[0]
    ky = axis[1]
    kz = axis[2]
    newPath = []
    for p in path:
        newPoint = [0, 0, 0]
        newPoint[0] = (
            (kx * kx * v + c) * p[0]
            + (kx * ky * v - kz * s) * p[1]
            + (kx * kz * v + ky * s) * p[2]
        )
        newPoint[1] = (
            (kx * ky * v + kz * s) * p[0]
            + (ky * ky * v + c) * p[1]
            + (ky * kz * v - kx * s) * p[2]
        )
        newPoint[2] = (
            (kx * kz * v - ky * s) * p[0]
            + (ky * kz * v + kx * s) * p[1]
            + (kz * kz * v + c) * p[2]
        )
        newPath.append(newPoint)
    return newPath


def generate_path(image, origin, angle, axis):
    """
    VLADIMMO
    Generate path in 3D space of where to draw image
    :param origin: 3D point of image origin
    :param radius: Radius of image in centimeters
    :param num: Number of points in image
    :param angle: Angle to rotate image by
    :param axis: Unit vector to rotate image around
    :returns: List of points to draw, where a point is an array: [x, y, z]
    """
    path = []
    if args.alternative:
        path = imageToPathAlt(image, args.lift_height, args.careful, args.scale)
    else:
        path = imageToPath(image, args.lift_height, args.careful, args.scale)
    # Rotate using the rotation function
    path = rotate_path(path, angle, axis)
    # Add origin to path:
    path = [p + origin for p in path]
    return path


def generate_movement(path):
    """
    Generate Crustcrawler arm movement through a message

    :param path: List of points to draw
    :returns: FollowJointTrajectoryGoal describing the arm movement
    """
    # Generate our goal message
    movement = FollowJointTrajectoryGoal()
    # Names describes which joint is actuated by which element in the coming
    # matrices
    movement.trajectory.joint_names.extend(["joint_1", "joint_2", "joint_3"])
    # Goal tolerance describes how much we allow the movement to deviate
    # from true value at the end
    movement.goal_tolerance.extend(
        [
            JointTolerance("joint_1", 0.1, 0.0, 0.0),
            JointTolerance("joint_2", 0.1, 0.0, 0.0),
            JointTolerance("joint_3", 0.1, 0.0, 0.0),
        ]
    )
    # Goal time is how many seconds we allow the movement to take beyond
    # what we define in the trajectory
    movement.goal_time_tolerance = rospy.Duration(2)  # seconds
    time = 4.0  # Cumulative time since start in seconds
    # start_pos = inverse_kinematic([0.0, 0.0, np.pi / 2.0])
    movement.trajectory.points.append(
        create_trajectory_point([0.0, 0.0, np.pi / 2.0], time)
    )
    # First point in path will require extra time
    time += 12.0
    movement.trajectory.points.append(
        create_trajectory_point(inverse_kinematic(path[0]), time)
    )
    # The rest of the points
    for i_point in range(1, len(path)):
        prev_point = path[i_point - 1]
        point = path[i_point]
        delta = [
            point[0] - prev_point[0],
            point[1] - prev_point[1],
            point[2] - prev_point[2],
        ]
        dist = np.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
        time += dist
        print("Point (deltatime, coordinates, joint rotations):")
        print(dist)
        print(point)
        print(np.degrees(inverse_kinematic(point)))
        print("---------------------------------------")
        movement.trajectory.points.append(
            create_trajectory_point(inverse_kinematic(point), time)
        )

    # Once drawing is done we add the default position
    time += 8.0
    movement.trajectory.points.append(
        create_trajectory_point([0.0, 0.0, np.pi / 2.0], time)
    )
    return movement


def draw_image(image, origin, angle, axis):
    """
    VLADIMMO
    Draw image using Crustcrawler

    :param image: The image file to draw
    :param origin: 3D point of drawing origin
    :param angle: Angle to rotate the drawing by
    :param axis: Unit vector to rotate the drawing around
    """
    # First start by creating action client, this is responsible for passing
    # our parameters and monitoring the Crustcrawler during operations
    client = actionlib.SimpleActionClient(
        "/crustcrawler/controller/follow_joint_trajectory", FollowJointTrajectoryAction
    )
    # Generate image path
    path = generate_path(image, origin, angle, axis)
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
        print(
            "Crustcrawler failed due to: '{!s}'({!s})".format(
                result.error_string, result.error_code
            )
        )
    return result.error_code


if __name__ == "__main__":
    import argparse
    import sys

    # Create command line parser and add options:
    parser = argparse.ArgumentParser(
        description="CrustCrawler image drawer TM(C), patent pending!",
        version="Spring 2018",
    )
    parser.add_argument(
        "--image", "-img", type=str, required=True, help="Image file to draw"
    )
    parser.add_argument(
        "--lift_height",
        "-lift",
        type=int,
        required=True,
        help="Distance to lift in order to not draw",
    )
    parser.add_argument(
        "--careful",
        "-crfl",
        type=bool,
        default=False,
        help="Whether or not the robot should be careful when drawing",
    )
    parser.add_argument(
        "--alternative",
        "-alt",
        type=bool,
        default=False,
        help="Whether or not to use the alternative path generation",
    )
    parser.add_argument(
        "--scale", "-scl", type=float, default=1.0, help="The scale of the image"
    )
    parser.add_argument(
        "--origin",
        "-o",
        type=float,
        nargs=3,
        metavar=("x", "y", "z"),
        required=True,
        help="Origin of the board",
    )
    parser.add_argument(
        "--orientation",
        "-orient",
        type=float,
        default=0.0,
        metavar="degrees",
        help="Orientation of the board along the X-axis",
    )
    args = parser.parse_args()
    # Ensure points are NumPy arrays
    args.origin = np.array(args.origin)
    orient = np.array([0, 1.0, 0])
    # Ensure that arguments are within legal limits:
    if 0.0 > args.orientation or args.orientation > 90.0:
        sys.exit(
            "Orientation must be in range [0.0, 90.0], was: {:.1f}".format(
                args.orientation
            )
        )
    max_dist = np.linalg.norm(args.origin)
    # Create ROS node
    rospy.init_node("image_drawer", anonymous=True)
    # Call function to draw image
    try:
        sys.exit(
            draw_image(args.image, args.origin, np.deg2rad(args.orientation), orient)
        )
    except rospy.ROSInterruptException:
        sys.exit("Program aborted during image drawing")
