import argparse
import copy
import math
import numpy as np
import random
import rospy
import struct
import webcolors


from geometry_msgs.msg import (
    Pose,
    Point,
    Twist
)

from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.preprocessing import scale

from tf.transformations import euler_from_quaternion

__author__ = 'Shaun Howard (smh150@case.edu)'


def closest_color(requested_colour):
    min_colours = {}
    for key, name in webcolors.css3_hex_to_names.items():
        r_c, g_c, b_c = webcolors.hex_to_rgb(key)
        rd = (r_c - requested_colour[0]) ** 2
        gd = (g_c - requested_colour[1]) ** 2
        bd = (b_c - requested_colour[2]) ** 2
        min_colours[(rd + gd + bd)] = name
    return min_colours[min(min_colours.keys())]


def get_color_name(requested_colour):
    try:
        closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
    except ValueError:
        closest_name = closest_color(requested_colour)
        actual_name = None
    return actual_name, closest_name


def point_to_3x1_vector(point):
    """
    Converts a Point with x,y,z to a 3x1 numpy array.
    :param point: the Point object with x,y,z attributes
    :return: the 3x1 numpy array of the Point
    """
    return np.array((point.x, point.y, point.z))


def pose_to_7x1_vector(pose):
    """
    Converts a Pose to a 7x1 numpy array.
    :param pose: the Pose to convert to 7x1 array
    :param pose:
    :return:
    """
    return np.array((pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                     pose.orientation.y, pose.orientation.z, pose.orientation.w))


def get_euler_angles_for_orientation(goal_orientation):
    """
    Converts a given orientation np array into an np array of the equivalent euler angles
    in order yaw, pitch, roll
    :param goal_orientation: the orientation np.array
    :return: the np.array yaw, pitch roll euler angles
    """
    return np.array(euler_from_quaternion(goal_orientation))


def generate_random_decimal(start=0.00001, stop=0.5, decimal_places=5):
    """
    Generates a uniformly random decimal value between start and stop inclusive with the provided
    number of decimal places.
    :param start: the start number for the random range
    :param stop: the stop number for the random range
    :param decimal_places: the number of decimal places for the result to have
    :return: the rounded randomly generated number in between start and stop
    """
    return round(random.uniform(start, stop), decimal_places)


def generate_random_3d_point_at_length_away(center_pt, dist):
    """
    Generates a random 3d point using the generalized solution of picking a random point uniformly distributed
    on the unit sphere. The given distance is in meters.
    :param center_pt: point to generate random point the given dist from
    :param dist: the distance in meters to generate a random point from the given point
    :return: a random point generated along the generalized sphere at the dist away from the given point
    """
    z = random.uniform(-1, 1)
    theta = random.uniform(0, 2*math.pi)
    r = math.sqrt(1-(z*z))
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    x_rand = center_pt[0] + dist * x
    y_rand = center_pt[1] + dist * y
    z_rand = center_pt[2] + dist * z
    return np.array((x_rand, y_rand, z_rand))


def unwrap_angles_dict_to_list(angles_dict, keys):
    return [angles_dict[key] for key in keys] if angles_dict is not None else None


def wrap_angles_in_dict(angles, keys):
    """
    Creates a dictionary from the provided ordered angles list corresponding to the provided keys name list.
    The mapping is constructed like so: for i in len(keys): q_dict[keys[i]] = angles[i].
    :param angles: the ordered angles list corresponding to keys list of names
    :param keys: the ordered list of keys from base to end effector
    :return: the mapping from keys to angles in a dictionary
    """
    q_dict = dict()
    for i in range(len(keys)):
        q_dict[keys[i]] = angles[i]
    return q_dict


def generate_goal_pose_w_same_orientation(dest_point, endpoint_orientation):
    """
    Creates a Pose from the 3x1 destination point np array and Pose.orientation data object as endpoint_orientation.
    If endpoint_orientation is None, a partially-filled Pose will be returned with only x,y,z populated.
    """
    ik_pose = Pose()
    ik_pose.position.x = dest_point[0]
    ik_pose.position.y = dest_point[1]
    ik_pose.position.z = dest_point[2]
    o = endpoint_orientation
    if o is not None:
        ik_pose.orientation.x = o.x
        ik_pose.orientation.y = o.y
        ik_pose.orientation.z = o.z
        ik_pose.orientation.w = o.w
    return ik_pose


def get_args():
    """
    Handles argument parsing and gets the startup arguments. No defaults are set here. They are set in the Merry
    class constructor.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=""""
                                     An interface to the CWRU robotics Merry robot for potential field motion planning.
                                     For help on usage, refer to the github README @ github.com/ShaunHoward/potential_\
                                     fields.""")
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=False, choices=['left', 'right'],
        help='limb to record/playback waypoints'
    )
    parser.add_argument(
        '-s', '--speed', type=float,
        help='joint position motion speed ratio [0.0-1.0] (default=0.5)'
    )
    parser.add_argument(
        '-a', '--accuracy', type=float,
        help='joint position accuracy (rad) at which waypoints must achieve'
    )
    return parser.parse_args(rospy.myargv()[1:])


def get_critical_points_of_obstacles(merry):
    """
    Considers the closest point the obstacle currently.
    In the future, will use obstacle detection to find the critical points of obstacles nearest to the robot.
    :return: a list of critical points paired with distances to end effector for each of the nearest obstacles
    """
    closest_point = None
    closest_dist = 15

    if merry.closest_points is not None and len(merry.closest_points) > 0:
        # copy points so they're not modified while in use
        curr_points = copy.deepcopy(merry.closest_points)
        normalized_points = scale(curr_points)
        if not merry.kmeans_initialized:
            predicition = merry.kmeans.fit(normalized_points)
            merry.kmeans_initialized = True
        else:
            prediction = merry.kmeans.predict(normalized_points)

        print(prediction)
        # run obstacle detection (K-means clustering) on the points closest to the robot
        # for p in closest_points:
        #     dist = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
        #     if closest_point is None or dist < closest_dist:
        #         closest_point = p
        #         closest_dist = dist
    return None if not (closest_point and closest_dist) else [(closest_point, closest_dist)]


def pose_vector_to_pose_msg(pose_vector):
    """
    Converts an ordered pose angle vector to a Pose.
    Mapping is 0,1,2 -> x,y,z, 3,4,5,6 -> q.x,q.y,q.z,q.w,
    this will created a 3-d pose if only 0,1,2 are populated, otherwise if 7 values are present,
    a 7-d pose will be created.
    :param pose_vector: the ordered angle vector of either x,y,z or x,y,z,q.x,q.y,q.z,q.w set
    :return: the pose, either 3-d or 7-d depending on if q_list is length 3 or length 7
    """
    pose = Pose()
    if len(pose_vector) == 7:
        return get_pose(pose_vector[0], pose_vector[1], pose_vector[2], pose_vector[3], pose_vector[4], pose_vector[5],
                        pose_vector[6])
    elif len(pose_vector) >= 2:
        return get_pose(pose_vector[0], pose_vector[1], pose_vector[2])
    return pose


def get_pose(x, y, z, ox=0, oy=0, oz=0, ow=1):
    """
    Creates a pose using x,y,z,ox,oy,oz,ow values.
    Default ox,oy,oz,ow values are 0,0,0,1.
    :param x: x position
    :param y: y position
    :param z: z position
    :param ox: x quaternion
    :param oy: y quaternion
    :param oz: z quaternion
    :param ow: w quaternion
    :return:
    """
    pm = Pose()
    pm.position.x = x
    pm.position.y = y
    pm.position.z = z
    pm.orientation.x = ox
    pm.orientation.y = oy
    pm.orientation.z = oz
    pm.orientation.w = ow
    return pm


def get_current_endpoint_pose(arm):
    """
    Returns the Pose of the provided arm endpoint.
    :param arm: the Baxter Interface Limb to get the endpoint_pose() from
    :return: the Pose msg of the provided arm
    """
    # retrieve current pose from endpoint
    current_pose = arm.endpoint_pose()
    pose_msg = Pose()
    pose_msg.position.x = current_pose['position'].x
    pose_msg.position.y = current_pose['position'].y
    pose_msg.position.z = current_pose['position'].z
    pose_msg.orientation.x = current_pose['orientation'].x
    pose_msg.orientation.y = current_pose['orientation'].y
    pose_msg.orientation.z = current_pose['orientation'].z
    pose_msg.orientation.w = current_pose['orientation'].w
    return pose_msg


def get_current_endpoint_velocities(arm):
    """
    Returns the Twist msg of the provided arm endpoint.
    :param arm: the Baxter Interface Limb to get the endpoint_velocity() from
    :return: the Twist msg of the provided arm
    """
    current_vels = arm.endpoint_velocity()
    vel_msg = Twist()
    vel_msg.linear.x = current_vels['linear'].x
    vel_msg.linear.y = current_vels['linear'].y
    vel_msg.linear.z = current_vels['linear'].z
    vel_msg.angular.x = current_vels['angular'].x
    vel_msg.angular.y = current_vels['angular'].y
    vel_msg.angular.z = current_vels['angular'].z
    return vel_msg


def get_kmeans_instance(num_clusts=10):
    """Creates and returns a scikit learn Kmeans clusterer instance with the provided number of clusters."""
    # seed numpy with the answer to the universe
    np.random.seed(42)
    kmeans = KMeans(init='k-means++', n_clusters=num_clusts, n_init=num_clusts)
    return kmeans


def lookup_transform(tf_, from_frame, to_frame):
    """"
    Looks up a transform from the from_frame to the to_frame.
    Checks if the frame to_frame exists and if the frame from_frame exists.
    Then, checks for the latest update time and gets the position and quaternion transforms
    from the from_frame to the to_frame.
    :param from_frame: the frame the msg starts at
    :param to_frame: the frame the msg needs to transform to
    :return: the position and quaternion for the transform from_frame to to_frame, or None for both if the
    transform frames do not exist
    """
    position = None
    quaternion = None
    if tf_.frameExists(to_frame) and tf_.frameExists(from_frame):
        t = tf_.getLatestCommonTime(to_frame, from_frame)
        position, quaternion = tf_.lookupTransform(to_frame, from_frame, t)
    return position, quaternion


def as_matrix2(tf_, target_frame, source_frame):
    """
    Transforms the translation and rotation from the Pose msg to a 4x4 matrix
    from the from_frame to the to_frame.
    :param tf_: the TransformListener instance to use for the conversion
    :param target_frame: the frame to convert to
    :param source_frame: the frame to convert from
    :return: a 4x4 matrix describing the rotation of the end effector
    """
    return tf_.fromTranslationRotation(*lookup_transform(tf_, source_frame, target_frame))


def transform_pcl2(tf_, target_frame, source_frame, point_cloud, duration=2):
    """
    Transforms the given point_cloud from the source_frame to the target_frame using a custom
    implementation of the pcl2 transform_pcl2 using a TransformListener instance tf_. The
    transform is sought over the provided duration number of seconds.
    :param tf_: the TransformListener instance
    :param target_frame: the frame to convert PointCloud to
    :param source_frame: the frame to convert PointCloud from
    :param point_cloud: the PointCloud to transform
    :param duration: the number of seconds to capture a transformation over
    :return: the List of Points with x,y,z extracted, transformed and loaded from the provided point_cloud
    """
    # returns a list of transformed Points
    tf_.waitForTransform(target_frame, source_frame, rospy.Time.now(), rospy.Duration(duration))
    mat44 = as_matrix2(tf_, target_frame, source_frame)
    if point_cloud.any():
        return [Point(*(tuple(np.dot(mat44, np.array([p[0], p[1], p[2], 1])))[:3])) for p in point_cloud]
    else:
        return []


def rgb_float_to_tuple(color):
    """
    Converts rgb float color to a tuple using bit-shifts.
    First packs the float to obtain bits, then performs shifts to get 3 rgb values.
    :param color: the float number to get rgb values of
    :return: the rgb values for given color float
    """
    rgb_bits = float_to_bits(color)
    return ((rgb_bits >> 16) & 0xff), ((rgb_bits >> 8) & 0xff), (rgb_bits & 0xff)


# http://stackoverflow.com/questions/14431170/get-the-bits-of-a-float-in-python
def float_to_bits(f):
    s = struct.pack('>f', f)
    return struct.unpack('>l', s)[0]


# found helpers for color conversions at:
# http://code.activestate.com/recipes/576919-python-rgb-and-hsv-conversion/
def hsv2rgb(h, s, v):
    h = float(h)
    s = float(s)
    v = float(v)
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    r, g, b = 0, 0, 0
    if hi == 0:
        r, g, b = v, t, p
    elif hi == 1:
        r, g, b = q, v, p
    elif hi == 2:
        r, g, b = p, v, t
    elif hi == 3:
        r, g, b = p, q, v
    elif hi == 4:
        r, g, b = t, p, v
    elif hi == 5:
        r, g, b = v, p, q
    r, g, b = int(r * 255), int(g * 255), int(b * 255)
    return r, g, b


def rgb2hsv(r, g, b):
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    df = mx - mn
    h = 0
    if mx == mn:
        h = 0
    elif mx == r:
        h = (60 * ((g - b) / df) + 360) % 360
    elif mx == g:
        h = (60 * ((b - r) / df) + 120) % 360
    elif mx == b:
        h = (60 * ((r - g) / df) + 240) % 360
    if mx == 0:
        s = 0
    else:
        s = df / mx
    v = mx
    return h, s, v

# def move_to_start(self, start_angles=None):
#     print("Moving the {0} arm to start pose...".format(self._limb))
#     if not start_angles:
#         start_angles = dict(zip(self.JOINT_NAMES[:7], [0]*7))
#     self.set_joint_velocities(start_angles)
#     # self.gripper_open()
#     rospy.sleep(1.0)
#     print("Running. Ctrl-c to quit")

#
# def generate_goal_velocities(self, goal_point, obs, side="right"):
#     """
#     Generates the next move in a series of moves using an APF planning method
#     by means of the current robot joint states and the desired goal point.
#     :param goal_point: the goal 3-d point with x, y, z fields
#     :param obs: the obstacles for the planner to avoid
#     :param side: the arm side of the robot
#     :return: the status of the op and the goal velocities for the arm joints
#     """
#     return planner.plan(self.bkin, self.generate_goal_pose(goal_point), obs, self.get_current_endpoint_velocities(),
#                         self.get_joint_angles(side), side)
#
# def set_joint_velocities(self, joint_velocities):
#     """
#     Moves the Baxter robot end effector to the given dict of joint velocities keyed by joint name.
#     :param joint_velocities:  a list of join velocities containing at least those key-value pairs of joints
#      in JOINT_NAMES
#     :return: a status about the move execution; in success, "OK", and in error, "ERROR".
#     """
#     status = "OK"
#     rospy.sleep(1.0)
#
#     # Set joint position speed ratio for execution
#     self._limb.set_joint_position_speed(self._speed)
#
#     # execute next move
#     if not rospy.is_shutdown() and joint_velocities is not None and len(joint_velocities) == len(JOINT_NAMES):
#         vel_dict = dict()
#         curr_vel = 0
#         for name in JOINT_NAMES:
#             vel_dict[name] = joint_velocities[curr_vel]
#             curr_vel += 1
#         self._limb.set_joint_velocities(vel_dict)
#     else:
#         status = ERROR
#         rospy.logerr("Joint velocities are unavailable at the moment. \
#                       Will try to get goal velocities soon, but staying put for now.")
#
#     # Set joint position speed back to default
#     self._limb.set_joint_position_speed(0.3)
#     return status
