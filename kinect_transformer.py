import numpy as np

import rospy
import std_msgs.msg
import tf
from geometry_msgs.msg import Point32
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud

import helpers as h
from solver.collision_checker import CollisionChecker
from solver.ik_solver import KDLIKSolver

__author__ = "Shaun Howard (smh150@case.edu)"


class KinectTransformer:
    """
    Class for the Kinect transformation and filtration system that subscribes to kinect/depth/points,
    transforms said points from kinect_pc_frame into base frame, unpacks the rgb values for all points, and
    filters two clouds for left and right arms based on color and distance from base/arm links using fwd kinematics.
    NOTE: MAKE SURE KINECT STARTS WITH BAXTER_WORLD LAUNCH!
    """

    def __init__(self):
        """
        Creates subscriber for kinect point cloud with a message queue size of 10.
        Creates publishers for left and right arm obstacles cloud, with arm filtered out and queue size of 10. These
        publishers latch for the most up-to-date message to be received upon subscription.
        Instantiates a transform listener for point cloud transformations.
        Instantiates CollisionChecker instances for both arms, utilizing the custom KDL IK solver.
        """
        rospy.init_node("kinect_transformer")
        self.kinect_depth_sub = rospy.Subscriber("kinect/depth/points", pc2.PointCloud2, self.kinect_cb, queue_size=10)
        self.left_obs_pub = rospy.Publisher("left_arm_obstacles", PointCloud, queue_size=10, latch=True)
        self.right_obs_pub = rospy.Publisher("right_arm_obstacles", PointCloud, queue_size=10, latch=True)
        self.tf = tf.TransformListener()
        self.closest_rgb_points = []
        # create collision checkers with the left and right kin solver instances
        self.left_cc = CollisionChecker([], KDLIKSolver("left"))
        self.right_cc = CollisionChecker([], KDLIKSolver("right"))

    def part_of_arm(self, point, rgb_tuple, side, collision_radius=0.175):
        """
        Checks if the given point is a part of the side arm specified.
        Uses the color provided in the rgb tuple to determine if the point is red, using both lower and upper red hues.
        Checks the point for collisions with the arm using the CollisionChecker class.
        Returns whether the point is a part of the sie arm specified based on color and collision with specified arm.
        :param point: the 3x1 point array
        :param rgb_tuple: the r,g,b tuple describing the color of the point
        :param side: the side arm to determine if point is part of
        :param collision_radius: the radius around arm links to consider in determining point containment of
        configuration space, another magic number
        :return: if the point is in fact a red point that collides with the specified arm
        """
        collides_with_arm = False
        if side == "left":
            collides_with_arm = self.left_cc.check_collision(point, collision_radius)
        elif side == "right":
            collides_with_arm = self.right_cc.check_collision(point, collision_radius)

        is_red = False
        if collides_with_arm:
            # actual_color, closest_color = h.get_color_name(rgb_tuple)
            # is_red = "red" in closest_color.lower()
            # if is_red:
            print "new arm point rgb values: "
            print rgb_tuple
        # the point is considered part of the arm if it is red and collides with the arm
        return collides_with_arm

    def filter_out_arm(self, rgb_points, side):
        """
        Filters out the points that are part of the arm for the given side.
        Returns a list of points containing only those that are thought not to represent the "side" arm.
        :param rgb_points: the (rgb_tuple, point) tuple list that represents the rgb points of environment
        :param side: the side to filter out of obstacles
        :return: the filtered 3x1 vector points list with points ONLY
        """
        # store only new points, not part of current side arm
        filtered_points = []
        # populate filtered points list with points not including the current arm based on distance and color
        for point, color in rgb_points:
            rgb_tuple = h.rgb_float_to_tuple(color)
            if not self.part_of_arm(point, rgb_tuple, side):
                filtered_points.append(point)
        return filtered_points

    def build_and_publish_obstacle_point_clouds(self, reachable_workspace_points):
        """
        Builds and publishes obstacles point clouds for both the left and right Baxter arms.
        Publishes obstacle clouds in the base from, as they are transformed and filtered points.
        :param reachable_workspace_points: the overall environment points list, filtered and transformed to base frame,
        in reachable workspace
        """
        obstacle_cloud = PointCloud()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base'
        obstacle_cloud.header = header
        left_filtered_pts = self.filter_out_arm(reachable_workspace_points, "left")
        # update collision checker obstacle list
        self.left_cc.update_obstacles(left_filtered_pts)
        for point in left_filtered_pts:
            obstacle_cloud.points.append(Point32(point[0], point[1], point[2]))
        print "publishing new left obstacle cloud!"
        self.left_obs_pub.publish(obstacle_cloud)

        obstacle_cloud = PointCloud()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base'
        obstacle_cloud.header = header
        right_filtered_pts = self.filter_out_arm(reachable_workspace_points, "right")
        # update collision checker obstacle list
        self.right_cc.update_obstacles(right_filtered_pts)
        for point in right_filtered_pts:
            obstacle_cloud.points.append(Point32(point[0], point[1], point[2]))
        print "publishing new right obstacle cloud!"
        self.right_obs_pub.publish(obstacle_cloud)

    def kinect_cb(self, data, source="kinect_pc_frame", dest="base", min_dist=0.1, max_dist=1.45, min_height=-1):
        """
        Receives kinect points from the kinect subscriber linked to the publisher stream.
        Important notes for unpacking floats: http://www.pcl-users.org/How-to-extract-rgb-data-from-PointCloud2-td3203403.html
        Format of PointCloud2 msg is X, Y, Z, RGB -- 4 floating point numbers.
        pc2.read_points handles this for us.
        The 4th value must be unpacked to determine the actual RGB color.
        NOTE: min_height is a magic number that is the z value at the wheel base of the Baxter simulator robot.
        """
        # TODO add left and right arm points to filter out of published "obstacles" per side
        points = np.array([p for p in pc2.read_points(data, skip_nans=True)])
        transformed_points = h.transform_pcl2(self.tf, dest, source, points)
        transformed_points = [h.point_to_3x1_vector(p) for p in transformed_points]
        points_list = []
        # extract only colors, in order, from points list
        colors = points[:, 3]
        i = 0
        # construct tuples of format: (dist_to_robot, point, rgb_color)
        # filter points by parameter criteria
        for p in transformed_points:
            dist = np.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)
            if min_dist < dist < max_dist and p[2] >= min_height:
                points_list.append((dist, p, colors[i]))
            i += 1
        # sort points by distance from robot base
        # TODO sorted_pts = np.sort(points_list, 0)
        sorted_pts = points_list
        # extract only the closest points, in order of dist from robot base, and add color and point vector to list
        self.closest_rgb_points = [(point, color) for dist, point, color in sorted_pts]
        self.build_and_publish_obstacle_point_clouds(self.closest_rgb_points)
        if len(self.closest_rgb_points) > 0:
            print "kinect cb: there are this many close points: " + str(len(self.closest_rgb_points))

if __name__ == "__main__":
    KinectTransformer()
    while not rospy.is_shutdown():
        rospy.spin()
