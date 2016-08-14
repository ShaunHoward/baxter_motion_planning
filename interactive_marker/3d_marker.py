#!/usr/bin/env python
import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

__author__ = "Shaun Howard (smh150@case.edu)"


counter = 0
server = None


def frameCallback(msg):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform((0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base", "moving_frame")
    counter += 1


def processFeedback(feedback):
    rospy.loginfo(" ".join(["marker at x y z:", str(feedback.pose.position.x), str(feedback.pose.position.y),
                  str(feedback.pose.position.z)]))


def alignMarker(feedback):
    pose = feedback.pose
    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5
    rospy.loginfo(feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) +
                  "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                  str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z))
    server.setPose(feedback.marker_name, pose)
    server.applyChanges()


def rand(min_, max_):
    return min_ + random()*(max_-min_)


def makeBox(msg):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def saveMarker(int_marker):
  server.insert(int_marker, processFeedback)


def make6DofMarker(side, fixed, interaction_mode, position_, show_6dof=False, frame="base", size=0.5):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame
    int_marker.pose.position = position_
    int_marker.scale = size

    int_marker.name = side + "_marker"
    int_marker.description = side + "_control"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof:
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    server.insert(int_marker, processFeedback)


if __name__ == "__main__":
    rospy.init_node("merry_end_point_markers")
    br = TransformBroadcaster()

    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)
    server = InteractiveMarkerServer("merry_end_point_markers")

    # make left endpoint marker
    position = Point(0.672248005867, 0.485744655132, 0.0593262910843)
    make6DofMarker("left", False, InteractiveMarkerControl.NONE, position, True)

    # make right endpoint marker
    position = Point(0.672248005867, -0.485744655132, 0.0593262910843)
    make6DofMarker("right", False, InteractiveMarkerControl.NONE, position, True)

    server.applyChanges()
    rospy.loginfo("marker is running in the background")
    rospy.spin()
