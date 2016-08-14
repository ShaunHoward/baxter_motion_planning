import numpy as np
import struct

import rospy
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped
)
from std_msgs.msg import Header

from baxter_kdl.kdl_kinematics import KDLKinematics as kin
from urdf_parser_py.urdf import URDF
import helpers as h
from hrl_geom.pose_converter import PoseConv

__author__ = "Shaun Howard (smh150@case.edu)"


def get_min_joints():
        return np.array([-3.059, -1.57079632679, -3.059, -0.05, -3.05417993878, -2.147, -1.70167993878])


def get_max_joints():
        return np.array([3.059, 2.094, 3.059, 2.618, 3.05417993878, 1.047, 1.70167993878])


class KDLIKSolver:
    def __init__(self, limb):
        self._baxter = URDF.from_parameter_server(key='robot_description')
        self._base_link = "base"
        self._tip_link = limb + '_gripper'
        self.solver = kin(self._baxter, self._base_link, self._tip_link)
        self.min_joins = get_min_joints()
        self.max_joints = get_max_joints()

    def solve(self, pose, use_rr):
        x_positions = h.pose_to_7x1_vector(pose)
        position = x_positions[:3]
        orientation = x_positions[3:]
        if not use_rr:
            return self.solver.inverse(position, orientation)
        else:
            return self.solver.inverse_search(position, orientation, timeout=2.)
            # return self.solver.inverse_biased_search(pose, goal_q_bias, goal_weights)

    def solve_biased_random_restarts(self, position, orientation):
        return self.solver.inverse_biased_search(position, orientation)

    def solve_fwd_kin(self, q_list):
        return self.solver.forward(q_list)

    def joint_fwd_kin(self, q_list, base_link, end_link):
        return self.solver.forward_2(q_list, base_link, end_link)

    def fwd_kin_all(self, q_list):
        return self.solver.forward_all(q_list)

    def jacobian(self, q_list):
        return self.solver.jacobian(q_list)

    def jacobian_transpose(self, q_list):
        return self.solver.jacobian(q_list).T

    def jacobian_pinv(self, q_list):
        return np.linalg.pinv(self.solver.jacobian(q_list))

    def joints_in_limits(self, q_list):
        return self.solver.joints_in_safe_limits(q_list).all()

    def all_joints_at_limits(self, q_list):
        return self.solver.all_joints_at_limits(q_list).all()

    def clip_joints_to_limits(self, q_list):
        return self.solver.clip_joints_safe(q_list)


class RethinkIKSolver:

    def __init__(self):
        self.r_ns = "ExternalTools/right/PositionKinematicsNode/IKService"
        self.r_iksvc = rospy.ServiceProxy(self.r_ns, SolvePositionIK)

        self.l_ns = "ExternalTools/left/PositionKinematicsNode/IKService"
        self.l_iksvc = rospy.ServiceProxy(self.l_ns, SolvePositionIK)

    def solve(self, limb, pose):
        # pose = h.pose_vector_to_pose_msg(pose_arr)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        pose = PoseStamped(header=hdr, pose=pose)
        ikreq.pose_stamp.append(pose)
        if limb is "left":
            ns = self.l_ns
            iksvc = self.l_iksvc
        else:
            ns = self.r_ns
            iksvc = self.r_iksvc
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return None

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int'
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if resp_seeds[0] != resp.RESULT_INVALID:
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            rospy.loginfo("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(seed_str))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo("IK Joint Solution:\n{0}".format(limb_joints))
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return None
        return limb_joints
