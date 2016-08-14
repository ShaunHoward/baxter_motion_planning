from functools import partial

from ompl import base as ob
from ompl import control as oc
from ompl import geometric as og
from ompl.geometric import RRTstar
from solver.ik_solver import KDLIKSolver, get_min_joints, get_max_joints

x = 0
y = 1
z = 2
ox = 3
oy = 4
oz = 5
ow = 6


def is_state_valid(space_info, state):
    return space_info.satisfiesBounds(state)


def propagator(q, u, q_dot):
    q_dot = q + u
    return q_dot


def get_min_max_joint_limits(side):
    kdl = KDLIKSolver(side)
    min_fwd = kdl.solve_fwd_kin(get_min_joints())
    max_fwd = kdl.solve_fwd_kin(get_max_joints())
    return min_fwd, max_fwd


def setup_bounds(side):
    space = ob.SE3StateSpace()
    bounds = ob.RealVectorBounds(7)
    bounds.setLow(x, 0)
    bounds.setHigh(x, 1.5)
    if side == "left":
        y_low = -1.5
        y_high = 0
    else:
        y_low = 0
        y_high = 1.5
    bounds.setLow(y, y_low)
    bounds.setHigh(y, y_high)
    bounds.setLow(z, -1)
    bounds.setHigh(z, 1)
    min_fwd, max_fwd = get_min_max_joint_limits(side)
    bounds.setLow(ox, min_fwd[ox])
    bounds.setHigh(ox, max_fwd[ox])
    bounds.setLow(oy, min_fwd[oy])
    bounds.setHigh(oy, max_fwd[oy])
    bounds.setLow(oz, min_fwd[oz])
    bounds.setHigh(oz, max_fwd[oz])
    bounds.setLow(ow, min_fwd[ow])
    bounds.setHigh(ow, max_fwd[ow])
    space.setBounds(bounds)

    c_space = oc.RealVectorControlSpace(space, 7)
    c_bounds = ob.RealVectorBounds(7)
    c_bounds.setLow(x, min_fwd[x])
    c_bounds.setHigh(x, max_fwd[x])
    c_bounds.setLow(y, min_fwd[y])
    c_bounds.setHigh(y, max_fwd[y])
    c_bounds.setLow(z, min_fwd[z])
    c_bounds.setHigh(z, max_fwd[z])

    c_bounds.setLow(ox, min_fwd[ox])
    c_bounds.setHigh(ox, max_fwd[ox])
    c_bounds.setLow(oy, min_fwd[oy])
    c_bounds.setHigh(oy, max_fwd[oy])
    c_bounds.setLow(oz, min_fwd[oz])
    c_bounds.setHigh(oz, max_fwd[oz])
    c_bounds.setLow(ow, min_fwd[ow])
    c_bounds.setHigh(ow, max_fwd[ow])
    c_space.setBounds(c_space)

    ss = oc.SimpleSetup(c_space)
    return ss


def set_goal(ss, start_pos, goal_pos):
    # create a start state
    start = ob.SE3StateSpace(ss)
    start.setX(start_pos[x])
    start.setY(start_pos[y])
    start.setZ(start_pos[z])
    goal = ob.SE3StateSpace(ss)
    goal.setX(goal_pos[x])
    goal.setY(goal_pos[y])
    goal.setZ(goal_pos[z])
    ss.setStartAndGoalStates(start, goal, 0.05)


class RRT:

    def __init__(self, side):
        self.side = side

    def plan(self, start_pos, goal_pos, p_goal=0.05, step_size=0.05):
        ss = setup_bounds(self.side)
        set_goal(ss, start_pos, goal_pos)
        si = ss.getSpaceInformation()
        si.setPropagationStepSize(step_size)
        validity_checker = ob.StateValidityCheckerFn(partial(is_state_valid, si))
        ss.setStateValidityChecker(validity_checker)
        ode = oc.ODE(propagator)
        ode_solver = oc.ODEBasicSolver(si, ode)
        propagator_fn = oc.ODEBasicSolver.getStatePropagator(ode_solver)
        ss.setStatePropagator(propagator_fn)
        ss.setup()
        rrt_star = og.RRTstar(si)
        rrt_star.setGoalBias(p_goal)
        rrt_star.setRange(step_size)
        ss.setPlanner(rrt_star)
        solved = ss.solve(20.0)

        if solved:
            print("Found solution path:\n%s" % ss.getSolutionPath().printAsMatrix())
