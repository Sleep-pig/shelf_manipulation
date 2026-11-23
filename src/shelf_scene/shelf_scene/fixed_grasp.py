"""Execute the fixed front-facing approach against the observed target."""
from copy import deepcopy
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node

from shelf_planning.srv import EvaluateCandidate

from shape_msgs.msg import SolidPrimitive

from .layout import layout


class MissedGrasp(RuntimeError):
    pass


class FixedGrasp(Node):
    def __init__(self):
        super().__init__("fixed_grasp")
        self.strategy = self.declare_parameter("strategy", "fixed").value
        if self.strategy not in ("fixed", "multi", "precheck"):
            raise ValueError("strategy must be fixed, multi or precheck")
        self.yaw = 0.
        self.planning_seconds = 0.
        self.stage = "initialization"
        self.grasped = False
        self.attempts = 0
        self.rejections = []

    def wait(self, future, timeout=40):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("ROS request timed out")
        return future.result()

    def service(self, kind, name, request):
        client = self.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError(f"{name} unavailable")
            started = time.monotonic()
            reply = self.wait(client.call_async(request))
            if kind in (GetCartesianPath, GetPositionIK, EvaluateCandidate):
                self.planning_seconds += time.monotonic() - started
            return reply
        finally:
            self.destroy_client(client)

    def action(self, kind, name, goal):
        client = ActionClient(self, kind, name)
        try:
            if not client.wait_for_server(timeout_sec=10):
                raise RuntimeError(f"{name} unavailable")
            handle = self.wait(client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError(f"{name} rejected")
            result = self.wait(handle.get_result_async())
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                code = getattr(result.result, "error_code", None)
                raise RuntimeError(f"{name} failed: status={result.status}, error={code}")
            return result.result
        finally:
            client.destroy()

    def hand_pose(self, target, setback):
        pose = Pose()
        pose.position.x = target.x - setback * math.cos(self.yaw)
        pose.position.y = target.y - setback * math.sin(self.yaw)
        pose.position.z = target.z + 0.015
        # Rz(yaw) Ry(pi/2) points the fingers along the horizontal approach.
        pose.orientation.y = pose.orientation.w = math.sqrt(.5) * math.cos(self.yaw / 2)
        pose.orientation.z = math.sqrt(.5) * math.sin(self.yaw / 2)
        pose.orientation.x = -pose.orientation.z
        return pose

    def ik(self, pose, seed=None):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "panda_arm"
        ik.ik_link_name = "panda_hand"
        ik.pose_stamped.header.frame_id = "world"
        ik.pose_stamped.pose = pose
        if seed is not None:
            ik.robot_state = seed
        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.timeout.sec = 2
        answer = self.service(GetPositionIK, "compute_ik", request)
        if answer.error_code.val != 1:
            raise RuntimeError(f"Pregrasp IK failed: {answer.error_code.val}")
        return answer.solution

    def pregrasp(self, pose):
        self.move_joints(self.ik(pose).joint_state)

    def move_joints(self, joints):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = "panda_arm"
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.35
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=value, tolerance_above=.002,
                            tolerance_below=.002, weight=1.)
            for name, value in zip(joints.name, joints.position) if name.startswith("panda_joint")
        ])]
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = True
        started = time.monotonic()
        try:
            result = self.action(MoveGroup, "move_action", goal)
        finally:
            self.planning_seconds += time.monotonic() - started
        self.execute(result.planned_trajectory)

    def cartesian(self, pose, start=None):
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.group_name = "panda_arm"
        request.link_name = "panda_hand"
        if start is not None:
            request.start_state = start
        request.start_state.is_diff = True
        request.waypoints = [pose]
        request.max_step = .005
        request.revolute_jump_threshold = .25
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = request.max_acceleration_scaling_factor = .2
        answer = self.service(GetCartesianPath, "compute_cartesian_path", request)
        if answer.error_code.val != 1 or answer.fraction < .99999:
            raise RuntimeError(f"Incomplete Cartesian path: {answer.fraction:.3f}")
        return answer.solution

    def straight(self, pose):
        self.execute(self.cartesian(pose))

    def execute(self, trajectory):
        self.action(ExecuteTrajectory, "execute_trajectory",
                    ExecuteTrajectory.Goal(trajectory=trajectory))

    def grip(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = width / 2
        goal.command.max_effort = 2.5
        controllers = ["panda_hand_controller", "panda_right_finger_controller"]
        clients = [ActionClient(self, GripperCommand, f"{name}/gripper_cmd") for name in controllers]
        try:
            if not all(client.wait_for_server(timeout_sec=10) for client in clients):
                raise RuntimeError("Finger controllers unavailable")
            requests = [client.send_goal_async(goal) for client in clients]
            handles = [self.wait(request) for request in requests]
            if not all(handle.accepted for handle in handles):
                raise RuntimeError("Finger command rejected")
            results = [self.wait(handle.get_result_async()) for handle in handles]
            if any(result.status != GoalStatus.STATUS_SUCCEEDED for result in results):
                raise RuntimeError("Finger motion failed")
            return sum(result.result.position for result in results)
        finally:
            for client in clients:
                client.destroy()

    def observe(self, name, frame="world"):
        result = self.service(GetEntityState, "/gazebo/get_entity_state",
                              GetEntityState.Request(name=name, reference_frame=frame))
        if not result.success:
            raise RuntimeError(f"Gazebo entity unavailable: {name}")
        stamp = result.header.stamp
        self.last_observation_time = stamp.sec + stamp.nanosec * 1e-9
        return result.state

    def attach(self, attach):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        obj = CollisionObject(id="target", operation=CollisionObject.ADD if attach else CollisionObject.REMOVE)
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name="panda_hand", object=obj, touch_links=["panda_leftfinger", "panda_rightfinger"])]
        reply = self.service(ApplyPlanningScene, "apply_planning_scene",
                             ApplyPlanningScene.Request(scene=scene))
        if not reply.success:
            raise RuntimeError("MoveIt rejected target attachment update")

    def log_contact_geometry(self, phase):
        for name in ("panda::panda_leftfinger", "panda::panda_rightfinger", "target"):
            p = self.observe(name).pose.position
            self.get_logger().info(f"{phase} {name}: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

    def select_approach(self, target, box_yaw):
        query = GetPlanningScene.Request()
        query.components.components = 1023
        scene = self.service(GetPlanningScene, "get_planning_scene", query).scene
        initial = scene.robot_state
        current = dict(zip(initial.joint_state.name, initial.joint_state.position))
        candidates = []
        for yaw in dict.fromkeys([0., box_yaw, box_yaw-.22, box_yaw+.22]):
            self.yaw = yaw
            for elbow in (0., -.65, .65):
                seed = deepcopy(initial)
                j = seed.joint_state.name.index("panda_joint3")
                seed.joint_state.position[j] = max(-2.8, min(2.8, current["panda_joint3"] + elbow))
                label = f"yaw={yaw:.3f} elbow_offset={elbow:.2f}"
                try:
                    start = self.ik(self.hand_pose(target, .22), seed)
                    path = self.cartesian(self.hand_pose(target, .108), start)
                    check = EvaluateCandidate.Request(scene=scene, start=start,
                                                       approach=path.joint_trajectory)
                    if self.strategy == "precheck":
                        lift = self.hand_pose(target, .108)
                        lift.position.z += .035
                        retreat = deepcopy(lift)
                        retreat.position.x = .26
                        check.loaded_waypoints = [lift, retreat]
                    result = self.service(EvaluateCandidate, "evaluate_candidate", check)
                    if not result.valid:
                        raise RuntimeError(result.reason)
                    travel = sum(abs(v-current[n]) for n, v in
                                 zip(start.joint_state.name, start.joint_state.position)
                                 if n.startswith("panda_joint"))
                    for a, b in zip(path.joint_trajectory.points, path.joint_trajectory.points[1:]):
                        travel += sum(abs(u-v) for u, v in zip(a.positions, b.positions))
                    candidates.append((result.clearance, travel, yaw, start, path, label))
                except RuntimeError as error:
                    self.rejections.append(f"{label}: {error}")
                    self.get_logger().info(f"Candidate rejected {label}: {error}")
        for clearance, travel, yaw, start, path, label in sorted(
                candidates, key=lambda candidate: (-candidate[0], candidate[1])):
            self.yaw = yaw
            try:
                self.move_joints(start.joint_state)
            except RuntimeError as error:
                self.rejections.append(f"{label}: {error}")
                self.get_logger().info(f"Candidate transfer rejected {label}: {error}")
                continue
            self.execute(path)
            self.get_logger().info(f"Selected {label} clearance={clearance:.4f} travel={travel:.3f}")
            return
        raise RuntimeError("All candidate approaches rejected")

    def run(self):
        for attempt in range(2):
            self.attempts = attempt + 1
            try:
                self.pick_place()
                return
            except MissedGrasp as error:
                if attempt:
                    raise
                self.get_logger().info(f"{error}; reobserving target for final attempt")

    def pick_place(self):
        self.stage = "approach"
        observed = self.observe("target").pose
        target = observed.position
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        size = next(size for name, size, _, _ in layout(0) if name == "target")
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
        obj = CollisionObject(id="target", primitives=[box], primitive_poses=[observed])
        obj.header.frame_id = "world"
        scene.world.collision_objects = [obj]
        if not self.service(ApplyPlanningScene, "apply_planning_scene",
                            ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError("Failed to refresh observed target")
        self.grip(.08)
        if self.strategy == "fixed":
            self.pregrasp(self.hand_pose(target, .22))
            self.straight(self.hand_pose(target, .108))
        else:
            q = observed.orientation
            box_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.select_approach(target, box_yaw)
        self.get_logger().info("Approach completed")
        self.log_contact_geometry("approach")
        self.stage = "grasp"
        self.grip(.060)
        self.grip(.050)
        opening = self.grip(.030)
        if opening < .033:
            self.grip(.08)
            self.straight(self.hand_pose(target, .22))
            raise MissedGrasp("Empty grasp: measured opening below 33 mm; reopened and retracted")
        self.get_logger().info(f"Jaw opening after closing: {opening:.4f} m")
        self.log_contact_geometry("close")
        before = self.observe("target", "panda::panda_link7").pose.position
        self.attach(True)
        lift = self.hand_pose(target, .108)
        lift.position.z += .035
        self.straight(lift)
        raised = self.observe("target").pose.position
        after = self.observe("target", "panda::panda_link7").pose.position
        slip = math.dist((before.x, before.y, before.z), (after.x, after.y, after.z))
        if abs(raised.z - target.z - .035) > .008 or slip > .008:
            self.grip(.08)
            self.attach(False)
            self.straight(self.hand_pose(target, .22))
            raise MissedGrasp(f"Box moved {raised.z-target.z:.4f} m during 35 mm test lift, relative slip={slip:.4f} m")
        self.get_logger().info(f"Verified physical lift: {raised.z - target.z:.3f} m")
        self.grasped = True
        self.stage = "withdraw"
        lift.position.x = .26
        self.straight(lift)
        self.stage = "transfer"
        tray = self.observe("tray").pose.position
        place = self.hand_pose(tray, .108)
        place.position.z = tray.z + .06
        self.pregrasp(place)
        self.log_contact_geometry("release")
        self.grip(.08)
        self.attach(False)
        self.stage = "placement"
        self.verify_placement(tray)
        self.stage = "complete"

    def verify_placement(self, tray):
        sizes = {name: size for name, size, _, _ in layout(0)}
        box = sizes["target"]
        tray_size = sizes["tray"]
        deadline = time.monotonic() + 4
        stable_since = None
        while time.monotonic() < deadline:
            state = self.observe("target")
            p, q = state.pose.position, state.pose.orientation
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = (
                (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)),
            )
            ex, ey, ez = [sum(abs(v)*d/2 for v, d in zip(row, box)) for row in rotation]
            inside = (abs(p.x-tray.x)+ex <= tray_size[0]/2 and
                      abs(p.y-tray.y)+ey <= tray_size[1]/2 and
                      abs(p.z-ez-tray.z-tray_size[2]/2) <= .003)
            linear, angular = state.twist.linear, state.twist.angular
            still = math.hypot(linear.x, linear.y, linear.z) < .005 and (
                math.hypot(angular.x, angular.y, angular.z) < .05)
            stamp = self.last_observation_time
            if inside and still:
                if stable_since is None:
                    stable_since = stamp
                if stamp - stable_since >= .5:
                    self.get_logger().info("Box stationary inside tray for 0.5 simulation seconds")
                    return
            else:
                stable_since = None
            time.sleep(.05)
        raise RuntimeError(f"Tray check failed: position={p}, inside={inside}, still={still}, stamp={stamp}, stable_since={stable_since}")



def main():
    rclpy.init()
    node = FixedGrasp()
    error = None
    try:
        node.run()
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        print("TRIAL_RESULT " + json.dumps(dict(
            strategy=node.strategy, success=node.stage == "complete", stage=node.stage,
            grasped=node.grasped, attempts=node.attempts, planning_s=node.planning_seconds,
            rejections=node.rejections, error=error)), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
