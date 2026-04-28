"""Traveling Salesman Problem executor using D*Lite.

On startup:
  1. Subscribe to /map and /amcl_pose.
  2. Once both arrive, build an inflated obstacle grid.
  3. Run D* Lite once per vertex (robot start + every waypoint) in full-expand
     mode to populate a pairwise cost matrix.
  4. Brute-force all permutations of the waypoints (4! = 24) to pick the
     minimum-length tour from the robot's current pose.
  5. Send each leg to Nav2's /navigate_to_pose action in order.

See docs/dstar_lite_tradeoffs.md for why this particular problem does not
exercise D* Lite's incremental strengths.
"""

from __future__ import annotations

import itertools
import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool

from turtlebot4_reactive_controller.dstar_lite import INF, DStarLite
from turtlebot4_reactive_controller.waypoints import WAYPOINTS, Waypoint

# the robot is basically a circle, this is how wide it is
ROBOT_RADIUS_M = 0.18  # TurtleBot 4 footprint radius (approx)

# every half meter the robot stops and looks for a face (can change via ROS param)
FACE_CHECK_DEFAULT_M = 1.2           # ~20 inches; override with ROS param
FACE_TOPIC_DEFAULT = '/face_detected'  # published by face_detector node
CMD_VEL_TOPIC_DEFAULT = '/cmd_vel_unstamped'   # Create 3 expects Twist here, only used for 
                                               # face reaction, other movements handled by Nav2
ROTATE_SPEED_RAD_S = 0.2    # how fast we spin during the 180 turn (rad/s)
ROTATE_TIMEOUT_S = 15.0     # give up spinning after 15s so we don't get stuck forever
FIRST_LOOK_TIMEOUT_S = 3.0  # wait 3s for a face before giving up and beeping
BEEP_INTERVAL_S = 1.0       # beep every second until someone shows up
BEEP_FREQ_HZ = 880          # annoying note
BEEP_DURATION_MS = 200      # short beep so it doesn't sound like a flatline


def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    """8-connected binary dilation by `radius` cells, numpy-only."""
    # basically we're making obstacles "fatter" so the robot doesn't clip walls
    out = mask.astype(bool, copy=True)
    for _ in range(radius):
        # Pad with False so border cells don't wrap or raise index errors
        pad = np.pad(out, 1, constant_values=False)
        # OR all 8 neighbors plus self: a cell becomes True if any neighbor was True.
        # Each slice is the padded array shifted by one cell in that direction:
        #   [:-2, :-2]=NW  [:-2,1:-1]=N   [:-2, 2:]=NE
        #   [1:-1,:-2]=W   [1:-1,1:-1]=C  [1:-1, 2:]=E
        #   [2:,  :-2]=SW  [2:, 1:-1]=S   [2:,   2:]=SE
        out = (
            pad[:-2, :-2] | pad[:-2, 1:-1] | pad[:-2, 2:] |
            pad[1:-1, :-2] | pad[1:-1, 1:-1] | pad[1:-1, 2:] |
            pad[2:, :-2] | pad[2:, 1:-1] | pad[2:, 2:]
        )
    return out


class TSPExecutor(Node):

    def __init__(self):
        super().__init__('tsp_executor')
        # these stay None until the map/pose callbacks fire
        self.obstacles: np.ndarray | None = None
        self.map_info = None
        self.current_pose: tuple[float, float] | None = None

        # Params - pull from ROS parameter server so we can tune without recompiling
        self.declare_parameter('face_check_distance_m', FACE_CHECK_DEFAULT_M)
        self.declare_parameter('face_topic', FACE_TOPIC_DEFAULT)
        self.declare_parameter('cmd_vel_topic', CMD_VEL_TOPIC_DEFAULT)
        self.face_check_distance_m = float(
            self.get_parameter('face_check_distance_m').value
        )
        face_topic = self.get_parameter('face_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # the map is latched (TRANSIENT_LOCAL) so we get it even if we subscribe late
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_pose, 10, # Creates a 10 long buffer of AMCL approximations
        )

        # Odometry: used to track distance since last face check and yaw for turns
        self._last_odom_xy: tuple[float, float] | None = None
        self._last_odom_yaw: float | None = None
        self._distance_since_check: float = 0.0
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)

        # Face-detection flag published by the face_detector node
        self._face_detected: bool = False
        self.create_subscription(Bool, face_topic, self._on_face, 10)

        # Actuators - we need to publish velocity commands and play sounds
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.audio_pub = self.create_publisher(
            AudioNoteVector, '/cmd_audio', 10,
        )

        # this is how we tell Nav2 "go here" and wait for it to finish
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    # subscriptions

    def _on_map(self, msg: OccupancyGrid):
        self.map_info = msg.info
        # reshape the flat array into a 2D grid matching the map dimensions
        arr = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width,
        )
        # anything >= 50% occupied OR unknown (-1) counts as an obstacle
        obstacle = (arr >= 50) | (arr < 0)  # treat unknown as obstacle
        # convert robot radius from meters to grid cells, minimum 1
        radius_cells = max(1, int(math.ceil(ROBOT_RADIUS_M / msg.info.resolution)))
        # inflate obstacles so the planner keeps the robot body away from walls
        self.obstacles = _dilate(obstacle, radius_cells).astype(np.uint8)
        self.get_logger().info(
            f'map loaded: {msg.info.width}x{msg.info.height} @ '
            f'{msg.info.resolution:.3f}m/cell, inflated by {radius_cells} cells'
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        # just grab x,y from the AMCL estimate, we don't need the covariance
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Extract yaw from quaternion using the standard ZYX Euler decomp
        # Only the z-axis rotation matters for a ground robot moving in 2D.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        xy = (p.x, p.y)
        if self._last_odom_xy is not None:
            # Accumulate straight-line distance traveled since the last face check.
            dx = xy[0] - self._last_odom_xy[0]
            dy = xy[1] - self._last_odom_xy[1]
            self._distance_since_check += math.hypot(dx, dy)
        self._last_odom_xy = xy
        self._last_odom_yaw = yaw

    def _on_face(self, msg: Bool):
        # just a boolean flag, either a face was seen or it wasn't
        self._face_detected = bool(msg.data)

    # interrupt behaviors

    def _rotate_180(self) -> None:
        if self._last_odom_yaw is None:
            self.get_logger().warn('no odom yet; skipping 180 turn')
            return
        twist = Twist()
        twist.angular.z = ROTATE_SPEED_RAD_S  # spin counterclockwise
        start_yaw = self._last_odom_yaw
        last_yaw = start_yaw
        accumulated = 0.0  # total angle rotated so far
        # safety timeout so we don't spin forever if odom dies
        deadline_ns = (
            self.get_clock().now().nanoseconds + int(ROTATE_TIMEOUT_S * 1e9)
        )
        while rclpy.ok() and accumulated < math.pi:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            cur = self._last_odom_yaw
            if cur is None:
                continue
            # atan2(sin, cos) gives the shortest signed angle delta, handles wrap-around
            d = math.atan2(math.sin(cur - last_yaw), math.cos(cur - last_yaw))
            accumulated += abs(d)
            last_yaw = cur
            if self.get_clock().now().nanoseconds > deadline_ns:
                self.get_logger().warn('180 turn timed out')
                break
        self.cmd_vel_pub.publish(Twist())  # stop - send zero velocity

    def _beep(self) -> None:
        # build a single-note audio message and fire it at the speaker
        note = AudioNote()
        note.frequency = BEEP_FREQ_HZ
        note.max_runtime = Duration(
            sec=0, nanosec=BEEP_DURATION_MS * 1_000_000,
        )
        msg = AudioNoteVector()
        msg.notes = [note]
        msg.append = False  # replace whatever's playing, don't queue after it
        self.audio_pub.publish(msg)

    def _wait_for_face(self, timeout_s: float) -> bool:
        """Spin for up to timeout_s, returning True once /face_detected is True."""
        deadline_ns = (
            self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        )
        # keep spinning the event loop so callbacks keep firing
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline_ns:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._face_detected:
                return True
        return False  # timed out, nobody home

    def _face_check(self) -> None:
        """Turn around, look for a face, beep until one is found."""
        self.get_logger().info('interrupt: rotating 180')
        self._rotate_180()
        self.get_logger().info('interrupt: scanning for face')
        # give it 3 seconds to spot someone before escalating to beeping
        if self._wait_for_face(FIRST_LOOK_TIMEOUT_S):
            self.get_logger().info('face detected, resuming')
            return
        # nobody looked, start beeping annoyingly until they do
        self.get_logger().warn('no face detected, COMMENCE BEEPING')
        while rclpy.ok():
            self._beep()
            if self._wait_for_face(BEEP_INTERVAL_S):
                self.get_logger().info('face detected, keep going')
                self._rotate_180()
                return

    # coordinate conversion

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        # the map origin is the bottom-left corner in world coordinates
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        # subtract origin offset then divide by cell size to get grid indices
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        return (row, col)

    def _cell_ok(self, cell: tuple[int, int]) -> bool:
        r, c = cell
        # first check bounds, then check if the cell is free (0 = free, 1 = obstacle)
        if not (0 <= r < self.obstacles.shape[0] and 0 <= c < self.obstacles.shape[1]):
            return False
        return self.obstacles[r, c] == 0

    # TSP planning

    def solve_tour(self) -> list[Waypoint]:
        """Return waypoints in the order the robot should visit them."""
        # treat the robot's current position as the mandatory first stop
        start = Waypoint(name='START', x=self.current_pose[0], y=self.current_pose[1])
        nodes: list[Waypoint] = [start] + list(WAYPOINTS)
        cells = [self._world_to_grid(n.x, n.y) for n in nodes]
        # sanity check: make sure none of our waypoints are inside a wall
        for n, c in zip(nodes, cells):
            if not self._cell_ok(c):
                raise RuntimeError(
                    f'waypoint {n.name} ({n.x:.2f},{n.y:.2f}) maps to blocked '
                    f'or out-of-bounds cell {c}'
                )

        n = len(nodes)
        # n x n cost matrix, cost[i][j] = path cost from node i to node j
        cost = [[INF] * n for _ in range(n)]
        # run D* Lite from each node as the goal to fill in a whole column at once
        for j in range(n):
            planner = DStarLite(self.obstacles, goal=cells[j])
            planner.compute_full()  # expands the whole grid, not just to one target
            for i in range(n):
                cost[i][j] = planner.cost_to_goal(cells[i]) if i != j else 0.0
            self.get_logger().info(
                f'cost field computed with goal={nodes[j].name}'
            )

        # Brute-force: start is fixed at index 0; permute the remaining 1..n-1.
        # It's O(n!) but n=4 means only 24 permutations, so whatever
        wp_indices = list(range(1, n))
        best_perm, best_cost = None, INF
        for perm in itertools.permutations(wp_indices):
            total = 0.0
            prev = 0
            for idx in perm:
                total += cost[prev][idx]
                prev = idx
            if total < best_cost:
                best_cost, best_perm = total, perm

        if best_perm is None or best_cost >= INF:
            raise RuntimeError('no feasible tour, one or more legs unreachable')

        order = [nodes[i] for i in best_perm] # copy best order
        order_names = ' -> '.join(['START'] + [w.name for w in order])
        self.get_logger().info(
            f'best tour: {order_names}  (total cost = {best_cost:.2f} cells)'
        )
        return order

    # Nav2 execution

    def navigate_to(self, wp: Waypoint) -> bool:
        """Drive to waypoint (wp), interrupting every `face_check_distance_m` to run
        the turn-and-detect-face ritual. Resumes by re-sending the same goal
        so Nav2 replans from the after pose."""
        while rclpy.ok():
            self._distance_since_check = 0.0  # reset odometer for this leg

            # build the Nav2 goal message - orientation.w=1 means "don't care about heading"
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose.header.frame_id = 'map'
            nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
            nav_goal.pose.pose.position.x = wp.x
            nav_goal.pose.pose.position.y = wp.y
            nav_goal.pose.pose.orientation.w = 1.0
            self.get_logger().info(f'-> {wp.name}  ({wp.x:.2f}, {wp.y:.2f})')

            # send the goal and block until Nav2 accepts (or rejects) it
            goal_acceptance_future = self.nav_client.send_goal_async(nav_goal)
            rclpy.spin_until_future_complete(self, goal_acceptance_future)
            goal_handle = goal_acceptance_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f'goal {wp.name} rejected by Nav2')
                return False

            # now poll in a loop - we need to check distance every 100ms
            nav_result_future = goal_handle.get_result_async()
            interrupted_for_face_check = False
            while rclpy.ok() and not nav_result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1) # Process subscriptions once
                if self._distance_since_check >= self.face_check_distance_m:
                    self.get_logger().info(
                        f'  {wp.name}: {self._distance_since_check:.2f} m '
                        f'traveled, cancelling for face check'
                    )
                    # cancel the current nav goal so we can spin in place
                    cancel_ack_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_ack_future)
                    rclpy.spin_until_future_complete(self, nav_result_future)
                    interrupted_for_face_check = True
                    break

            if interrupted_for_face_check:
                self._face_check()
                continue  # re-send same goal; Nav2 replans from new pose

            final_status = nav_result_future.result().status
            # 4 == STATUS_SUCCEEDED (action_msgs/GoalStatus)
            reached_waypoint = final_status == 4 # 4 is STATUS_SUCCEEDED
            self.get_logger().info(
                f'  {wp.name} finished status={final_status} '
                f'({"OK" if reached_waypoint else "FAIL"})'
            )
            return reached_waypoint
        return False


def main():
    rclpy.init()
    node = TSPExecutor()
    try:
        # spin until we have a map AND know where we are - can't plan without both
        node.get_logger().info('waiting for /map and /amcl_pose ...')
        while rclpy.ok() and (node.obstacles is None or node.current_pose is None):
            rclpy.spin_once(node, timeout_sec=0.5)

        # Nav2 might still be booting up, wait for the action server to be ready
        node.get_logger().info('waiting for /navigate_to_pose action server ...')
        node.nav_client.wait_for_server()

        # solve TSP then drive the tour in order
        order = node.solve_tour()
        for wp in order:
            if not node.navigate_to(wp):
                node.get_logger().error('leg failed, aborting tour')
                break
        else:
            node.get_logger().info('tour complete')
    finally:
        # always clean up even if something crashes
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
