from heapq import heappush, heappop
from math import hypot, floor, inf

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_services_default,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):  # comparator for heapq (min-heap) sorting
        return self.g < other.g


class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # !TODO: Path request subscriber (Done - CY)
        self.sub_path_request_ = self.create_subscription(
            Path,
            "path_request",
            self.callbackSubPathRequest_,
            10, #can be fine tuned
        )

        # Handles: Publishers
        # !TODO: Path publisher (Done - CY))
        self.pub_path_ = self.create_publisher(
            Path,
            "path",
            10, #can be fine tuned
        )
    

        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):   
        
        # !TODO: write to rbt_x_, rbt_y_, goal_x_, goal_y_ (Done - CY)
        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y
        
        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        
        # !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
        # (Done - CY)
        self.costmap_resolution_ = msg.info.resolution
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.costmap_rows_ = msg.info.height
        self.costmap_cols_ = msg.info.width

        self.costmap_ = list(msg.data) #create a copy of the costmap data

        # Check if costmap data length matches rows * cols
        '''
        expected_len = int(self.costmap_rows_ * self.costmap_cols_)
        if len(self.costmap_) != expected_len:
            self.get_logger().warn(
                f"Costmap data length ({len(self.costmap_)}) != width*height ({expected_len})."
            )
        '''

        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # run the path planner
        self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)

        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row. (Done - CY)
    def XYToCR_(self, x, y):
        #Logic: x value - costmap origin, to get relative position in meters. Then divide by resolution to get cell index.
        # Using floor to get cell wrt to bottom left of cell, then convert to int.
        # Map continuous world coords (meters, map frame) to integer grid indices
        # Column increases with +X, Row increases with +Y
        c = int(floor((x - self.costmap_origin_x_) / self.costmap_resolution_))
        r = int(floor((y - self.costmap_origin_y_) / self.costmap_resolution_))

        return c, r

    # Converts cell column and cell row to world coordinates. (Done - CY)
    def CRToXY_(self, c, r):
        #Logic: reverse of XYToCR_, + self.costmap_resolution_/2 to get to center of cell
        # Map integer grid indices (c,r) to the center of that cell in world coords (meters)
        x = self.costmap_origin_x_ + (c * self.costmap_resolution_) + self.costmap_resolution_ / 2
        y = self.costmap_origin_y_ + (r * self.costmap_resolution_) + self.costmap_resolution_ / 2

        return x, y

    # Converts cell column and cell row to flattened array index. (Done - CY)
    #Logic: rows * total number of columns to get position of row in list, then + position in current row
    def CRToIndex_(self, c, r):
        return int(r * self.costmap_cols_ + c)

    # Returns true if the cell column and cell row is outside the costmap.
    #(Done - CY)
    #Logic: if c or r < 0 or more than total cols/rows - 1 return true.
    def outOfMap_(self, c, r):
        return (c < 0 or r < 0) or ((c >= self.costmap_cols_) or (r >= self.costmap_rows_))

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):

        #Exit if start = goal (Todo)
        # Delete both lines when ready to code planner.py -----------------
        #self.publishInterpolatedPath(start_x, start_y, goal_x, goal_y)
        #return

        # Initializations ---------------------------------
        
        # Initialize nodes
        #nodes = [DijkstraNode(0, 0)]  # replace this (Done - CY)
        rows = self.costmap_rows_
        cols = self.costmap_cols_
        nodes = [DijkstraNode(c,r) for r in range(rows) for c in range(cols)]

        # Initialize start and goal
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)  # replace this (Done - CY))
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        start_node = nodes[rbt_idx]
        start_node.g = 0.0
        start_node.parent = None #actually dont need this

        # Initialize open list
        open_list = []
        heappush(open_list, start_node)

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            node = heappop(open_list)

            # Skip if visited
            if node.expanded:
                continue
            node.expanded = True

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"
                nodelist = []

                # obtain the path from the nodes. (Done - CY)
                while node.parent is not None:
                    nodelist.append(node)
                    node = node.parent

                nodelist.append(node)  # append start node
                nodelist.reverse()  # reverse to get path from start to goal
                for node in nodelist:
                    pose = PoseStamped()
                    pose.pose.position.x, pose.pose.position.y = self.CRToXY_(node.c, node.r)
                    msg_path.poses.append(pose)

                # publish path
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                nb_c = node.c + dc
                nb_r = node.r + dr
                #nb_idx = 0 * nb_c * nb_r
                nb_idx = self.CRToIndex_(nb_c, nb_r)

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                # Get the neighbor node
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded
                if nb_node.expanded:
                    continue

                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)
                cell_cost = self.costmap_[nb_idx]
                if cell_cost > self.max_access_cost_:
                    continue

                # Get the relative g-cost and push to open-list
                dist = hypot(dc, dr) #account for diagonal movement
                relative_g = node.g + dist * (cell_cost + 1)
                if relative_g < nb_node.g:
                    nb_node.g = relative_g
                    nb_node.parent = node
                    heappush(open_list, nb_node)


        self.get_logger().warn("No Path Found!")


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
