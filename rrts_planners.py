from pathlib import Path
import sys
from collections import deque
import argparse

import numpy as np
from typing import List, Optional, Tuple

if __package__ in (None, ""):
    project_root = Path(__file__).resolve().parents[1]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))

from utils.kd_tree import KDTree, Node
from tabletop_sim.obstacles import TabletopEnv, closed_polygon


class RRTStarTabletop:
    def __init__(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        env: TabletopEnv,
        step_len: float = 1.0,
        search_radius: float = 2.0,
        max_iter: int = 1000,
        cost_type: str = "length"
    ):
        """
        start, goal: (x, y)
        env: TabletopEnv, provides obstacle generation/collision checks
        step_len: step size for tree expansion
        search_radius: radius for rewiring neighbors
        max_iter: maximum iterations
        """
        self.env = env
        self.start = Node(start)
        self.goal = Node(goal)

        self.x_range: Tuple[float, float] = (
            float(self.env.x_range[0]),
            float(self.env.x_range[1]),
        )
        self.y_range: Tuple[float, float] = (
            float(self.env.y_range[0]),
            float(self.env.y_range[1]),
        )
        self.step_len = step_len
        self.search_radius = search_radius
        self.max_iter = max_iter

        self.nodes: List[Node] = [self.start]
        self._kdtree = KDTree(self.start, k_dimension=2)

        if self.collision_pt(self.start):
            raise ValueError("start is in collision with environment obstacles")
        if self.collision_pt(self.goal):
            raise ValueError("goal is in collision with environment obstacles")

        # Cost is path length.
        self.cost_type = cost_type
        if self.cost_type == "length":
            self.start.cost = 0.0
            self.start.len = 0.0
            self.goal.cost = float("inf")
            self.goal.len = float("inf")
        elif self.cost_type == "clearance":
            self.start.cost = -self.start.clearance
        else:
            raise ValueError("Unknown cost type in planner")

    def dist(self, n1: Node, n2: Node):
        return float(np.linalg.norm(n1.pos - n2.pos))

    def collision_pt(self, node: Node):
        assert node is not None, "Node is None in collision check"
        return not self.env.pt_obstacle_free(node.pos)

    def collision_edge(self, n1: Node, n2: Node):
        assert n1 is not None and n2 is not None, "Node is None in collision_edge check"
        return not self.env.seg_obstacle_free(n1.pos, n2.pos)

    def sample(self):
        return Node(
            [
                np.random.uniform(*self.x_range),
                np.random.uniform(*self.y_range),
            ]
        )

    def nearest(self, rnd: Node) -> Optional[Node]:
        if self._kdtree is None:
            return None
        return self._kdtree.nearest_node(rnd)

    def get_neighbors(self, node: Node):
        if self._kdtree is None:
            return []
        return self._kdtree.radius_search(
            center=tuple(node.pos), radius=self.search_radius
        )

    def steer(self, from_node: Node, to_node: Node) -> Optional[Node]:
        direction = to_node.pos - from_node.pos
        dist = np.linalg.norm(direction)
        if dist <= 1e-8:
            return None

        direction /= dist
        if dist <= self.step_len:
            new_pos = to_node.pos
        else:
            new_pos = from_node.pos + direction * self.step_len

        new_pos = np.clip(
            new_pos,
            [self.x_range[0], self.y_range[0]],
            [self.x_range[1], self.y_range[1]],
        )

        new_node = Node(new_pos)
        if self.cost_type == "clearance":
            new_node.clearance = self.env.pt_clearance(new_node.pos)
        return new_node
    
    def update_node_cost(self, node: Node):
        node.cost = 0
        if self.cost_type == "length":
            node.cost = node.len
        elif self.cost_type == "clearance":
            if node.parent is None:
                node.cost = -10000 # start position
            else:
                node.cost = -min(-node.parent.cost, node.clearance)

    def new_cost(self, from_node: Node, to_node: Node):
        # if from_node is not None or to_node is not None:
        #     raise ValueError("Neither from_node and to_node should be None")
        
        new_len = from_node.len + self.dist(from_node, to_node)
        if self.cost_type == "length":
            return new_len
        elif self.cost_type == "clearance":
            return -min(-from_node.cost, to_node.clearance)
        

    def choose_best_parent(self, new_node: Node, nearest: Node, neighbors: List[Node]):
        best_parent = nearest
        best_cost = self.new_cost(nearest, new_node)

        for node in neighbors:
            if self.collision_edge(node, new_node):
                continue

            temp_cost = self.new_cost(node, new_node)
            if temp_cost < best_cost - 1e-9:
                best_parent = node
                best_cost = temp_cost

        return best_parent

    def update_subtree(self, new_parent: Node, child: Node):
        old_parent = child.parent
        if old_parent is new_parent:
            return
        if old_parent is not None:
            old_parent.children = [c for c in old_parent.children if c is not child]
        child.parent = new_parent
        child.len = new_parent.len + self.dist(new_parent, child)
        self.update_node_cost(child)
        if new_parent is not None and child not in new_parent.children:
            new_parent.children.append(child)

        queue = deque([child])
        while queue:
            node = queue.popleft()
            for child in node.children:
                child.len = node.len + self.dist(node, child)
                self.update_node_cost(child)
                queue.append(child)

    def rewire(self, new_node: Node, neighbors: List[Node]):
        for node in neighbors:
            if node is new_node.parent:
                continue
            if self.collision_edge(new_node, node):
                continue

            new_cost = self.new_cost(new_node, node)
            if new_cost < node.cost - 1e-9:
                self.update_subtree(new_node, node)

    def plan(self, stop_on_first_path=False):
        """
        Run RRT* planning. If stop_on_first_path is True, return as soon as goal is connected.
        """
        progress_interval = max(1, self.max_iter // 20)

        for it in range(self.max_iter):
            print(f"iteration: {it + 1}/{self.max_iter}", end="\r")
            if it % progress_interval == 0:
                rnd_pos = self.goal.pos + np.random.uniform(-1, 1, (2,)) * 0.1
                rnd = Node(rnd_pos)
            else:
                rnd = self.sample()

            nearest = self.nearest(rnd)

            new_node = self.steer(nearest, rnd)
            if new_node is None or self.collision_pt(new_node):
                continue

            neighbors = self.get_neighbors(new_node)
            parent = self.choose_best_parent(new_node, nearest, neighbors)
            if parent is None:
                continue

            self.update_subtree(parent, new_node)
            self._kdtree.insert_node(new_node)
            self.nodes.append(new_node)
            self.rewire(new_node, neighbors)

            if self.dist(
                new_node, self.goal
            ) <= self.step_len and not self.collision_edge(new_node, self.goal):
                new_cost = self.new_cost(new_node, self.goal)
                if self.goal.parent is None or new_cost < self.goal.cost - 1e-9:
                    self.update_subtree(new_node, self.goal)

                    if stop_on_first_path:
                        return self.get_path()

        if self.goal.parent is not None:
            return self.get_path()
        return None

    def get_path(self):
        if self.goal.parent is None:
            return None

        path = []
        node = self.goal
        while node is not None:
            path.append(node.pos)
            node = node.parent
        path.reverse()
        return path


def _sample_free_point(env: TabletopEnv, max_tries: int = 20000) -> np.ndarray:
    for _ in range(max_tries):
        p = np.array(
            [
                np.random.uniform(env.x_range[0], env.x_range[1]),
                np.random.uniform(env.y_range[0], env.y_range[1]),
            ],
            dtype=float,
        )
        if env.pt_obstacle_free(p):
            return p
    raise RuntimeError(f"Failed to sample a free point after {max_tries} tries.")


def main():
    parser = argparse.ArgumentParser(description="Simple RRT* tabletop planner test")
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    parser.add_argument("--obstacle-num", type=int, default=20, help="Number of obstacles in tabletop env")
    parser.add_argument("--max-iter", type=int, default=1200, help="RRT* max iterations")
    parser.add_argument("--step-len", type=float, default=30.0, help="RRT* step length")
    parser.add_argument("--search-radius", type=float, default=60.0, help="RRT* rewire radius")
    parser.add_argument("--save", type=str, default="", help="Optional output image path")
    args = parser.parse_args()

    # np.random.seed(args.seed)
    env = TabletopEnv(obstacle_num=args.obstacle_num, x_range=[0, 1000], y_range=[0, 600])

    start = _sample_free_point(env)
    goal = _sample_free_point(env)
    for _ in range(10000):
        if np.linalg.norm(goal - start) >= 500.0:
            break
        goal = _sample_free_point(env)
    else:
        raise RuntimeError("Failed to sample sufficiently separated start/goal.")

    planner = RRTStarTabletop(
        start=start,
        goal=goal,
        env=env,
        step_len=args.step_len,
        search_radius=args.search_radius,
        max_iter=args.max_iter,
        cost_type="clearance"
    )
    path = planner.plan(stop_on_first_path=False)
    print()
    print(f"Planner done. nodes={len(planner.nodes)}, path_found={path is not None}")
    if path is not None:
        print(f"Path length={planner.goal.cost:.3f}, waypoints={len(path)}")

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_aspect("equal", adjustable="box")

    # Draw walls.
    for wall in env.env_walls:
        poly = closed_polygon(wall.vertices)
        ax.fill(poly[:, 0], poly[:, 1], color="lightgray", alpha=0.95)
        ax.plot(poly[:, 0], poly[:, 1], color="black", linewidth=1.0)

    # Draw obstacles.
    for obs in env.obstacles:
        poly = closed_polygon(obs.vertices)
        color = "tab:blue" if obs.shape_type == "square" else "tab:orange"
        ax.fill(poly[:, 0], poly[:, 1], color=color, alpha=0.35)
        ax.plot(poly[:, 0], poly[:, 1], color=color, linewidth=1.0)

    # Draw tree edges.
    for node in planner.nodes:
        if node.parent is None:
            continue
        ax.plot(
            [node.parent.pos[0], node.pos[0]],
            [node.parent.pos[1], node.pos[1]],
            color="gray",
            linewidth=0.7,
            alpha=0.35,
        )

    # Draw path.
    if path is not None:
        path_arr = np.asarray(path, dtype=float)
        ax.plot(path_arr[:, 0], path_arr[:, 1], color="crimson", linewidth=2.5, label="path")
        ax.scatter(path_arr[:, 0], path_arr[:, 1], color="crimson", s=16, alpha=0.9)

    # Draw start/goal.
    ax.scatter(start[0], start[1], color="green", s=80, marker="o", label="start", zorder=5)
    ax.scatter(goal[0], goal[1], color="red", s=80, marker="x", label="goal", zorder=5)

    ax.set_xlim(env.x_range[0] - 10, env.x_range[1] + 10)
    ax.set_ylim(env.y_range[0] - 10, env.y_range[1] + 10)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(
        f"RRTStarTabletop (seed={args.seed}, obs={args.obstacle_num}, "
        f"nodes={len(planner.nodes)}, path_found={path is not None})"
    )
    ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.4)
    ax.legend(loc="upper right")

    if args.save:
        fig.savefig(args.save, dpi=160, bbox_inches="tight")
        print(f"Figure saved to: {args.save}")

    backend = plt.get_backend().lower()
    if "agg" in backend:
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    main()
