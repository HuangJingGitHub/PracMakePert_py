"""RRT Implementation"""
import jax.numpy as jnp
import jax.random as random
import seaborn as sns
import matplotlib.pyplot as plt
import copy
import time


class SimpleNode:
    def __init__(self, state, path_len):
        self._state = state
        self._path_len = path_len

    def set_state(self, state):
        self._state = state

    @property
    def state(self):
        return self._state
    
    @property
    def path_len(self):
        return self._path_len
    
    def __repr__(self) -> str:
        return 'Simple Node' + f' at {self._state} obstacles' \
                            + f' with {self._path_len}'


class Node:
    """
        Node in RRT/RRT* algorithm
    """
    def __init__(self, state):
        self._state = state
        self._parent = None
        self.left = None
        self.right = None
        
        # For usage in RRR*
        self._cost = -100. * 200.
        self.min_dist = 200.
        
    def set_parent(self, node):
        self._parent = copy.deepcopy(node)
        
    def set_cost(self, cost):
        self._cost = cost

    def reset_parent(self, node):
        self._parent = copy.deepcopy(node)

    def reset_min_dist(self, min_dist):
        self.min_dist = min_dist

    def __eq__(self, other):
        eq = False
        if jnp.linalg.norm(self.state - other.state) < 1e-3:
            eq = True
        return eq
        
    @property
    def state(self):
        return self._state
    
    @property
    def parent(self):
        return self._parent
    
    @property
    def cost(self):
        return self._cost
        

class kdTree:
    def __init__(self, 
                root: Node):
        self._root = root
        self._kDimension = 2
        self._tree_size = 1
    
    def add_with_root(self, 
                      root: Node,
                      new_node: Node,
                      depth: int):
        if depth % self._kDimension == 0: 
            if new_node._state[0] <= root._state[0]:
                if root.left is None:
                    root.left = new_node
                else:
                    self.add_with_root(root.left, new_node, depth + 1)
            else:
                if root.right is None:
                    root.right = new_node
                else:
                    self.add_with_root(root.right, new_node, depth + 1)
        else:
            if new_node._state[1] <= root._state[1]:
                if root.left is None:
                    root.left = new_node
                else:
                    self.add_with_root(root.left, new_node, depth + 1)
            else:
                if root.right is None:
                    root.right = new_node
                else:
                    self.add_with_root(root.right, new_node, depth + 1)            

    def add_node(self, 
                 new_node: Node):
        if new_node is None:
            return

        if self._root is None:
            self._root = new_node
        self.add_with_root(self._root, new_node, 0)
        self._tree_size += 1
    
    def closer_node_in_two(self, 
                            target: Node,
                            candidate_1: Node,
                            candidate_2: Node):
        if candidate_1 is None:
            return candidate_2
        if candidate_2 is None:
            return candidate_1
        if jnp.linalg.norm(target._state - candidate_1._state) <= jnp.linalg.norm(target._state - candidate_2._state):
            return candidate_1
        return candidate_2

    def nearest_with_root(self,
                          root: Node,
                          target: Node,
                          depth: int):
        if root is None:
            return None
        
        next_subtree = None
        other_subtree = None
        if depth % self._kDimension == 0:
            if target._state[0] <= root._state[0]:
                next_subtree = root.left
                other_subtree = root.right
            else:
                next_subtree = root.right
                other_subtree = root.left
        else:
            if target._state[1] <= root._state[1]:
                next_subtree = root.left
                other_subtree = root.right
            else:
                next_subtree = root.right
                other_subtree = root.left            

        temp_res = self.nearest_with_root(next_subtree, target, depth + 1)
        cur_best = self.closer_node_in_two(target, temp_res, root)

        cur_dist = jnp.linalg.norm(target._state - cur_best._state)
        if depth % self._kDimension == 0:
            dist_to_boundary = abs(target._state[0] - root._state[0])
        else:
            dist_to_boundary = abs(target._state[1] - root._state[1])
        
        if cur_dist > dist_to_boundary:
            temp_res = self.nearest_with_root(other_subtree, target, depth + 1)
            cur_best = self.closer_node_in_two(target, temp_res, cur_best)
        return cur_best
    
    def nearest_node(self,
                    target: Node):
        return self.nearest_with_root(self._root, target, 0)

    def range_search_with_root(self,
                                root: Node,
                                parent: Node,
                                res: list,
                                x_min: float, 
                                x_max: float,
                                y_min: float,
                                y_max: float, 
                                depth: int):
        if root is None:
            return
        if depth % self._kDimension == 0 and parent is not None:
            if root._state[1] <= parent._state[1] and parent._state[1] < y_min:
                return 
            if root._state[1] > parent._state[1] and parent._state[1] > y_max:
                return 
        elif parent is not None:
            if root._state[0] <= parent._state[0] and parent._state[0] < x_min:
                return 
            if root._state[0] > parent._state[0] and parent._state[0] > x_max:
                return 
        
        if root._state[0] >= x_min and root._state[0] <= x_max and root._state[1] >= y_min and root._state[1] < y_max:
            res.append(root)
        self.range_search_with_root(root.left, root, res, x_min, x_max, y_min, y_max, depth + 1)
        self.range_search_with_root(root.right, root, res, x_min, x_max, y_min, y_max, depth + 1)
    
    def range_serach(self, 
                     x_min: float, 
                     x_max: float,
                     y_min: float,
                     y_max: float):
        res = []
        if x_min >= x_max or y_min >= y_max:
            print("Invalid range in range search")
            return res
        self.range_search_with_root(self._root, None, res, x_min, x_max, y_min, y_max)


class RRT:
    """
        RRT Implementation
    """
    def __init__(self, 
                 start_config: jnp.ndarray,
                 goal_config: jnp.ndarray,
                 map,
                 step_size: float = 0.01,
                 goal_sample_rate: int = 50,
                 max_iter: int = 500,
                 seed: int = 0
                 ):
        self._start = Node(start_config)
        self._goal = Node(goal_config)
        self._kdtree = kdTree(self._start)
        
        self._map = map
        self._resolution = map._resolution

        self._node_list = []
        
        self._step_size = step_size if step_size > self._resolution else 2.0 * self._resolution
        self._max_iter = max_iter
        self._goal_sample_rate = goal_sample_rate
        
        self._rng_key = random.PRNGKey(seed=seed)
        

    def _get_random_node(self):
        '''
            sample a collision-free random node
        '''
        self._rng_key, rng_key = random.split(self._rng_key, 2)
        
        if random.randint(rng_key, (1,), 0, 100)[0] > self._goal_sample_rate:
            rand_state = self._map.sample_free_pos()
        else:
            rand_state = self._map.sample_free_pos(toward_goal=True)
        #rand_state = self._map.sample_free_pos() 
        return Node(rand_state)
    

    @staticmethod
    def _compute_node_distance(node1: Node, node2: Node):
        return jnp.linalg.norm(node1._state - node2._state)
    
    
    def _get_nearest_node(self, rand_node):
        dlist = [RRT._compute_node_distance(rand_node, node) for node in self._node_list]
        dlist = jnp.array(dlist)
        min_idx = jnp.argmin(dlist)
        min_dist = dlist[min_idx]
        return min_idx, min_dist
    
    def _get_nearest_node_kdtree(self, rand_node):
        pass
        
        
    def _steer(self, from_node: Node, to_node: Node, step_size: float) -> Node:
        dist = jnp.linalg.norm(to_node.state - from_node.state)
        if dist <= step_size:
            new_node_state = to_node.state
        else:
            new_node_state = from_node.state + step_size * (to_node.state - from_node.state) / dist
        new_node = Node(new_node_state)
        new_node.set_parent(from_node)
        return new_node
    
    
    def _check_edge_collision(self, node1: Node, node2: Node) -> bool:
        return self._map.check_line_collision(node1.state, node2.state)
    
    
    def _check_node_collision(self, node: Node):
        return self._map.check_pos_collision(node.state)
        
        
    def plan(self, verbose=True, animation=True):
        # initialize the tree 
        self._node_list = [self._start]
        kd_tree = kdTree(self._start)

        for i in range(self._max_iter):
            # sample a random valid node
            rand_node = self._get_random_node()
            
            # search for the nearest tree node
            #nearest_ind, _ = self._get_nearest_node(rand_node)
            #nearest_node = self._node_list[nearest_ind]
            nearest_node = kd_tree.nearest_node(rand_node)
            # steer
            new_node = self._steer(nearest_node, rand_node, self._step_size)
            
            # check edge collision
            if not self._check_node_collision(new_node):
                if not self._check_edge_collision(nearest_node, new_node):
                    test_node = Node(new_node._state)
                    kd_tree.add_node(test_node)
                    #self._node_list.append(new_node)

            if animation and i % 5 == 0:
                self._draw_graph(rand_node)
                
            if self._calc_dist_to_goal(self._node_list[-1]) <= self._step_size:
                final_node = Node(self._goal.state)
                final_node.set_parent(self._node_list[-1])
                self._node_list.append(final_node)
                sol = self._generate_final_course()
                print(f'Find a feasible path with {len(sol)} nodes!')
                return sol
            else:
                if verbose and i % 10 == 0:
                    print(f"Iter: {i} || No. of Tree Nodes: {len(self._node_list)}") 

        print('Failed to find a feasible path...')
        return None
    

    def _draw_graph(self, node: Node):
        pass
    

    def _calc_dist_to_goal(self, node: Node):
        return RRT._compute_node_distance(node, self._goal)
    

    def _generate_final_course(self):
        path = [self._goal.state]
        node = self._node_list[len(self._node_list) - 1]
        while node.parent is not None:
            path.append(node.state)
            node = node.parent
        path.append(node.state)
        return path
            
            
if __name__ == '__main__':
    
    import sys
    import os
    import pathlib

    ROOT_DIR = str(pathlib.Path(__file__).parent)
    sys.path.append(ROOT_DIR)
    os.chdir(ROOT_DIR)

    from world_map import TwoDimMap
    from utils import plot_circle

    world_map = TwoDimMap([0., 2., 0., 2.], resolution=0.02)
    start = jnp.array([0.0, 0.])
    goal = jnp.array([2.0, 2.0])
    
    world_map.update_start(start)
    world_map.update_goal(goal)
    
    rng_key = random.PRNGKey(seed=4)

    for i in range(30):
        rng_key, rng_key_x, rng_key_y, rng_key_r = random.split(rng_key, 4)
        x = random.uniform(rng_key_x, shape=(1,), minval=0.1, maxval=1.75)
        y = random.uniform(rng_key_y, shape=(1,), minval=0.1, maxval=1.75)
        r = random.uniform(rng_key_r, shape=(1,), minval=0.05, maxval=0.15)
        obs = (x[0], y[0], r[0])
        world_map.add_obstacle(obs)
    
    if not world_map.check_pos_collision(start) and not world_map.check_pos_collision(goal):
        rrt = RRT(
            start_config=start,
            goal_config=goal,
            map=world_map,
            step_size=0.1,
            goal_sample_rate=0,
            seed=250,
            max_iter=500
        )
        path_solution = rrt.plan()
    else:
        path_solution = None
    
    """     kd_tree_test = kdTree(None)
    for node in rrt._node_list:
        print(node._state)
        kd_tree_test.add_node(node) """

    if path_solution is not None:
        print(f'Path Length: {len(path_solution)}')
        path = jnp.array(path_solution)
        
        sns.set_theme('notebook')
        fig, ax = plt.subplots(1, 1)
        for obs in world_map._obstacle:
            plot_circle(obs[0], obs[1], obs[2], ax)
            
        for node in rrt._node_list:
            plt.scatter(node.state[0], node.state[1], c='k', s=0.5)
            if node.parent is not None:
                plt.plot([node.state[0], node.parent.state[0]],
                          [node.state[1], node.parent.state[1]], 
                          'k-.', linewidth=0.5)
        
        for i in range(len(path_solution)-1):
            plt.scatter(path[i,0], path[i,1])
            plt.plot(path[i:i+2,0], path[i:i+2,1], linewidth=2.5)
        
        plt.scatter(path[len(path_solution)-1,0], path[len(path_solution)-1,1]) 
        
        plt.scatter(start[0], start[1], marker='*', linewidths=2)
        plt.scatter(goal[0], goal[1], marker='*', linewidths=2) 
        plt.axis('equal')
        plt.show()        
    
    