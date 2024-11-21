import jax.numpy as jnp

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
        # self._parent = copy.deepcopy(node)
        self._parent = node

    def set_cost(self, cost):
        self._cost = cost

    def reset_parent(self, node):
        # self._parent = copy.deepcopy(node)
        self._parent = node

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
                                res: list,
                                x_min: float, 
                                x_max: float,
                                y_min: float,
                                y_max: float, 
                                depth: int):
        if root is None:
            return
        
        if root._state[0] >= x_min and root._state[0] <= x_max and root._state[1] >= y_min and root._state[1] <= y_max:
            res.append(root)

        if depth % self._kDimension == 0:
            if root._state[0] < x_min:
                self.range_search_with_root(root.right, res, x_min, x_max, y_min, y_max, depth + 1)
            elif root._state[0] > x_max:
                self.range_search_with_root(root.left, res, x_min, x_max, y_min, y_max, depth + 1)
            else:
                self.range_search_with_root(root.left, res, x_min, x_max, y_min, y_max, depth + 1)
                self.range_search_with_root(root.right, res, x_min, x_max, y_min, y_max, depth + 1)
        else:
            if root._state[1] < y_min:
                self.range_search_with_root(root.right, res, x_min, x_max, y_min, y_max, depth + 1)
            elif root._state[1] > y_max:
                self.range_search_with_root(root.left, res, x_min, x_max, y_min, y_max, depth + 1)
            else:
                self.range_search_with_root(root.left, res, x_min, x_max, y_min, y_max, depth + 1)
                self.range_search_with_root(root.right, res, x_min, x_max, y_min, y_max, depth + 1)
    
    def range_search(self, 
                     x_min: float, 
                     x_max: float,
                     y_min: float,
                     y_max: float):
        res = []
        if x_min >= x_max or y_min >= y_max:
            print("Invalid range in range search")
            return res
        self.range_search_with_root(self._root, res, x_min, x_max, y_min, y_max, 0)


        """ DEPRECATED 
        Implementation above is more compact and easy to understand.   
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
            
            if root._state[0] >= x_min and root._state[0] <= x_max and root._state[1] >= y_min and root._state[1] <= y_max:
                res.append(root)
            self.range_search_with_root(root.left, root, res, x_min, x_max, y_min, y_max, depth + 1)
            self.range_search_with_root(root.right, root, res, x_min, x_max, y_min, y_max, depth + 1)
        
        def range_search(self, 
                        x_min: float, 
                        x_max: float,
                        y_min: float,
                        y_max: float):
            res = []
            if x_min >= x_max or y_min >= y_max:
                print("Invalid range in range search")
                return res
            self.range_search_with_root(self._root, None, res, x_min, x_max, y_min, y_max, 0) """