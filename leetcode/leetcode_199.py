# Definition for a binary tree node.
# class TreeNode:
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution:
    def rightSideView(self, root: TreeNode) -> List[int]:
        res = []
        if not root:
            return res
        
        levelNode = [root]
        while levelNode:
            res.append(levelNode[-1].val)
            levelNodeNum =  len(levelNode)
            for i in range(levelNodeNum):
                if levelNode[0].left:
                    levelNode.append(levelNode[0].left)
                if levelNode[0].right:
                    levelNode.append(levelNode[0].right)
                levelNode.pop(0)

        return res
