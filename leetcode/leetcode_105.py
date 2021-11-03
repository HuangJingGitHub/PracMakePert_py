# Definition for a binary tree node.
# class TreeNode:
#     def __init__(self, val=0, left=None, right=None):
#         self.val = val
#         self.left = left
#         self.right = right
class Solution:
    def buildTree(self, preorder: List[int], inorder: List[int]) -> TreeNode:
         return self.helper(preorder, 0, len(preorder), inorder, 0, len(inorder))
    
    def helper(self, preorder, pStart, pEnd, inorder, iStart, iEnd):
        if pStart == pEnd:
            return None
        
        rootVal = preorder[pStart]
        root = TreeNode(rootVal)

        inorderRootIdx = 0
        for i in range(iStart, iEnd):
            if inorder[i] == rootVal:
                inorderRootIdx = i
                break
        
        leftNum = inorderRootIdx - iStart
        root.left = self.helper(preorder, pStart + 1, pStart + leftNum + 1, inorder, iStart, inorderRootIdx)
        root.right = self.helper(preorder, pStart + leftNum + 1, pEnd, inorder, inorderRootIdx + 1, iEnd)

        return root
