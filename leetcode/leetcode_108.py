# Definition for a binary tree node.
# class TreeNode:
#     def __init__(self, x):
#         self.val = x
#         self.left = None
#         self.right = None

class Solution:
    def sortedArrayToBST(self, nums: List[int]) -> TreeNode:
        self.nums = nums
        if not nums:
            return None
        return self.buildBST(0, len(nums))

    def buildBST(self, start: int, end: int) -> TreeNode:
        if start >= end:
            return None
        
        mid = start + (end - start) // 2
        newNode = TreeNode(self.nums[mid])
        newNode.left = self.buildBST(start, mid)
        newNode.right = self.buildBST(mid+1, end)

        return newNode
