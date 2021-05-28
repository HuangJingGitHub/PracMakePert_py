class Solution:
    def canJump(self, nums: List[int]) -> bool:
        k = 0
        for i, num in enumerate(nums):
            if k < i:
                return False
            k = max(k, i + num)
        return True
