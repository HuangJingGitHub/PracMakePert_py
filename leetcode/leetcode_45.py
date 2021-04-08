class Solution:
    def jump(self, nums: List[int]) -> int:
        if len(nums) == 1:
            return 0
            
        pos = 0
        jump = 1
        while pos + nums[pos] < len(nums) - 1:
            for i in range(pos + 1, pos + nums[pos] + 1):
                if i + nums[i] > pos + nums[pos]:
                    pos = i
            jump += 1
        return jump
