class Solution:
    def firstMissingPositive(self, nums: List[int]) -> int:
        for i in range(0, len(nums)):
            while nums[i] != i + 1:
                if nums[i] <= 0 or nums[i] > len(nums) or nums[i] == nums[nums[i] - 1]:
                    break
                idx = nums[i] - 1
                nums[i] = nums[idx]
                nums[idx] = idx + 1
        
        for i in range(0, len(nums)):
            if nums[i] != i + 1:
                return i + 1
        return len(nums) + 1
