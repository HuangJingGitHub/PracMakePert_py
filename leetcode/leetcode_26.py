class Solution:
    def removeDuplicates(self, nums: List[int]) -> int:
        if len(nums) <= 1:
            return len(nums)
        curNum = nums[0]
        i = 1
        while i < len(nums):
            if nums[i] == curNum:
                nums.pop(i)
            else:
                curNum = nums[i]
                i += 1
        return len(nums)
      
// no pop()      
class Solution:
    def removeDuplicates(self, nums: List[int]) -> int:
        if len(nums) <= 1:
            return len(nums)
        j = 0
        for i in range(1, len(nums)):
            if (nums[j] != nums[i]):
                nums[j + 1] = nums[i]
                j += 1
        return j + 1  
