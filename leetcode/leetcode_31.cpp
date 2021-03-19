class Solution:
    def nextPermutation(self, nums: List[int]) -> None:
        """
        Do not return anything, modify nums in-place instead.
        """
        if len(nums) <= 1:
            return 
        
        i = len(nums) - 1
        while i > 0:
            if nums[i - 1] < nums[i]:
                j = len(nums) - 1
                while j >= i:
                    if nums[j] > nums[i - 1]:
                        nums[i - 1], nums[j] = nums[j], nums[i - 1]
                        break
                    j -= 1
                j = len(nums) - 1
                while i < j:
                    nums[i], nums[j] = nums[j], nums[i]
                    i += 1
                    j -= 1
                break
            i -= 1
            
        if i == 0:
            nums.reverse()
