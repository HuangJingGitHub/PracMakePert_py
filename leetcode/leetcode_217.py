class Solution:
    def containsDuplicate(self, nums: List[int]) -> bool:
        if len(nums) < 2:
            return False
        
        numsSet = set()
        numsSet.add(nums[0])
        for i in range(1, len(nums)):
            if nums[i] in numsSet:
                return True
            numsSet.add(nums[i])
        return False
