class Solution:
    def twoSum(self, nums: List[int], target: int) -> List[int]:
        res = []
        hashMap = {}

        for index, num in enumerate(nums):
            if num in hashMap:
                res.append(hashMap[num])
                res.append(index)
            hashMap[target - num] = index
        
        return res