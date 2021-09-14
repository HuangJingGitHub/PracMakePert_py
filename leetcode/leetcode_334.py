class Solution:
    def increasingTriplet(self, nums: List[int]) -> bool:
        minNum = 1e10
        midNum = 1e10

        for num in nums:
            if num <= minNum:
                minNum = num
            elif num <= midNum:
                midNum = num
            else:
                return True
        return False
