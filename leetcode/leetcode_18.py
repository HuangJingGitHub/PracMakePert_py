class Solution:
    def fourSum(self, nums: List[int], target: int) -> List[List[int]]:
        nums.sort()
        res = []

        for i in range(len(nums) - 3):
            if i > 0 and nums[i] == nums[i - 1]:
                continue
            for j in range(i + 1, len(nums) - 2):
                if nums[j] == nums[j - 1] and j != i + 1:
                    continue
                partSum = nums[i] + nums[j]
                leftPt = j + 1
                rightPt = len(nums) - 1

                while leftPt < rightPt:
                    curSum = partSum + nums[leftPt] + nums[rightPt]
                    if curSum == target:
                        res.append([nums[i], nums[j], nums[leftPt], nums[rightPt]])
                        leftPt += 1
                        rightPt -= 1
                        while leftPt < rightPt and nums[leftPt] == nums[leftPt - 1]:
                            leftPt += 1
                        while rightPt > leftPt and nums[rightPt] == nums[rightPt + 1]:
                            rightPt -= 1
                    elif curSum < target:
                        leftPt += 1
                    else:
                        rightPt -= 1
        return res
