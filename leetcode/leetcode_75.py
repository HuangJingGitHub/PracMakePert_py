class Solution:
    def sortColors(self, nums: List[int]) -> None:
        """
        Do not return anything, modify nums in-place instead.
        """
        cnt0 = 0
        cnt1 = 0
        cnt2 = 0

        for i in range(0, len(nums)):
            if nums[i] == 0:
                nums[cnt2] = 2
                cnt2 += 1
                nums[cnt1] = 1
                cnt1 += 1
                nums[cnt0] = 0
                cnt0 += 1
            elif nums[i] == 1:
                nums[cnt2] = 2
                cnt2 += 1
                nums[cnt1] = 1
                cnt1 += 1
            else:
                nums[cnt2] = 2
                cnt2 += 1
