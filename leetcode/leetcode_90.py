class Solution:
    res = []
    def subsetsWithDup(self, nums: List[int]) -> List[List[int]]:
        nums.sort()
        self.res = []
        path = []
        self.backtrack(0, nums, path)
        return self.res

    def backtrack(self, start: int, nums: List[int], path: List[int]) -> None:
        self.res.append(copy.deepcopy(path))

        for i in range(start, len(nums)):
            if i != start and nums[i] == nums[i - 1]:
                continue
            path.append(nums[i])
            self.backtrack(i + 1, nums, path)
            path.pop(-1)
