class Solution:
    res = []
    def subsets(self, nums: List[int]) -> List[List[int]]:
        self.res = []
        path = []
        self.backtrace(0, nums, path)
        return self.res
    
    def backtrace(self, begin: int, nums: List[int], path: List[int]) -> None:
        self.res.append(copy.deepcopy(path))

        for i in range(begin, len(nums)):
            path.append(nums[i])
            self.backtrace(i + 1, nums, path)
            path.pop(-1)
