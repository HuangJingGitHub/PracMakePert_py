class Solution:
    def combinationSum(self, candidates: List[int], target: int) -> List[List[int]]:
        res = []
        path = []

        candidates.sort()
        self.backtrace(candidates, 0, target, res, path)
        return res

    def backtrace(self, candidates: List[int], startIdx: int, target: int, res: List[List[int]], path: List[int]) -> None:
        if (target == 0):
            res.append(path.copy())
            return
        
        for i in range(startIdx, len(candidates)):
            if candidates[startIdx] <= target:
                path.append(candidates[i])
                self.backtrace(candidates, i, target - candidates[i], res, path)
                path.pop()
