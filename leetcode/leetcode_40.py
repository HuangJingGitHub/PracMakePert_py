class Solution:
    def combinationSum2(self, candidates: List[int], target: int) -> List[List[int]]:
        res = []
        path = []
        candidates.sort()
        self.backtrace(0, target, candidates, res, path)
        return res
    

    def backtrace(self, startIdx: int, target: int, candidates: List[int], res: List[List[int]], path: List[int]) -> None:
        if target == 0:
            res.append(path.copy())
            return
        
        for i in range(startIdx, len(candidates)):
            if candidates[i] <= target:
                if i > startIdx and candidates[i] == candidates[i - 1]:
                    continue
                path.append(candidates[i])
                self.backtrace(i + 1, target - candidates[i], candidates, res, path)
                path.pop()
            else:
                return
