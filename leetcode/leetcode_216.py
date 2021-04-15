class Solution:
    def combinationSum3(self, k: int, n: int) -> List[List[int]]:
        res = []
        path = []

        self.backtrace(res, path, 1, k, n)
        return res
    
    def backtrace(self, res: List[List[int]], path: list[int], start: int, k: int, target: int) -> None:
        if len(path) == k:
            if target == 0:
                res.append(path.copy())
            return
        
        for i in range(start, 10):
            if i > target:
                break
            path.append(i)
            self.backtrace(res, path, i + 1, k, target - i)
            path.pop()
            
