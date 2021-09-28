class Solution:
    res = []
    def combine(self, n: int, k: int) -> List[List[int]]:
        self.res = []
        path = []
        self.trackback(n, k, 1, path)
        return self.res
    
    def trackback(self, n: int, k: int, begin: int, path: List[int]) -> None:
        if len(path) == k:
            self.res.append(copy.deepcopy(path))
            return
        
        for i in range(begin, n - k + len(path) + 2):
            path.append(i)
            self.trackback(n, k, i + 1, path)
            path.pop(-1)
