# backtracing + dfs
class Solution:
    def permute(self, nums: List[int]) -> List[List[int]]:
        res = []
        path = []
        visited = [False for i in range(len(nums))]

        self.dfs(nums, 0, res, path, visited)
        return res
    
    def dfs(self, nums: List[int], depth: int, res: List[List[int]], path: List[int], visited: List[bool]) -> None:
        if depth == len(nums):
            res.append(path.copy())
            return
        
        for i in range(len(nums)):
            if not visited[i]:
                path.append(nums[i])
                visited[i] = True
                self.dfs(nums, depth + 1, res, path, visited)
                path.pop()
                visited[i] = False
