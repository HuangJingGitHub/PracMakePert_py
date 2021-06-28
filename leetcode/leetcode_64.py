class Solution:
    def minPathSum(self, grid: List[List[int]]) -> int:
        m = len(grid)
        n = len(grid[0])

        for col in range(1, n):
            grid[0][col] += grid[0][col - 1]
        for row in range(1, m):
            grid[row][0] += grid[row - 1][0]
        
        for row in range(1, m):
            for col in range(1, n):
                grid[row][col] += min(grid[row][col - 1], grid[row - 1][col])
        return grid[-1][-1]
