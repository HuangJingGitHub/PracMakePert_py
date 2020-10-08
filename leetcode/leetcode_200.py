# classical DFS
class Solution:
    def numIslands(self, grid: List[List[str]]) -> int:
        res = 0
        for i in range(len(grid)):
            for j in range(len(grid[0])):
                if grid[i][j] == '1':
                    if self.DFS_ReachWater(grid, i, j):
                        res += 1

        return res
    
    def DFS_ReachWater(self, grid: List[List[str]], row: int, col: int) -> bool:
        if row < 0 or row >= len(grid) or col < 0 or col >= len(grid[0]):
            return True
        if grid[row][col] != '1':
            return True
        
        grid[row][col] = '2'

        left = self.DFS_ReachWater(grid, row, col - 1)
        right = self.DFS_ReachWater(grid, row, col + 1)
        up = self.DFS_ReachWater(grid, row - 1, col)
        down = self.DFS_ReachWater(grid, row + 1, col)

        return left and right and up and down

     
